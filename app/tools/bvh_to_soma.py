# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Convert BVH motions from another skeleton to the SOMA base skeleton, so they can be retargeted to robots.

The retargeter only reads SOMA-skeleton BVH files. This tool moves a motion onto the SOMA skeleton: each SOMA joint
takes the world rotation of its matching source joint, measured from a calibration frame in which both skeletons
stand in the same pose. The SOMA zero pose has the arms down with the elbows bent, so for the calibration the SOMA
arms are first raised to match the source's pose (bone directions, plus the thumb side of the hand for forearm twist).
Hips translation is scaled by the ratio of the two skeletons' leg lengths. SOMA joints without a source joint
(fingers, eyes, jaw) keep their zero-pose rotations.

Supported sources:

- ``cmu``: the CMU Graphics Lab Motion Capture Database in its MotionBuilder-friendly BVH conversion (cgspeed,
  B. Hahne). Every file starts with a T-pose frame facing +Z, which is used for calibration and dropped.

Usage (from the repository root):

    uv run python app/tools/bvh_to_soma.py --input 13_11.bvh --output 13_11_soma.bvh
    uv run python app/tools/bvh_to_soma.py --input cmu/13 --output soma/13   # every .bvh below a folder
"""

from __future__ import annotations

import argparse
import pathlib
import sys
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation

SOMA_ZERO_POSE = (
    pathlib.Path(__file__).resolve().parents[2] / "soma_retargeter" / "assets" / "soma" / "soma_zero_frame0.bvh"
)


# BVH reading and writing


@dataclass
class Bvh:
    """A BVH file. End sites are joints named ``<parent>_End`` with no channels."""

    names: list[str]
    parents: list[int]
    offsets: np.ndarray  # (joints, 3)
    channels: list[list[str]]
    frames: np.ndarray  # (frames, channel values)
    frame_time: float
    hierarchy_text: str = ""

    def index(self, name: str) -> int:
        try:
            return self.names.index(name)
        except ValueError:
            raise ValueError(f"Joint '{name}' is not in the skeleton.") from None


def read_bvh(path: str | pathlib.Path) -> Bvh:
    text = pathlib.Path(path).read_text()
    hierarchy_text, _, motion_text = text.partition("MOTION")
    if not motion_text:
        raise ValueError(f"{path} has no MOTION section.")

    names, parents, offsets, channels = [], [], [], []
    stack, current = [], -1
    for line in hierarchy_text.splitlines():
        tokens = line.split()
        if not tokens:
            continue
        if tokens[0] in ("ROOT", "JOINT", "End"):
            names.append(tokens[1] if tokens[0] != "End" else f"{names[stack[-1]]}_End")
            parents.append(stack[-1] if stack else -1)
            offsets.append(np.zeros(3))
            channels.append([])
            current = len(names) - 1
        elif tokens[0] == "{":
            stack.append(current)
        elif tokens[0] == "}":
            stack.pop()
        elif tokens[0] == "OFFSET":
            offsets[current] = np.array([float(v) for v in tokens[1:4]])
        elif tokens[0] == "CHANNELS":
            channels[current] = tokens[2:]

    lines = [line for line in motion_text.splitlines() if line.strip()]
    num_frames = int(lines[0].split(":")[1])
    frame_time = float(lines[1].split(":")[1])
    frames = np.array([[float(v) for v in line.split()] for line in lines[2 : 2 + num_frames]])
    expected = sum(len(c) for c in channels)
    if frames.shape != (num_frames, expected):
        raise ValueError(f"{path}: expected {num_frames} frames of {expected} values, found {frames.shape}.")
    return Bvh(names, parents, np.array(offsets), channels, frames, frame_time, hierarchy_text)


def _rotation_order(channels: list[str]) -> str:
    return "".join(c[0].upper() for c in channels if c.endswith("rotation"))


def forward_kinematics(bvh: Bvh, frame: np.ndarray) -> tuple[list[Rotation], np.ndarray]:
    """World rotations and positions of every joint for one frame of channel values.

    Position channels replace the joint's offset, as in the SOMA files.
    """
    world_rot, world_pos = [], np.zeros((len(bvh.names), 3))
    column = 0
    for j, channels in enumerate(bvh.channels):
        values = frame[column : column + len(channels)]
        column += len(channels)
        position = bvh.offsets[j].copy()
        if any(c.endswith("position") for c in channels):
            position = np.array([v for c, v in zip(channels, values) if c.endswith("position")])
        order = _rotation_order(channels)
        local = (
            Rotation.from_euler(order, [v for c, v in zip(channels, values) if c.endswith("rotation")], degrees=True)
            if order
            else Rotation.identity()
        )
        parent = bvh.parents[j]
        if parent < 0:
            world_rot.append(local)
            world_pos[j] = position
        else:
            world_rot.append(world_rot[parent] * local)
            world_pos[j] = world_pos[parent] + world_rot[parent].apply(position)
    return world_rot, world_pos


def write_bvh(path: pathlib.Path, template: Bvh, frames: np.ndarray, frame_time: float) -> None:
    """Write frames of channel values under the template's hierarchy."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(template.hierarchy_text.rstrip() + "\n")
        f.write(f"MOTION\nFrames: {len(frames)}\nFrame Time: {frame_time:.6f}\n")
        for row in frames:
            f.write(" ".join(f"{v:.6f}" for v in row) + "\n")


# Source skeletons


@dataclass
class SourceRig:
    """How a source skeleton maps onto SOMA.

    ``joints`` maps each driven SOMA joint to the source joint whose rotation it takes. Joints listed in ``bones``
    have their SOMA bone (joint to child) turned onto the source bone for calibration. Those also in ``radial`` are
    turned so the thumb side of the hand points the same way too, which sets forearm and hand twist (palm direction):
    on SOMA it is the line across the knuckles from the pinky to the index finger, on the source a world direction in
    the calibration pose. Other joints are taken to stand the same way in both calibration poses (upright, facing
    +Z, shoulders relaxed).
    """

    joints: dict[str, str]
    bones: dict[str, tuple[str, str]] = field(default_factory=dict)  # SOMA joint: (SOMA child, source child)
    radial: dict[str, tuple[tuple[str, str], tuple[float, float, float]]] = field(default_factory=dict)
    leg: tuple[tuple[str, str, str], tuple[str, str, str]] = (("", "", ""), ("", "", ""))  # (hip, knee, ankle)
    calibration_frame: int = 0
    first_frame: int = 0


def _sides(table: dict) -> dict:
    """Expand ``{Side}`` (Left, Right) and ``{S}`` (L, R) in keys and values, including inside tuples."""

    def expand(value, side: str):
        if isinstance(value, str):
            return value.format(Side=side, S=side[0])
        if isinstance(value, tuple):
            return tuple(expand(v, side) for v in value)
        return value

    return {expand(k, side): expand(v, side) for side in ("Left", "Right") for k, v in table.items()}


CMU = SourceRig(
    joints={
        "Hips": "Hips",
        "Spine1": "LowerBack",
        "Spine2": "Spine",
        "Chest": "Spine1",
        "Neck1": "Neck",
        "Neck2": "Neck1",
        "Head": "Head",
        **_sides(
            {
                "{Side}Shoulder": "{Side}Shoulder",
                "{Side}Arm": "{Side}Arm",
                "{Side}ForeArm": "{Side}ForeArm",
                "{Side}Hand": "{Side}Hand",
                "{Side}Leg": "{Side}UpLeg",
                "{Side}Shin": "{Side}Leg",
                "{Side}Foot": "{Side}Foot",
                "{Side}ToeBase": "{Side}ToeBase",
            }
        ),
    },
    bones=_sides(
        {
            "{Side}Arm": ("{Side}ForeArm", "{Side}ForeArm"),
            "{Side}ForeArm": ("{Side}Hand", "{Side}Hand"),
            "{Side}Hand": ("{Side}HandMiddle1", "{Side}HandIndex1"),
        }
    ),
    # The T-pose has the palms down, so the thumb side of each hand points forward.
    radial=_sides(
        {
            "{Side}ForeArm": (("{Side}HandPinky1", "{Side}HandIndex1"), (0.0, 0.0, 1.0)),
            "{Side}Hand": (("{Side}HandPinky1", "{Side}HandIndex1"), (0.0, 0.0, 1.0)),
        }
    ),
    leg=(("LeftLeg", "LeftShin", "LeftFoot"), ("LeftUpLeg", "LeftLeg", "LeftFoot")),
    calibration_frame=0,
    first_frame=1,
)

SOURCES = {"cmu": CMU}


# Retargeting


def _unit(v: np.ndarray) -> np.ndarray:
    return v / np.linalg.norm(v)


def _shortest_arc(a: np.ndarray, b: np.ndarray) -> Rotation:
    """Smallest rotation turning direction ``a`` onto direction ``b``."""
    a, b = _unit(a), _unit(b)
    axis = np.cross(a, b)
    sin, cos = np.linalg.norm(axis), float(np.dot(a, b))
    if sin < 1e-9:
        if cos > 0:
            return Rotation.identity()
        perpendicular = np.cross(a, [1.0, 0.0, 0.0] if abs(a[0]) < 0.9 else [0.0, 1.0, 0.0])
        return Rotation.from_rotvec(np.pi * _unit(perpendicular))
    return Rotation.from_rotvec(axis / sin * np.arctan2(sin, cos))


def _two_vector_alignment(a1: np.ndarray, a2: np.ndarray, b1: np.ndarray, b2: np.ndarray) -> Rotation:
    """Rotation turning ``a1`` onto ``b1`` exactly and ``a2`` as close to ``b2`` as that allows."""

    def frame(primary, secondary):
        x = _unit(primary)
        y = _unit(secondary - np.dot(secondary, x) * x)
        return np.stack([x, y, np.cross(x, y)], axis=1)

    return Rotation.from_matrix(frame(b1, b2) @ frame(a1, a2).T)


def _leg_length(pos: np.ndarray, bvh: Bvh, chain: tuple[str, str, str]) -> float:
    hip, knee, ankle = (pos[bvh.index(n)] for n in chain)
    return float(np.linalg.norm(knee - hip) + np.linalg.norm(ankle - knee))


def retarget_to_soma(source: Bvh, rig: SourceRig, soma: Bvh) -> np.ndarray:
    """Channel values for the SOMA skeleton, one row per source frame from ``rig.first_frame``."""
    soma_rot0, soma_pos0 = forward_kinematics(soma, soma.frames[0])
    src_rot_cal, src_pos_cal = forward_kinematics(source, source.frames[rig.calibration_frame])

    # Per driven joint, the constant rotation from the source joint's frame to the SOMA joint's frame when both
    # skeletons stand in the calibration pose: soma_world = source_world * offset.
    offsets = {}
    for soma_name, source_name in rig.joints.items():
        s, c = soma.index(soma_name), source.index(source_name)
        align = Rotation.identity()
        if soma_name in rig.bones:
            soma_child, source_child = rig.bones[soma_name]
            soma_bone = soma_pos0[soma.index(soma_child)] - soma_pos0[s]
            source_bone = src_pos_cal[source.index(source_child)] - src_pos_cal[c]
            if soma_name in rig.radial:
                (soma_from, soma_to), source_radial = rig.radial[soma_name]
                soma_radial = soma_pos0[soma.index(soma_to)] - soma_pos0[soma.index(soma_from)]
                align = _two_vector_alignment(soma_bone, soma_radial, source_bone, np.array(source_radial))
            else:
                align = _shortest_arc(soma_bone, source_bone)
        offsets[soma_name] = src_rot_cal[c].inv() * align * soma_rot0[s]

    scale = _leg_length(soma_pos0, soma, rig.leg[0]) / _leg_length(src_pos_cal, source, rig.leg[1])
    hips_s, hips_c = soma.index("Hips"), source.index(rig.joints["Hips"])
    height_offset = soma_pos0[hips_s, 1] - scale * src_pos_cal[hips_c, 1]

    # Zero-pose local rotations, kept by joints the source does not drive.
    zero_local = [
        soma_rot0[j] if p < 0 else soma_rot0[p].inv() * soma_rot0[j] for j, p in enumerate(soma.parents)
    ]

    rows = []
    for frame in source.frames[rig.first_frame :]:
        src_rot, src_pos = forward_kinematics(source, frame)
        world: list[Rotation] = []
        row = []
        for j, (name, parent) in enumerate(zip(soma.names, soma.parents)):
            if name in rig.joints:
                world.append(src_rot[source.index(rig.joints[name])] * offsets[name])
            else:
                world.append(zero_local[j] if parent < 0 else world[parent] * zero_local[j])
            channels = soma.channels[j]
            if not channels:
                continue
            local = world[j] if parent < 0 else world[parent].inv() * world[j]
            if any(c.endswith("position") for c in channels):
                if name == "Hips":
                    position = scale * src_pos[hips_c] + np.array([0.0, height_offset, 0.0])
                else:
                    position = soma.offsets[j] if parent >= 0 else np.zeros(3)
                row.extend(position)
            order = _rotation_order(channels)
            if order:
                row.extend(local.as_euler(order, degrees=True))
        rows.append(row)
    return np.array(rows)


def convert(input_path: pathlib.Path, output_path: pathlib.Path, rig: SourceRig, soma: Bvh) -> None:
    source = read_bvh(input_path)
    frames = retarget_to_soma(source, rig, soma)
    write_bvh(output_path, soma, frames, source.frame_time)
    print(f"[INFO]: {input_path} -> {output_path} ({len(frames)} frames @ {1.0 / source.frame_time:.0f} FPS)")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Convert BVH motions from another skeleton to the SOMA skeleton.")
    parser.add_argument("--input", required=True, type=pathlib.Path, help="A .bvh file, or a folder of them.")
    parser.add_argument("--output", required=True, type=pathlib.Path, help="Output .bvh file, or output folder.")
    parser.add_argument("--source", default="cmu", choices=sorted(SOURCES), help="Skeleton of the input files.")
    args = parser.parse_args(argv)

    rig = SOURCES[args.source]
    soma = read_bvh(SOMA_ZERO_POSE)
    if args.input.is_dir():
        inputs = sorted(args.input.rglob("*.bvh"))
        if not inputs:
            print(f"[ERROR]: No .bvh files below {args.input}.", file=sys.stderr)
            return 1
        for path in inputs:
            convert(path, args.output / path.relative_to(args.input), rig, soma)
    else:
        convert(args.input, args.output, rig, soma)
    return 0


if __name__ == "__main__":
    sys.exit(main())
