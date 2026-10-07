# Converting other BVH skeletons to SOMA

The retargeter reads only SOMA-skeleton BVH files. `app/tools/bvh_to_soma.py` moves motions recorded on another
skeleton onto the SOMA skeleton, so they can then be retargeted to any robot like a SOMA motion.

## Supported sources

| `--source` | Skeleton | Notes |
|---|---|---|
| `cmu` (default) | [CMU Graphics Lab Motion Capture Database](http://mocap.cs.cmu.edu/), MotionBuilder-friendly BVH conversion (cgspeed, 2008/2010 releases) | Every file starts with a T-pose frame facing +Z. It is used for calibration and left out of the output. |

## Convert

From the repository root, convert one file:

```bash
uv run python app/tools/bvh_to_soma.py --input 13_11.bvh --output output/soma/13_11.bvh
```

or every `.bvh` below a folder, keeping the subfolders:

```bash
uv run python app/tools/bvh_to_soma.py --input cmu/13 --output output/soma/13
```

The output keeps the source frame rate (120 FPS for CMU). Retarget it like any SOMA BVH, for example with
[batch retargeting](batch-retargeting.md) pointed at the output folder.

## How it works

1. **Calibration.** Both skeletons need to stand in the same pose in one frame. The SOMA zero pose has the arms down
   and the elbows bent forward, so its arms are first raised to match the source's calibration pose. Each upper arm,
   forearm and hand bone is turned onto the matching source bone. The forearm and hand are also turned so the thumb
   side of the hand points the same way: on SOMA the line across the knuckles from the pinky to the index finger, on
   CMU forward, because the CMU T-pose has the palms down. This keeps forearm twist (palm direction) right. The
   spine, clavicles, legs, feet and head stand the same way in both poses and are left as they are. (CMU's clavicle
   bone starts at the spine's center line and rises about 20° in its T-pose; lining SOMA's up with it would raise
   the shoulders.)
2. **Rotations.** Every frame, each SOMA joint takes the world rotation of its source joint, offset by the difference
   between the two skeletons in the calibration pose. This copies the motion while keeping SOMA's bone lengths and
   joint frames.
3. **Translation.** The hips position is scaled by the ratio of the two skeletons' leg lengths (hip to ankle), and
   shifted so the hips height matches in the calibration pose.
4. SOMA joints without a source joint (fingers, eyes, jaw) keep their zero-pose rotations.

| SOMA | CMU |
|---|---|
| Hips | Hips |
| Spine1, Spine2, Chest | LowerBack, Spine, Spine1 |
| Neck1, Neck2, Head | Neck, Neck1, Head |
| Left/RightShoulder, Arm, ForeArm, Hand | Left/RightShoulder, Arm, ForeArm, Hand |
| Left/RightLeg, Shin, Foot, ToeBase | Left/RightUpLeg, Leg, Foot, ToeBase |

## Checking the result

Bone directions in the output follow the source closely: upper arm, forearm and hand match exactly, legs within
about 1°, and the spine within a few degrees, because the two skeletons split the spine differently. Clavicles and
feet keep a small constant difference, because the skeletons' shapes differ there while both stand naturally in the
calibration pose. Palm direction follows the source's hand; CMU's separately animated thumb joints are not used.
Load the output in the interactive viewer to check it before retargeting a large batch.

The CMU data has its own terms of use; see [mocap.cs.cmu.edu](http://mocap.cs.cmu.edu/).
