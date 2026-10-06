# SPDX-License-Identifier: Apache-2.0

"""
Download the Asimov 1 robot description used by the ``asimov_1`` target.

The Asimov 1 URDF and meshes are not redistributed with SOMA Retargeter. This script
downloads them, unmodified, from a pinned commit of the upstream repository (through a fork
that preserves that commit, falling back to upstream) into the target's ``desc/`` directory,
together with upstream's license files, and verifies every file against a pinned SHA-256
checksum. Files that are already present and valid are skipped.

The downloaded files remain subject to the upstream licenses (see the license files placed
next to them). The ``desc/`` directory is ignored by git and excluded from package builds.

Usage:
    uv run python app/tools/fetch_asimov_assets.py [--force]
"""

import argparse
import hashlib
import os
import pathlib
import sys
import tempfile
import urllib.error
import urllib.request

from soma_retargeter.robotics.robot_registry import get_robot_asset_root

TARGET          = "asimov_1"
UPSTREAM_REPO   = "menloresearch/asimov-1"
UPSTREAM_COMMIT = "f71f3fe87ddfa6b7aac2a5cddeaad5dbbc19cca7"
RAW_URL         = "https://raw.githubusercontent.com/{repo}/{commit}/{path}"

# Repositories to download from, in order. The fork keeps the pinned commit available even if
# upstream is deleted or rewritten; upstream is the fallback. The checksums below apply to both.
SOURCES = ("Dene33/asimov-1", UPSTREAM_REPO)

# Upstream path -> SHA-256. Files under sim-model/ keep their upstream layout below desc/ so the
# URDF's relative mesh references (../assets/meshes/...) resolve without modifying the URDF.
FILES = {
    "sim-model/urdf/asimov_1.urdf":                    "b080188264eeac00cd4fe69d5d0735371b7079664615503ca5e7ee63a28afd99",
    "sim-model/assets/meshes/ANKLE_B_NEW.STL":         "5379b4e0a89e67dfe862f50ef22e3b091588d9c21542ef199b5a65ef6614262b",
    "sim-model/assets/meshes/IMU_ORIGIN.STL":          "4103db420f4b511026f9967711366852243e80904909ff491300117e796194de",
    "sim-model/assets/meshes/LEFT_ANKLE_A.STL":        "00dd23a465b135d15ef991b46bb314bf801c0b03c7aada1db8d1f9e17d9568a2",
    "sim-model/assets/meshes/LEFT_ELBOW.STL":          "fc697105abe95a3a3b23356527f7180113055d6168f40e66472c834eab485375",
    "sim-model/assets/meshes/LEFT_HIP_PITCH.STL":      "b63a1752cbae0cc09b140052df32c8ea45e14c0ebaea5d53ad317c8c90246d9a",
    "sim-model/assets/meshes/LEFT_HIP_ROLL.STL":       "7bd909211765b2a133354677bc6ee09a38f290d8afab0e325b419ff9db79a12c",
    "sim-model/assets/meshes/LEFT_HIP_YAW.STL":        "ef18971b3c0aae2e06ad66a1e4c3a98440163d337e3ffa3750a1c3969e40a422",
    "sim-model/assets/meshes/LEFT_KNEE.STL":           "c1429554f677ad70f0d1c26969079ad19add24c08c2c0ce29ed2c31c1a1c41f1",
    "sim-model/assets/meshes/LEFT_SHOULDER_PITCH.STL": "5e7c93c3f5f7e60f7cc139da16237507915db84fa3a6d67bc6b07158b77e7e23",
    "sim-model/assets/meshes/LEFT_SHOULDER_ROLL.STL":  "539c9175e7b60d0df3ffd6d9620825244ae684967a63aec05d3da7aac7879237",
    "sim-model/assets/meshes/LEFT_SHOULDER_YAW.STL":   "82df3e536649a1d4bb218c157de154de4a94736ae195dc974e4ab97b8e86caa5",
    "sim-model/assets/meshes/LEFT_WRIST_YAW.STL":      "966dd01e04b12fb5f5d46720d8fd82a094f3f8a8404f643c9b9c5b0c01018c38",
    "sim-model/assets/meshes/NECK_PITCH.STL":          "6467104a3890738a7dd4f946c1533d0f136d75ad6566e9627f57f64cda874798",
    "sim-model/assets/meshes/NECK_YAW.STL":            "47b311a7138ebae9326216e5b79d85cf2750f0283a133188eaf5b9a14670a7c7",
    "sim-model/assets/meshes/RIGHT_ANKLE_A.STL":       "351c68c81211de8ff1bb0dbeea7e933335ec3a1a9035446f48829c4029a42d5f",
    "sim-model/assets/meshes/RIGHT_ELBOW.STL":         "aa3df244f5dbe2404911005d288f658814709f0ff64b8c03d0a5af3079f5e6c6",
    "sim-model/assets/meshes/RIGHT_HIP_PITCH.STL":     "3d7b4b69b2e5f13baaf2c822a3965c9e1db16a444b375f66813be57c6d1800a4",
    "sim-model/assets/meshes/RIGHT_HIP_ROLL.STL":      "c4b84b8450d8e2830217f69b22999d10795c15fd72a7f99382fd1abcccda65b1",
    "sim-model/assets/meshes/RIGHT_HIP_YAW.STL":       "664a399f5b9a15cd11567cf08cf20e6fb386ff48f3ac741cd1f25848e237276e",
    "sim-model/assets/meshes/RIGHT_KNEE.STL":          "64d35ef5e9cb1d1a57e2232d670cb798d2aa7a6985bdafdaed7b3dd125aba7fe",
    "sim-model/assets/meshes/RIGHT_SHOULDER_PITCH.STL": "325c6cf9b79c09f53ec05cf5fd7c89e2d430d85f4fa2fe2ae5827879437d9531",
    "sim-model/assets/meshes/RIGHT_SHOULDER_ROLL.STL": "7ea6c316d2292a5e929f91963fde0a72e378343758575c7652a40cbf8d3b51b3",
    "sim-model/assets/meshes/RIGHT_SHOULDER_YAW.STL":  "3bf54ab880e06c09f841d640374e9a91a99d3146cd2c5df98a79487c0c357f25",
    "sim-model/assets/meshes/RIGHT_WRIST_YAW.STL":     "f71c5a4e4b762bd141477ac6b46af9598300630ce923f67f6c02bf98d4a67586",
    "sim-model/assets/meshes/WAIST_YAW.STL":           "24d5000c0a90bc5b053c6734ce517d8d23e9957d3ee59994e1b65d3a1db52a7d",
    "HARDWARE-LICENSE.txt":                            "253ad3f89603e728abfa60c36fbcaf8225cf55c1eab12725f19fb3d74d647f3a",
    "SOFTWARE-LICENSE.txt":                            "4f2416509c30c4f0c4de101cc30c571e3be33fdb5c42cabc0d2915e8ed5ea51e",
}


def _local_path(desc_dir: pathlib.Path, upstream_path: str) -> pathlib.Path:
    """Map an upstream path to its location below *desc_dir* (``sim-model/`` prefix dropped)."""
    rel = pathlib.PurePosixPath(upstream_path)
    if rel.parts[0] == "sim-model":
        rel = rel.relative_to("sim-model")
    return desc_dir.joinpath(*rel.parts)


def _sha256(path: pathlib.Path) -> str:
    """Return the hex SHA-256 digest of the file at *path*."""
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _download(url: str, dst: pathlib.Path, expected_sha256: str) -> None:
    """Download *url* to *dst*, replacing it only if the checksum matches *expected_sha256*."""
    dst.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(dir=dst.parent, prefix=f".{dst.name}.", suffix=".part")
    tmp = pathlib.Path(tmp_name)
    try:
        with os.fdopen(fd, "wb") as out, urllib.request.urlopen(url, timeout=60) as response:
            for chunk in iter(lambda: response.read(1 << 20), b""):
                out.write(chunk)
        actual = _sha256(tmp)
        if actual != expected_sha256:
            raise ValueError(f"checksum mismatch for {url}: expected {expected_sha256}, got {actual}")
        tmp.replace(dst)
    finally:
        tmp.unlink(missing_ok=True)


def main():
    """Entry point: download (or verify) every pinned Asimov 1 description file."""
    parser = argparse.ArgumentParser(description="Download the Asimov 1 robot description for the asimov_1 target.")
    parser.add_argument("--force", action="store_true", help="Download every file again, even if a valid copy exists.")
    args = parser.parse_args()

    desc_dir = pathlib.Path(get_robot_asset_root(TARGET)) / "desc"
    print(f"[INFO]: Fetching {UPSTREAM_REPO}@{UPSTREAM_COMMIT[:7]} (from {', '.join(SOURCES)}) into {desc_dir}")

    downloaded = skipped = 0
    for upstream_path, expected_sha256 in FILES.items():
        dst = _local_path(desc_dir, upstream_path)
        if not args.force and dst.is_file() and _sha256(dst) == expected_sha256:
            skipped += 1
            continue
        errors = []
        for repo in SOURCES:
            url = RAW_URL.format(repo=repo, commit=UPSTREAM_COMMIT, path=upstream_path)
            try:
                _download(url, dst, expected_sha256)
                break
            except (urllib.error.URLError, OSError, ValueError) as error:
                errors.append(f"{repo}: {error}")
        else:
            print(f"[ERROR]: Could not fetch {upstream_path}:\n  " + "\n  ".join(errors))
            sys.exit(1)
        downloaded += 1
        print(f"[INFO]: {upstream_path} ({repo})")

    print(f"[INFO]: Done: {downloaded} downloaded, {skipped} already up to date.")
    print("[INFO]: These files are third-party assets covered by the upstream license files placed next to them; "
          "they are for local use and are not part of this repository.")


if __name__ == "__main__":
    main()
