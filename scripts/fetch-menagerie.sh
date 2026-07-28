#!/bin/sh
# Fetch the robot scenes this app ships from MuJoCo Menagerie (sparse checkout,
# pinned commit) into third_party/menagerie/<dir>. Every scene is consumed
# byte-identical — this project never patches a Menagerie file.
#
# ADDING A ROBOT: append its Menagerie directory to DIRS below. Nothing else in
# this script changes; the early-exit and the sparse-checkout set are both
# derived from that one list.
set -eu

PIN=${MENAGERIE_PIN:-71f066ad0be9cd271f7ed58c030243ef157af9f4}
REPO=https://github.com/google-deepmind/mujoco_menagerie.git
ROOT=$(cd "$(dirname "$0")/.." && pwd)
DEST="$ROOT/third_party/menagerie"

# Menagerie directory names, which are NOT our scene ids: `franka_emika_panda`
# stages as `franka` and `robotstudio_so101` as `so101` (CMakeLists.txt owns
# that mapping). Keep both spellings straight when adding a robot.
DIRS="franka_emika_panda robotstudio_so101"

# EVERY directory must be present, not just the first. The previous form tested
# only franka_emika_panda/scene.xml, so a checkout made before the SO101 was
# added would report "already fetched" forever and never widen — the build then
# fails much later, in CMake, with a message about a scene nobody asked for.
missing=""
for d in $DIRS; do
  if [ ! -f "$DEST/$d/scene.xml" ]; then
    missing="$missing $d"
  fi
done
if [ -z "$missing" ]; then
  echo "already fetched:$(for d in $DIRS; do printf ' %s' "$DEST/$d"; done)"
  exit 0
fi
if [ -d "$DEST" ]; then
  echo "widening sparse checkout for:$missing"
fi

rm -rf "$DEST"
git clone --filter=blob:none --no-checkout "$REPO" "$DEST"
# Unquoted on purpose: DIRS is a word list, one sparse-checkout path each.
# shellcheck disable=SC2086
git -C "$DEST" sparse-checkout set $DIRS
git -C "$DEST" checkout "$PIN"
for d in $DIRS; do
  echo "fetched: $DEST/$d ($(ls "$DEST/$d/assets" | wc -l) mesh files)"
done
