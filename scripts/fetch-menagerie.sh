#!/bin/sh
# Fetch the Franka Emika Panda scene from MuJoCo Menagerie (sparse checkout,
# pinned commit) into third_party/menagerie/franka_emika_panda. The scene is
# consumed byte-identical — this project never patches it.
set -eu

PIN=${MENAGERIE_PIN:-71f066ad0be9cd271f7ed58c030243ef157af9f4}
REPO=https://github.com/google-deepmind/mujoco_menagerie.git
ROOT=$(cd "$(dirname "$0")/.." && pwd)
DEST="$ROOT/third_party/menagerie"

if [ -f "$DEST/franka_emika_panda/scene.xml" ]; then
  echo "already fetched: $DEST/franka_emika_panda"
  exit 0
fi

rm -rf "$DEST"
git clone --filter=blob:none --no-checkout "$REPO" "$DEST"
git -C "$DEST" sparse-checkout set franka_emika_panda
git -C "$DEST" checkout "$PIN"
echo "fetched: $DEST/franka_emika_panda ($(ls "$DEST/franka_emika_panda/assets" | wc -l) mesh files)"
