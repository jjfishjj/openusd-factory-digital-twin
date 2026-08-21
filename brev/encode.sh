#!/usr/bin/env bash
# Encode a path-traced PNG sequence into an mp4 and a looping gif.
# Usage: encode.sh <png-dir> <out-basename> [fps]
set -euo pipefail
DIR="${1:?png dir}"; OUT="${2:?out basename}"; FPS="${3:-12}"

PNGS=$(find "$DIR" -name 'rgb_*.png' | sort | head -1)
[ -z "$PNGS" ] && { echo "no rgb_*.png in $DIR"; exit 1; }

ffmpeg -y -framerate "$FPS" -pattern_type glob -i "$DIR/rgb_*.png" \
  -c:v libx264 -pix_fmt yuv420p -crf 18 -movflags +faststart "${OUT}.mp4"

ffmpeg -y -i "${OUT}.mp4" -vf \
  "fps=${FPS},scale=720:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" \
  -loop 0 "${OUT}.gif"

ls -lh "${OUT}.mp4" "${OUT}.gif"
