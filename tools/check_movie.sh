#!/usr/bin/env bash
# Checks the H.264 encoder against a real decoder: renders a short noisy sequence, writes movies with
# several settings, decodes them with ffmpeg and requires the decoded pictures to equal the encoder's
# reconstruction byte for byte (so P frames can't drift in any player).
# usage: tools/check_movie.sh <path to hdc> <examples dir>
set -euo pipefail
HDC="$1"
EX="$2"
command -v ffmpeg >/dev/null || { echo "ffmpeg not installed"; exit 1; }
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
# few samples and no denoising: noisy frames exercise every coefficient path
"$HDC" render "$EX/smoke.hdg" "$work/f.png" --frames 1 14 --samples 2 --no-denoise --size 322 182 > /dev/null
"$HDC" render "$EX/bounce.hdg" "$work/b.png" --frames 1 14 --samples 4 --size 256 144 > /dev/null
fail=0
check() { # name frames-pattern out qp extra...
    local name="$1" first="$2" out="$3" w="$4" h="$5"; shift 5
    "$HDC" movie "$first" "$work/$out" --recon "$work/recon.yuv" "$@" > /dev/null
    ffmpeg -v error -y -i "$work/$out" -f rawvideo -pix_fmt yuv420p "$work/dec.yuv"
    local want=$(( w * h * 3 / 2 * 14 ))
    if [ "$(stat -c %s "$work/dec.yuv")" != "$want" ]; then echo "FAIL $name: decoded size $(stat -c %s "$work/dec.yuv"), expected $want"; fail=1; return; fi
    if cmp -s "$work/dec.yuv" "$work/recon.yuv"; then echo "ok   $name"; else echo "FAIL $name: decoder output differs from the encoder's"; fail=1; fi
}
check "smoke mp4 qp23 (size not a multiple of 16)" "$work/f.0001.png" a.mp4 322 182 --qp 23
check "smoke mov qp12 keyint 5" "$work/f.0001.png" a.mov 322 182 --qp 12 --keyint 5
check "smoke mp4 qp0" "$work/f.0001.png" b.mp4 322 182 --qp 0 --slices 3
check "smoke mp4 qp45" "$work/f.0001.png" c.mp4 322 182 --qp 45 --slices 1
check "bounce mp4 qp28" "$work/b.0001.png" d.mp4 256 144 --quality small
check "bounce mov high" "$work/b.0001.png" e.mov 256 144 --quality high --fps 30
ffprobe -v error -show_entries stream=codec_name,profile,width,height,r_frame_rate,nb_frames -of compact "$work/e.mov"
exit $fail
