#!/usr/bin/env bash
# Convert source audio in audio/ to ComputerCraft DFPWM1a.
#
# Speakers play 48 kHz 8-bit PCM via speaker.playAudio. CC: Tweaked stores
# that as DFPWM (1 bit/sample). The computer cannot decode MP3, so conversion
# happens here — only the .dfpwm files are deployed.
#
#   audio/stop.mp3  ->  audio/stop.dfpwm
#
# Requires FFmpeg 5.1+ (native dfpwm encoder). If `ffmpeg` is not on PATH,
# a copy bundled via `uvx --from imageio-ffmpeg` is used when `uvx` exists.
#
# Canonical convert path (CC docs, music.madefor.cc):
#   ffmpeg -i in.mp3 -ac 1 -ar 48000 -c:a dfpwm out.dfpwm
# music.madefor.cc also resamples to 48 kHz mono, then encodes DFPWM1a
# from float PCM * 127. Neither tool leaves peak headroom.
#
# DFPWM is 1 bit/sample. A source already at 0 dBFS makes the predictor
# slam between ±127, which sounds like clipping. CC then re-encodes the
# decoded PCM for the client. We high-pass DC and loudnorm with a true-peak
# ceiling of -6 dBTP so the 1-bit stage has room to work.
# https://wiki.vexatos.com/dfpwm
# https://tweaked.cc/library/cc.audio.dfpwm.html
# https://tweaked.cc/guide/speaker_audio.html
# https://music.madefor.cc/
#
#   ./convert-audio.sh          # skip dfpwm newer than the source
#   ./convert-audio.sh --force  # rebuild every clip
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUDIO="$ROOT/audio"
FORCE=0
for arg in "$@"; do
  case "$arg" in
    -f|--force) FORCE=1 ;;
    -h|--help)
      echo "Usage: $0 [--force]"
      exit 0
      ;;
    *)
      echo "error: unknown argument $arg" >&2
      exit 1
      ;;
  esac
done

resolve_ffmpeg() {
  if command -v ffmpeg >/dev/null 2>&1; then
    command -v ffmpeg
    return
  fi
  if command -v uvx >/dev/null 2>&1; then
    uvx --from imageio-ffmpeg python -c \
      'import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())'
    return
  fi
  echo "error: ffmpeg 5.1+ is required to encode DFPWM." >&2
  echo "  sudo apt install ffmpeg" >&2
  echo "  or drop files on https://music.madefor.cc/ (48 kHz mono DFPWM1a)" >&2
  exit 1
}

FFMPEG="$(resolve_ffmpeg)"
# grep -q closes the pipe early and FFmpeg dies with SIGPIPE (141),
# which pipefail treats as failure even when the encoder exists.
if ! "$FFMPEG" -hide_banner -encoders 2>&1 | grep 'dfpwm' >/dev/null; then
  echo "error: $FFMPEG has no DFPWM encoder (need FFmpeg 5.1+)." >&2
  exit 1
fi

if [[ ! -d "$AUDIO" ]]; then
  echo "error: missing directory $AUDIO" >&2
  exit 1
fi

shopt -s nullglob
sources=("$AUDIO"/*.mp3 "$AUDIO"/*.wav "$AUDIO"/*.m4a "$AUDIO"/*.ogg "$AUDIO"/*.flac)
if [[ ${#sources[@]} -eq 0 ]]; then
  echo "No source audio in $AUDIO"
  exit 0
fi

echo "Encoding ${#sources[@]} file(s) with $FFMPEG"
for src in "${sources[@]}"; do
  stem="$(basename "$src")"
  stem="${stem%.*}"
  dst="$AUDIO/${stem}.dfpwm"
  if [[ "$FORCE" -eq 0 && -f "$dst" && "$dst" -nt "$src" ]]; then
    echo "  skip $stem.dfpwm (up to date)"
    continue
  fi
  echo "  $stem -> ${stem}.dfpwm"
  # Mono 48 kHz is required. loudnorm TP=-6 leaves headroom so the 1-bit
  # encoder does not rail; I=-16 is a typical speech/podcast loudness.
  "$FFMPEG" -y -hide_banner -loglevel error \
    -i "$src" \
    -af "highpass=f=80,loudnorm=I=-16:TP=-6:LRA=11" \
    -ac 1 -ar 48000 \
    -c:a dfpwm -f dfpwm \
    "$dst"
done
echo "Done."
