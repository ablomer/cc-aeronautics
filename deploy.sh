#!/usr/bin/env bash
# Upload ComputerCraft Lua programs and DFPWM clips to the ship's floppy
# disk over SFTP.
set -euo pipefail

HOST="${DEPLOY_HOST:-augusto@local.ablomer.io}"
REMOTE_DIR="${DEPLOY_DIR:-/srv/containers/neoforge/data/world/computercraft/disk/0}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$ROOT"

if [[ -x "$ROOT/convert-audio.sh" ]]; then
  "$ROOT/convert-audio.sh" || echo "warning: audio convert failed; deploying existing .dfpwm if any" >&2
fi

mapfile -t files < <(find . -maxdepth 1 -type f -name '*.lua' | sed 's|^\./||' | sort)
if [[ ${#files[@]} -eq 0 ]]; then
  echo "error: no .lua files in $ROOT" >&2
  exit 1
fi

audio_files=()
if [[ -d audio ]]; then
  mapfile -t audio_files < <(find audio -maxdepth 1 -type f -name '*.dfpwm' | sort)
fi

echo "Deploying ${#files[@]} lua file(s) and ${#audio_files[@]} clip(s) to ${HOST}:${REMOTE_DIR}"
for f in "${files[@]}"; do
  echo "  $f"
done
for f in "${audio_files[@]}"; do
  echo "  $f"
done

batch="$(mktemp)"
trap 'rm -f "$batch"' EXIT

{
  printf 'cd %s\n' "$REMOTE_DIR"
  for f in "${files[@]}"; do
    printf 'put %s\n' "$f"
  done
  if [[ ${#audio_files[@]} -gt 0 ]]; then
    printf -- '-mkdir audio\n'
    for f in "${audio_files[@]}"; do
      printf 'put %s %s\n' "$f" "$f"
    done
  fi
} >"$batch"

sftp -b "$batch" "$HOST"

echo "Done."
