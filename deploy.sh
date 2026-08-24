#!/usr/bin/env bash
# Upload ComputerCraft Lua programs to the ship's floppy disk over SFTP.
set -euo pipefail

HOST="${DEPLOY_HOST:-augusto@local.ablomer.io}"
REMOTE_DIR="${DEPLOY_DIR:-/srv/containers/neoforge/data/world/computercraft/disk/0}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$ROOT"

mapfile -t files < <(find . -maxdepth 1 -type f -name '*.lua' | sed 's|^\./||' | sort)
if [[ ${#files[@]} -eq 0 ]]; then
  echo "error: no .lua files in $ROOT" >&2
  exit 1
fi

echo "Deploying ${#files[@]} file(s) to ${HOST}:${REMOTE_DIR}"
for f in "${files[@]}"; do
  echo "  $f"
done

batch="$(mktemp)"
trap 'rm -f "$batch"' EXIT

{
  printf 'cd %s\n' "$REMOTE_DIR"
  for f in "${files[@]}"; do
    printf 'put %s\n' "$f"
  done
} >"$batch"

sftp -b "$batch" "$HOST"

echo "Done."
