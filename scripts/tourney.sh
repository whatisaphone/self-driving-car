#!/usr/bin/env bash

set -euo pipefail

here="$(dirname "$0")"
root="$here"/..
dest="$root"/target/tournament

rm -rf "$dest"
mkdir "$dest"

cd "$root"

# Build the app
cargo build --release
cp target/release/play.exe "$dest"

# Copy files needed by RLBot
cp python/appearance.cfg "$dest"
perl -pe 's=target/\w+/==' python/bot.cfg > "$dest"/bot.cfg
cp python/bot.py "$dest"
cp python/psyonix.cfg "$dest"

# Modify the local RLBot config for quick testing.
perl -pe 's=\bpython/==' rlbot.cfg > "$dest"/rlbot.cfg
