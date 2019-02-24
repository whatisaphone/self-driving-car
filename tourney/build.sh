#!/usr/bin/env bash

set -euo pipefail

here="$(dirname "$0" | xargs realpath)"
contents="$here"/contents
root="$here"/..
dest="$root"/target/tournament
dest_hero="$dest"/self-driving-car
dest_villain="$dest"/psyonix

# Set up the directory structure.
rm -rf "$dest"
mkdir "$dest"
mkdir "$dest_hero"
mkdir "$dest_villain"

# Build the app.
cd "$root"
cargo build --release
cp target/release/play.exe "$dest_hero"/self-driving-car.exe

cd "$contents"

# Copy the hero!
cat self-driving-car/self-driving-car.cfg \
    | perl -pe 's%^(path\s*=\s*).+$%\1self-driving-car.exe%' \
    > "$dest_hero"/self-driving-car.cfg
cp self-driving-car/appearance.cfg "$dest_hero"
cp self-driving-car/self_driving_car.py "$dest_hero"

# Copy the villain >:
cp psyonix/psyonix.cfg "$dest"/psyonix
cp psyonix/appearance.cfg "$dest"/psyonix

# Include an RLBot config for quick testing.
cp rlbot.cfg "$dest"

# Readme
cp README.txt "$dest"
