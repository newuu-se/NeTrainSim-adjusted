#!/usr/bin/env bash
# Builds the NeTrainSim console binary on macOS (Homebrew Qt6) and runs the
# bundled sample project, writing the results to res/.
set -euo pipefail

cd "$(dirname "$0")"

BUILD_DIR="build-mac"
OUTPUT_DIR="res"
SAMPLE_DIR="src/data/sampleProject"
TARGET="NeTrainSimConsole"

if ! command -v brew >/dev/null 2>&1; then
    echo "error: Homebrew is required to locate Qt6. See https://brew.sh" >&2
    exit 1
fi

QT_PREFIX="$(brew --prefix qt6)" || {
    echo "error: Qt6 not found. Install it with: brew install qt6" >&2
    exit 1
}

cmake -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_GUI=OFF \
    -DBUILD_SERVER=OFF \
    -DCMAKE_PREFIX_PATH="$QT_PREFIX"
cmake --build "$BUILD_DIR" --target "$TARGET" -j"$(sysctl -n hw.logicalcpu)"

mkdir -p "$OUTPUT_DIR"
"./$BUILD_DIR/src/$TARGET/NeTrainSim" \
    -n "$SAMPLE_DIR/nodesFile.dat" \
    -l "$SAMPLE_DIR/linksFile.dat" \
    -t "$SAMPLE_DIR/dieselTrain.dat" \
    -o "$OUTPUT_DIR"
