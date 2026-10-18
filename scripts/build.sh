#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

mkdir -p build/classes/main build/classes/test build/libs

echo "[1/3] Compiling FluxProcessor (Java 17 bytecode)..."
javac --release 17 -d build/classes/main $(find src/main/java -name "*.java")
javac --release 17 -cp build/classes/main -d build/classes/test $(find src/test/java -name "*.java")

echo "[2/3] Running FluxProcessor unit & integration tests..."
java -ea -cp build/classes/main:build/classes/test frc.flux.processor.FluxProcessorTest

echo "[3/3] Packaging executable build/libs/flux-processor.jar..."
jar --create --file build/libs/flux-processor.jar --main-class frc.flux.processor.FluxProcessorMain -C build/classes/main .

echo "Done! Executable JAR: $ROOT_DIR/build/libs/flux-processor.jar"
