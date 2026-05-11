#!/bin/bash
# Bundle all Verilog source files into a single text file.
# Usage: ./scripts/bundle_src.sh [output_file]
#   Default output: build/src_bundle.txt

OUTPUT="${1:-build/src_bundle.txt}"
mkdir -p "$(dirname "$OUTPUT")"

> "$OUTPUT"
for f in src/*.v; do
    echo "=== $f ===" >> "$OUTPUT"
    cat "$f" >> "$OUTPUT"
    echo "" >> "$OUTPUT"
done

echo "Bundled $(ls src/*.v | wc -l | tr -d ' ') files into $OUTPUT"
