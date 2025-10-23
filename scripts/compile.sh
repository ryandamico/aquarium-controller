#!/bin/bash

# compile.sh - Compile the aquarium controller firmware
#
# Usage:
#   ./compile.sh [options]
#
# Options:
#   --target VERSION    Override Device OS version (default: from particle.json)
#   --platform DEVICE   Override platform (default: from particle.json)
#   --output FILE       Output filename (default: aquarium-controller.bin)
#   -h, --help          Show this help

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

function print_usage() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --target VERSION    Override Device OS version"
    echo "  --platform DEVICE   Override platform (argon, photon, etc)"
    echo "  --output FILE       Output filename"
    echo "  -h, --help          Show this help"
}

# Read config from particle.json
CONFIG_FILE="$REPO_DIR/particle.json"
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: particle.json not found"
    exit 1
fi

# Parse JSON (basic parsing - requires jq for complex queries)
if command -v jq &> /dev/null; then
    DEFAULT_PLATFORM=$(jq -r '.build.platform' "$CONFIG_FILE")
    DEFAULT_TARGET=$(jq -r '.build.targetVersion' "$CONFIG_FILE")
else
    # Fallback to grep if jq not available
    DEFAULT_PLATFORM=$(grep -o '"platform"[[:space:]]*:[[:space:]]*"[^"]*"' "$CONFIG_FILE" | cut -d'"' -f4)
    DEFAULT_TARGET=$(grep -o '"targetVersion"[[:space:]]*:[[:space:]]*"[^"]*"' "$CONFIG_FILE" | cut -d'"' -f4)
fi

PLATFORM="${DEFAULT_PLATFORM:-argon}"
TARGET="${DEFAULT_TARGET:-4.2.0}"
OUTPUT="aquarium-controller-${PLATFORM}.bin"

# Parse arguments
while [ $# -gt 0 ]; do
    case "$1" in
        --target)
            TARGET="$2"
            shift 2
            ;;
        --platform)
            PLATFORM="$2"
            OUTPUT="aquarium-controller-${PLATFORM}.bin"
            shift 2
            ;;
        --output)
            OUTPUT="$2"
            shift 2
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            print_usage
            exit 1
            ;;
    esac
done

cd "$REPO_DIR"

echo -e "${BLUE}Compiling aquarium controller${NC}"
echo "Platform: $PLATFORM"
echo "Target OS: $TARGET"
echo "Output: $OUTPUT"
echo ""

~/bin/particle compile "$PLATFORM" . --target "$TARGET" --saveTo "$OUTPUT"

if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}Compilation successful!${NC}"
    echo "Binary: $OUTPUT"
else
    echo "Compilation failed"
    exit 1
fi
