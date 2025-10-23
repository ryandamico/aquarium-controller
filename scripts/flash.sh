#!/bin/bash

# flash.sh - Flash firmware to the aquarium controller
#
# Usage:
#   ./flash.sh [options]
#
# Options:
#   --binary FILE       Flash specific binary file
#   --compile           Compile first, then flash
#   --device ID         Override device ID (default: from particle.json)
#   --target VERSION    Device OS version for compilation
#   -h, --help          Show this help

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

function print_usage() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --binary FILE       Flash specific binary file"
    echo "  --compile           Compile first, then flash"
    echo "  --device ID         Override device ID or name"
    echo "  --target VERSION    Device OS version for compilation"
    echo "  -h, --help          Show this help"
    echo ""
    echo "Examples:"
    echo "  $0                                  # Flash current directory OTA"
    echo "  $0 --compile                        # Compile then flash"
    echo "  $0 --binary file.bin                # Flash specific binary"
    echo "  $0 --device Aquarium --compile      # Flash to device by name"
}

# Read config from particle.json
CONFIG_FILE="$REPO_DIR/particle.json"
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: particle.json not found"
    exit 1
fi

# Parse JSON
if command -v jq &> /dev/null; then
    DEFAULT_DEVICE=$(jq -r '.build.deviceName' "$CONFIG_FILE")
    DEFAULT_TARGET=$(jq -r '.build.targetVersion' "$CONFIG_FILE")
else
    DEFAULT_DEVICE=$(grep -o '"deviceName"[[:space:]]*:[[:space:]]*"[^"]*"' "$CONFIG_FILE" | cut -d'"' -f4)
    DEFAULT_TARGET=$(grep -o '"targetVersion"[[:space:]]*:[[:space:]]*"[^"]*"' "$CONFIG_FILE" | cut -d'"' -f4)
fi

DEVICE="${DEFAULT_DEVICE:-Aquarium}"
TARGET="${DEFAULT_TARGET:-4.2.0}"
BINARY=""
DO_COMPILE=false

# Parse arguments
while [ $# -gt 0 ]; do
    case "$1" in
        --binary)
            BINARY="$2"
            shift 2
            ;;
        --compile)
            DO_COMPILE=true
            shift
            ;;
        --device)
            DEVICE="$2"
            shift 2
            ;;
        --target)
            TARGET="$2"
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

# Compile if requested
if [ "$DO_COMPILE" = true ]; then
    echo -e "${BLUE}Compiling before flash...${NC}"
    "$SCRIPT_DIR/compile.sh" --target "$TARGET"
    BINARY="aquarium-controller-argon.bin"
fi

echo -e "${BLUE}Flashing firmware to device${NC}"
echo "Device: $DEVICE"

if [ -n "$BINARY" ]; then
    if [ ! -f "$BINARY" ]; then
        echo "Error: Binary file not found: $BINARY"
        exit 1
    fi
    echo "Binary: $BINARY"
    echo ""
    ~/bin/particle flash "$DEVICE" "$BINARY"
else
    echo "Source: Current directory (OTA)"
    echo "Target OS: $TARGET"
    echo ""
    echo -e "${YELLOW}Warning: This will compile in the cloud and flash OTA${NC}"
    echo "Press Ctrl+C to cancel, or Enter to continue..."
    read
    ~/bin/particle flash "$DEVICE" . --target "$TARGET"
fi

if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}Flash successful!${NC}"
else
    echo "Flash failed"
    exit 1
fi
