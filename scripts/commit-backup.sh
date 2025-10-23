#!/bin/bash

# commit-backup.sh - Commit a Particle Build backup to the aquarium-controller repo
#
# Usage:
#   ./commit-backup.sh <zip-file> -m "Your commit message"
#   ./commit-backup.sh <zip-file> --auto-message
#   ./commit-backup.sh <zip-file> --suggest

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

function print_usage() {
    echo "Usage: $0 <zip-file> [options]"
    echo ""
    echo "Options:"
    echo "  -m, --message MSG    Commit with the specified message"
    echo "  --auto-message       Let Claude Code analyze changes and suggest a message"
    echo "  --suggest            Show diff and suggested message, but don't commit"
    echo "  -h, --help          Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 backup.zip -m \"Fix cooling pump bug\""
    echo "  $0 backup.zip --auto-message"
    echo "  $0 backup.zip --suggest"
}

function error_exit() {
    echo -e "${RED}Error: $1${NC}" >&2
    exit 1
}

function info() {
    echo -e "${BLUE}$1${NC}"
}

function success() {
    echo -e "${GREEN}$1${NC}"
}

function warning() {
    echo -e "${YELLOW}$1${NC}"
}

# Parse arguments
ZIP_FILE=""
COMMIT_MESSAGE=""
MODE="manual"

if [ $# -eq 0 ]; then
    print_usage
    exit 1
fi

# Check for help first
if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    print_usage
    exit 0
fi

ZIP_FILE="$1"
shift

while [ $# -gt 0 ]; do
    case "$1" in
        -m|--message)
            if [ -z "$2" ]; then
                error_exit "Commit message required after -m/--message"
            fi
            COMMIT_MESSAGE="$2"
            MODE="manual"
            shift 2
            ;;
        --auto-message)
            MODE="auto"
            shift
            ;;
        --suggest)
            MODE="suggest"
            shift
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        *)
            error_exit "Unknown option: $1"
            ;;
    esac
done

# Validate zip file exists
if [ ! -f "$ZIP_FILE" ]; then
    error_exit "Zip file not found: $ZIP_FILE"
fi

# Make sure we're in a git repo
cd "$REPO_DIR"
if [ ! -d .git ]; then
    error_exit "Not in a git repository. Run this script from the git-repo-code-snapshots directory."
fi

# Check for uncommitted changes
if ! git diff-index --quiet HEAD -- 2>/dev/null; then
    error_exit "You have uncommitted changes. Please commit or stash them first."
fi

info "Extracting backup to repository..."

# Remove all files except .git, scripts/, config files, and README
find . -maxdepth 1 -not -name '.git' -not -name '.' -not -name '..' -not -name 'scripts' -not -name 'README.md' -not -name 'particle.json' -not -name '.gitignore' -exec rm -rf {} + 2>/dev/null || true

# Extract the zip file
unzip -q "$ZIP_FILE" || error_exit "Failed to extract zip file"

success "Backup extracted successfully"

# Stage all changes
git add -A

# Check if there are any changes
if git diff --cached --quiet; then
    warning "No changes detected in this backup. Nothing to commit."
    exit 0
fi

info "Changes detected:"
git status --short

# Get statistics
STATS=$(git diff --cached --stat)
echo ""
echo "$STATS"
echo ""

if [ "$MODE" = "manual" ]; then
    # Manual mode: commit with provided message
    if [ -z "$COMMIT_MESSAGE" ]; then
        error_exit "No commit message provided. Use -m, --auto-message, or --suggest"
    fi

    info "Committing with message: $COMMIT_MESSAGE"
    git commit -m "$COMMIT_MESSAGE"
    success "Committed successfully!"

elif [ "$MODE" = "suggest" ]; then
    # Suggest mode: show changes and prompt for Claude Code analysis
    info "Changes ready for review. To analyze with Claude Code:"
    echo ""
    echo -e "${YELLOW}Run this command:${NC}"
    echo "  git diff --cached --stat && git diff --cached | head -500"
    echo ""
    echo -e "${YELLOW}Or paste this prompt to Claude Code:${NC}"
    echo "  Review the staged changes in git and suggest a detailed commit message"
    echo "  following the style of previous commits in this repository. Include:"
    echo "  - Brief summary line"
    echo "  - Key changes and improvements"
    echo "  - New features or bug fixes"
    echo "  - Backup date if identifiable from filename"
    echo ""
    echo "Backup file: $(basename "$ZIP_FILE")"

elif [ "$MODE" = "auto" ]; then
    # Auto mode: generate analysis prompt and save to temp file
    info "Generating commit message analysis prompt..."

    TEMP_ANALYSIS="/tmp/aquarium-commit-analysis.txt"

    cat > "$TEMP_ANALYSIS" <<EOF
Please analyze the staged changes in the git repository at:
$REPO_DIR

Backup file: $(basename "$ZIP_FILE")

Generate a detailed commit message following the style of previous commits in this repository.

The commit message should include:
1. A concise summary line (50-72 chars)
2. Detailed description including:
   - Key changes and improvements
   - New features added
   - Bug fixes
   - Configuration changes
   - Backup date: $(basename "$ZIP_FILE" .zip)

Changes summary:
$STATS

Please review the actual code changes and provide a commit message in the same format
as previous commits in this repository. After reviewing, I'll need you to provide just
the commit message text that I can use with 'git commit -m'.

Use: git diff --cached
To see the actual changes.
EOF

    success "Analysis prompt saved to: $TEMP_ANALYSIS"
    echo ""
    info "Two options to proceed:"
    echo ""
    echo -e "${YELLOW}Option 1 - Interactive (recommended):${NC}"
    echo "  1. Open Claude Code in this directory"
    echo "  2. Paste: 'Review the staged git changes and suggest a detailed commit message"
    echo "     following the repository's commit style. Backup: $(basename "$ZIP_FILE")'"
    echo "  3. Copy the suggested message and run:"
    echo "     git commit -m \"<paste message here>\""
    echo ""
    echo -e "${YELLOW}Option 2 - Direct:${NC}"
    echo "  cat $TEMP_ANALYSIS | pbcopy"
    echo "  # Then paste into Claude Code"
    echo ""
    echo "Changes are staged and ready to commit once you have the message."
fi
