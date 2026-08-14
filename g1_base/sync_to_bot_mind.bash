#!/usr/bin/env bash
# ============================================================
# sync_to_bot_mind.bash
# Sync g1_base project to bot_mind/g1_base using rsync
# Respects .syncignore for exclusion rules
# ============================================================

set -euo pipefail

# --- Configuration ---
SRC_DIR="/home/lemon/vscode_projects/g1_base/"
DEST_DIR="/home/lemon/vscode_projects/bot_mind/g1_base"
SYNCIGNORE="${SRC_DIR}.syncignore"

# --- Colors ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# --- Functions ---
log_info()  { echo -e "${CYAN}[INFO]${NC}  $*"; }
log_ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }

# --- Pre-flight checks ---
if [[ ! -d "$SRC_DIR" ]]; then
    log_error "Source directory does not exist: $SRC_DIR"
    exit 1
fi

if [[ ! -f "$SYNCIGNORE" ]]; then
    log_error ".syncignore not found at: $SYNCIGNORE"
    exit 1
fi

# Create destination if it doesn't exist
if [[ ! -d "$DEST_DIR" ]]; then
    log_warn "Destination directory does not exist, creating: $DEST_DIR"
    mkdir -p "$DEST_DIR"
fi

# --- Dry-run support ---
DRY_RUN=""
if [[ "${1:-}" == "--dry-run" || "${1:-}" == "-n" ]]; then
    DRY_RUN="--dry-run"
    log_warn "DRY-RUN mode: no files will be modified"
fi

# --- Sync ---
log_info "Syncing: $SRC_DIR -> $DEST_DIR"
log_info "Using exclude file: $SYNCIGNORE"
echo ""

rsync -av --delete --delete-excluded \
    --exclude-from="$SYNCIGNORE" \
    $DRY_RUN \
    "$SRC_DIR" "$DEST_DIR"

echo ""
if [[ -n "$DRY_RUN" ]]; then
    log_warn "Dry-run complete. Run without --dry-run to apply changes."
else
    log_ok "Sync complete!"
fi
