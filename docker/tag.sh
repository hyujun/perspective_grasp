#!/bin/bash
# =============================================================================
# tag.sh - Apply `:<date>-<git-sha>` tags to the perspective_grasp ML images.
#
# After `docker compose -f docker/docker-compose.yml build` produces the
# `:latest` tag (or `${IMAGE_TAG}` if overridden), this script adds a
# date+SHA tag so the exact build is recoverable / rollback-able later.
#
# Usage:
#   ./docker/tag.sh                 # tag both :latest -> :YYYYMMDD-<sha7> (no push)
#   ./docker/tag.sh --push          # same, then `docker push` to ${REGISTRY}
#   ./docker/tag.sh --tag 2026-04-21-release  # explicit tag string
#
# Env vars (same as docker-compose.yml):
#   REGISTRY    default `perspective_grasp` (local-only; never pushes)
#   IMAGE_TAG   default `latest` (source tag to copy from)
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

REGISTRY="${REGISTRY:-perspective_grasp}"
SRC_TAG="${IMAGE_TAG:-latest}"
DO_PUSH=0
EXPLICIT_TAG=""

while (( $# > 0 )); do
    case "$1" in
        --push)   DO_PUSH=1; shift ;;
        --tag)    EXPLICIT_TAG="$2"; shift 2 ;;
        -h|--help)
            sed -n '2,17p' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        *)
            echo "Unknown arg: $1 (use --help)" >&2
            exit 1 ;;
    esac
done

# Derive the target tag.
if [ -n "$EXPLICIT_TAG" ]; then
    DST_TAG="$EXPLICIT_TAG"
else
    SHA="$(git -C "$REPO_ROOT" rev-parse --short=7 HEAD 2>/dev/null || echo 'nogit')"
    DIRTY="$(git -C "$REPO_ROOT" status --porcelain 2>/dev/null | head -1)"
    if [ -n "$DIRTY" ]; then
        SHA="${SHA}-dirty"
        echo ">>> warning: working tree has uncommitted changes; tagging as ${SHA}" >&2
    fi
    DATE="$(date +%Y%m%d)"
    DST_TAG="${DATE}-${SHA}"
fi

if [ "$DO_PUSH" = "1" ] && [ "$REGISTRY" = "perspective_grasp" ]; then
    echo ">>> refusing --push with default local REGISTRY='perspective_grasp'." >&2
    echo ">>> set REGISTRY=<host/org> before pushing (e.g. ghcr.io/myorg/pg)." >&2
    exit 1
fi

SERVICES=(ml-base foundationpose cosypose sam2 bundlesdf)

echo "============================================================"
echo " perspective_grasp - Image tag apply"
echo "   REGISTRY : $REGISTRY"
echo "   SRC tag  : $SRC_TAG"
echo "   DST tag  : $DST_TAG"
echo "   Push     : $([ "$DO_PUSH" = "1" ] && echo 'yes' || echo 'no')"
echo "============================================================"

for svc in "${SERVICES[@]}"; do
    src="${REGISTRY}/${svc}:${SRC_TAG}"
    dst="${REGISTRY}/${svc}:${DST_TAG}"

    if ! docker image inspect "$src" >/dev/null 2>&1; then
        echo ">>> [$svc] source image not found: $src (skipped)"
        continue
    fi

    echo ">>> [$svc] $src  ->  $dst"
    docker tag "$src" "$dst"

    if [ "$DO_PUSH" = "1" ]; then
        docker push "$dst"
    fi
done

echo ""
echo "Done. Record this tag somewhere recoverable:"
echo "   IMAGE_TAG=$DST_TAG"
