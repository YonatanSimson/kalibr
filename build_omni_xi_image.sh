#!/bin/bash
# Build the patched kalibr image from this checkout (see OMNI_FIXED_XI.md).
#   ./build_omni_xi_image.sh [tag]        default tag: kalibr_ubuntu2004_omnixi
# BASE (default 1f60227, upstream master this branch starts from) selects what counts as changed.
set -eo pipefail
cd "$(dirname "$0")"
TAG="${1:-kalibr_ubuntu2004_omnixi}"
BASE="${BASE:-1f60227}"
CTX="$(mktemp -d)"
trap 'rm -rf "$CTX"' EXIT
# Source files only: anything under a package directory, changed or new since BASE.
{ git diff --name-only "$BASE" --; git ls-files --others --exclude-standard; } \
    | grep '/' | sort -u | while read -r f; do [[ -f "$f" ]] && echo "$f"; done > "$CTX/files.txt"
echo "layering $(wc -l < "$CTX/files.txt" | tr -d ' ') changed files on kalibr_ubuntu2004:"
sed 's/^/  /' "$CTX/files.txt"
tar cf "$CTX/changed.tar" -T "$CTX/files.txt"
cp Dockerfile_omni_xi "$CTX/Dockerfile"
docker build -t "$TAG" "$CTX"
