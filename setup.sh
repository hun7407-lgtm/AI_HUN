#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="${1:-$ROOT_DIR/workspace}"

CYCLO_LAB_COMMIT="a5ea01967b145f839ca1ac8f51b42abf9ef87036"
AI_WORKER_COMMIT="e8c2eacb612e47473cdf03e44bee6d527c00b4f9"
ROBOTIS_APPLICATIONS_COMMIT="7ef0aabc748174cb91013866b2e4142122ef475c"

clone_or_update() {
  local repo_url="$1"
  local dir="$2"
  local commit="$3"

  if [[ ! -d "$dir/.git" ]]; then
    git clone "$repo_url" "$dir"
  fi

  git -C "$dir" fetch --all --tags
  git -C "$dir" checkout "$commit"
}

echo "Installing AIWORKER into: $INSTALL_DIR"
mkdir -p "$INSTALL_DIR"

clone_or_update "https://github.com/ROBOTIS-GIT/cyclo_lab.git" "$INSTALL_DIR/cyclo_lab" "$CYCLO_LAB_COMMIT"
clone_or_update "https://github.com/ROBOTIS-GIT/ai_worker.git" "$INSTALL_DIR/ai_worker" "$AI_WORKER_COMMIT"
clone_or_update "https://github.com/ROBOTIS-GIT/robotis_applications.git" "$INSTALL_DIR/robotis_applications" "$ROBOTIS_APPLICATIONS_COMMIT"

echo "Updating cyclo_lab submodules..."
git -C "$INSTALL_DIR/cyclo_lab" submodule update --init --recursive

echo "Applying AIWORKER overlay..."
rsync -a "$ROOT_DIR/overlays/cyclo_lab/" "$INSTALL_DIR/cyclo_lab/"
rsync -a "$ROOT_DIR/overlays/robotis_applications/" "$INSTALL_DIR/robotis_applications/"

echo
echo "Done."
echo
echo "Next steps:"
echo "  1. cd \"$INSTALL_DIR/cyclo_lab/docker\" && ./container.sh start"
echo "  2. cd \"$INSTALL_DIR/robotis_applications/docker\" && ./container.sh start"
echo "  3. cd \"$INSTALL_DIR/ai_worker/docker\" && ./container.sh start"
echo "  4. cd \"$INSTALL_DIR/cyclo_lab\" && python3 sg2_ltable_dashboard.py"
