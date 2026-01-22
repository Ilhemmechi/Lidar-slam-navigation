#!/usr/bin/env bash
# Run evo APE/RPE evaluation between ground-truth and estimated trajectories.
# Usage: ./run_evo.sh <groundtruth_file> <estimated_file> [format]
# format: tum (default) | kitti

set -euo pipefail

GT_FILE="$1"
EST_FILE="$2"
FORMAT="${3:-tum}"
OUT_DIR="results/evaluation_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT_DIR"

# Ensure evo is installed in the active environment
if ! command -v evo_ape >/dev/null 2>&1; then
  echo "evo not found. Installing evo into current Python environment..."
  pip install --upgrade pip
  pip install evo
fi

echo "Evaluating trajectories (format=$FORMAT)"
if [ "$FORMAT" = "tum" ]; then
  evo_ape tum "$GT_FILE" "$EST_FILE" -r full --save_results "$OUT_DIR/ape_results.txt" --plot --save_plot "$OUT_DIR/ape_plot.pdf"
  evo_rpe tum "$GT_FILE" "$EST_FILE" --delta 0.1 -r full --save_results "$OUT_DIR/rpe_results.txt" --plot --save_plot "$OUT_DIR/rpe_plot.pdf"
elif [ "$FORMAT" = "kitti" ]; then
  evo_ape kitti "$GT_FILE" "$EST_FILE" -r full --save_results "$OUT_DIR/ape_results.txt" --plot --save_plot "$OUT_DIR/ape_plot.pdf"
  evo_rpe kitti "$GT_FILE" "$EST_FILE" --delta 0.1 -r full --save_results "$OUT_DIR/rpe_results.txt" --plot --save_plot "$OUT_DIR/rpe_plot.pdf"
else
  echo "Unknown format: $FORMAT"
  exit 2
fi

echo "Results saved in $OUT_DIR"
