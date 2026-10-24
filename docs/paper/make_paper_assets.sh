#!/usr/bin/env bash
# Regenerate every number and figure that depends on the flights.
#
#   1. add a row to docs/experiments/flights_manifest.csv for the new flight
#   2. bash docs/paper/make_paper_assets.sh
#
# Scores new flights only (cached by bag path); `--refresh` re-reads every bag.
# CPU-heavy parts run at low priority and are the ones to avoid during a flight:
#   flights_summary.py, gate_plateau.py read the bags; recon_batch.py builds a 6 M-point
#   CAD tree. Run `--light` to redraw figures from the cached numbers only.
set -euo pipefail
cd "$(dirname "$0")/../.."
LIGHT=0; REFRESH=""
for a in "$@"; do case "$a" in --light) LIGHT=1;; --refresh) REFRESH="--refresh";; esac; done

if [ "$LIGHT" = 0 ]; then
  nice -n 15 python3 eval/flights_summary.py $REFRESH
  nice -n 15 python3 eval/gate_plateau.py $REFRESH
  # trust-SLAM counterfactual and sonar reconstruction for the new flights
  OMP_NUM_THREADS=2 nice -n 19 python3 eval/recon_batch.py
fi
for f in fig_control fig_nav fig_endstate fig_gate_plateau fig_recon_sim2 fig_symmetry fig_system fig_contam fig_where fig_cad fig_scene fig_orbslam fig_story; do
  python3 "eval/$f.py" 2>&1 | grep -v -e Axes3D -e warnings.warn || true
done
echo "done. numbers: docs/paper/live_numbers.md ; figures: eval/figures/"
