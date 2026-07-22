#!/usr/bin/env bash
# Numeric cross-validation of the Form A phase collapse on a SMALL preset:
# Pmax/Pmin [ F<=3k "crash" ] on the modular PRISM composition must equal
# [ F<=k ] on the closed-loop DRN, exactly (bounded until = k exact sweeps).
# Usage: bash crosscheck_collapse.sh <model.prism> <model.drn> <scenario_id> <scn_label>
set -euo pipefail
PRISM=$1; DRN=$2; SCEN_ID=$3; SCEN_LABEL=$4
SIF=${SIF:-/disk/scratch/s2563304/storm.sif}
HORIZONS=${HORIZONS:-"1 2 5 10 33 100"}
extract_result () { grep -oP 'Result \(for .*\): \K[0-9.eE+-]+' "$1" | tail -1; }
fail=0
for k in $HORIZONS; do
  for mode in max min; do
    RES=$([ $mode = max ] && echo maximize || echo minimize)
    P=$([ $mode = max ] && echo Pmax || echo Pmin)
    apptainer exec "$SIF" storm --prism "$PRISM" \
      --prop "${P}=? [ F<=$((3 * k)) \"crash\" ]" \
      --constants start_s=$SCEN_ID \
      --uncertainty-resolution $RES > /tmp/xc_prism.txt 2>&1
    v_prism=$(extract_result /tmp/xc_prism.txt)
    apptainer exec "$SIF" storm --explicit-drn "$DRN" \
      --prop "filter($mode, ${P}=? [ F<=$k \"crash\" ], \"$SCEN_LABEL\")" \
      --uncertainty-resolution $RES > /tmp/xc_drn.txt 2>&1
    v_drn=$(extract_result /tmp/xc_drn.txt)
    if python3 -c "import sys; sys.exit(0 if abs($v_prism - $v_drn) <= 1e-12 else 1)"; then
      echo "k=$k $P: PRISM(F<=$((3*k)))=$v_prism  DRN(F<=$k)=$v_drn  OK"
    else
      echo "k=$k $P: PRISM(F<=$((3*k)))=$v_prism  DRN(F<=$k)=$v_drn  MISMATCH"; fail=1
    fi
  done
done
[ $fail -eq 0 ] && echo "COLLAPSE CROSS-CHECK PASS" || { echo "COLLAPSE CROSS-CHECK FAILED"; exit 1; }
