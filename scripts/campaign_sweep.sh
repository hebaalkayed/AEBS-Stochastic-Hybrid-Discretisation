#!/usr/bin/env bash
# Resolution-sweep Storm campaign. SCRATCH-ONLY. Per model: composition
# certificate gate, then one batched invocation per interval resolution.
set -u
cd /disk/scratch/s2563304/interval-mdp-aebs

STORM="apptainer exec /disk/scratch/s2563304/storm.sif storm"
STAMP=$(date +%Y%m%d_%H%M%S)
OUT="logs/sweep_${STAMP}"
mkdir -p "$OUT"

HORIZONS="5 10 15 20 30 40 50 100"
FILTERS="scn_safe_cruising scn_warning_brake scn_emergency_brake nominal"

{
  echo "== provenance =="
  date -Is
  git rev-parse --short HEAD
  ls -l artifacts/sweep_*.drn 2>/dev/null
} > "$OUT/provenance.txt"

for PRESET in sweep_r100 sweep_r050 sweep_r025 sweep_r050v; do
  DRN="artifacts/sweep_${PRESET}.drn"
  PRISM="artifacts/sweep_${PRESET}.prism"
  if [ ! -f "$DRN" ]; then echo "MISSING $DRN, skipping"; continue; fi

  echo "== equivalence certificate: $PRESET =="
  python3 tests/drn_equivalence_test.py --prism "$PRISM" --drn "$DRN" \
      > "$OUT/equiv_${PRESET}.log" 2>&1
  if ! grep -q "COMPOSITION CERTIFICATE PASS" "$OUT/equiv_${PRESET}.log"; then
    echo "EQUIVALENCE FAILED for $PRESET, skipping verification"
    continue
  fi

  PMAX="$OUT/pmax_${PRESET}.props"
  PMIN="$OUT/pmin_${PRESET}.props"
  : > "$PMAX"; : > "$PMIN"
  for K in $HORIZONS; do
    for F in $FILTERS; do
      echo "filter(max, Pmax=? [ F<=$K \"crash\" ], \"$F\");" >> "$PMAX"
      echo "filter(min, Pmin=? [ F<=$K \"crash\" ], \"$F\");" >> "$PMIN"
    done
  done
  echo "filter(max, Pmax=? [ F<=100 \"crash\" ], \"scn_post_collision\");" >> "$PMAX"
  sed -i '$ s/;$//' "$PMAX" "$PMIN"

  for DIR in maximize minimize; do
    if [ "$DIR" = maximize ]; then PROPS="$PMAX"; else PROPS="$PMIN"; fi
    LOG="$OUT/${DIR}_${PRESET}.log"
    echo "== storm $DIR: $PRESET =="
    $STORM --explicit-drn "$DRN" --prop "$PROPS" \
        --uncertainty-resolution "$DIR" --timemem \
        > "$LOG" 2>&1
    N=$(grep -cE '^Result' "$LOG" || true)
    echo "  results parsed: $N"
  done
done

echo "== sweep campaign complete: $OUT =="
