#!/usr/bin/env bash
# Horizon-resolved brackets on the EXISTING certified production DRN.
# No regeneration; two batched invocations. SCRATCH-ONLY.
set -u
cd /disk/scratch/s2563304/interval-mdp-aebs

STORM="apptainer exec /disk/scratch/s2563304/storm.sif storm"
DRN="artifacts/modular_system.drn"
STAMP=$(date +%Y%m%d_%H%M%S)
OUT="logs/horizon_${STAMP}"
mkdir -p "$OUT"

HORIZONS="5 10 15 20 25 30 40 50 75"
FILTERS="scn_safe_cruising scn_warning_brake scn_emergency_brake nominal"

{
  echo "== provenance =="
  date -Is
  git rev-parse --short HEAD
  ls -l "$DRN"
} > "$OUT/provenance.txt"

PMAX="$OUT/pmax_prod.props"
PMIN="$OUT/pmin_prod.props"
: > "$PMAX"; : > "$PMIN"
for K in $HORIZONS; do
  for F in $FILTERS; do
    echo "filter(max, Pmax=? [ F<=$K \"crash\" ], \"$F\");" >> "$PMAX"
    echo "filter(min, Pmin=? [ F<=$K \"crash\" ], \"$F\");" >> "$PMIN"
  done
done
sed -i '$ s/;$//' "$PMAX" "$PMIN"

for DIR in maximize minimize; do
  if [ "$DIR" = maximize ]; then PROPS="$PMAX"; else PROPS="$PMIN"; fi
  LOG="$OUT/${DIR}_prod.log"
  echo "== storm $DIR: production DRN, horizon ladder =="
  $STORM --explicit-drn "$DRN" --prop "$PROPS" \
      --uncertainty-resolution "$DIR" --timemem \
      > "$LOG" 2>&1
  N=$(grep -cE '^Result' "$LOG" || true)
  echo "  results parsed: $N"
done
echo "== horizon campaign complete: $OUT =="
