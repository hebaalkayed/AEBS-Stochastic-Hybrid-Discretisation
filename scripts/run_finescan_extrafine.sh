#!/usr/bin/env bash
# Fine horizon rungs on the EXISTING certified extra_fine DRN.
# No regeneration; two batched invocations. SCRATCH-ONLY.
set -u
cd /disk/scratch/s2563304/interval-mdp-aebs
STORM="apptainer exec /disk/scratch/s2563304/storm.sif storm"
DRN="artifacts/extra_fine.drn"
STAMP=$(date +%Y%m%d_%H%M%S)
OUT="logs/extrafine_finescan_${STAMP}"
mkdir -p "$OUT"
HORIZONS="12 14 16 18"
WIDE_EXTRA="22 24 26 28"
FILTERS="scn_following_critical scn_following_tight scn_following_near scn_following_mid scn_following_wide"
{
  echo "== provenance =="
  date -Is
  git rev-parse --short HEAD
  ls -l "$DRN"
} > "$OUT/provenance.txt"
PMAX="$OUT/pmax_finescan.props"
PMIN="$OUT/pmin_finescan.props"
: > "$PMAX"; : > "$PMIN"
for K in $HORIZONS; do
  for F in $FILTERS; do
    echo "filter(max, Pmax=? [ F<=$K \"crash\" ], \"$F\");" >> "$PMAX"
    echo "filter(min, Pmin=? [ F<=$K \"crash\" ], \"$F\");" >> "$PMIN"
  done
done
for K in $WIDE_EXTRA; do
  echo "filter(max, Pmax=? [ F<=$K \"crash\" ], \"scn_following_wide\");" >> "$PMAX"
  echo "filter(min, Pmin=? [ F<=$K \"crash\" ], \"scn_following_wide\");" >> "$PMIN"
done
sed -i '$ s/;$//' "$PMAX" "$PMIN"
for DIR in maximize minimize; do
  if [ "$DIR" = maximize ]; then PROPS="$PMAX"; else PROPS="$PMIN"; fi
  LOG="$OUT/${DIR}_extrafine.log"
  echo "== storm $DIR: extra_fine DRN, fine horizon rungs =="
  $STORM --explicit-drn "$DRN" --prop "$PROPS" \
      --uncertainty-resolution "$DIR" --timemem \
      > "$LOG" 2>&1
  N=$(grep -cE '^Result' "$LOG" || true)
  echo "  results parsed: $N"
done
echo "== fine scan complete: $OUT =="
