#!/usr/bin/env bash
# Overnight Storm campaign against the closed-loop DRN artifact.
# Two batched jobs: all Pmax under maximize, all Pmin under minimize.
# Scenario filters (scn_*) plus the region ladder: nominal (safe and
# outside the warning region), braking (the dual-trigger WARNING region;
# never phrase as 'controller brakes'), emergency (dual-trigger region).
# Horizon: one DRN transition = one control period; F<=100 = 10 s.
set -euo pipefail
SCRATCH=/disk/scratch/s2563304
REPO=$SCRATCH/interval-mdp-aebs
DRN=$REPO/artifacts/modular_system.drn
SIF=$SCRATCH/storm.sif
STAMP=$(date +%Y%m%d_%H%M%S)
LOGDIR=$REPO/logs/campaign_drn_$STAMP
MEMCAP_KB=$((300 * 1024 * 1024))
TIMEOUT=${TIMEOUT:-20h}
mkdir -p "$LOGDIR"
[ -f "$DRN" ] || { echo "FATAL: $DRN missing (run the exporter first)"; exit 1; }
[ -f "$SIF" ] || { echo "FATAL: $SIF missing"; exit 1; }
AVAIL_G=$(df --output=avail -B1G "$SCRATCH" | tail -1 | tr -d ' ')
echo "[campaign] scratch free: ${AVAIL_G}G"
[ "$AVAIL_G" -ge 5 ] || { echo "FATAL: <5G free on scratch"; exit 1; }
{ echo "== provenance =="; date -Is; ls -l "$DRN"; head -12 "$DRN"; git -C "$REPO" rev-parse HEAD; } > "$LOGDIR/provenance.txt"

cat > "$LOGDIR/pmax.props" << 'PROPS'
filter(max, Pmax=? [ F<=100 "crash" ], "scn_safe_cruising");
filter(max, Pmax=? [ F<=100 "crash" ], "scn_warning_brake");
filter(max, Pmax=? [ F<=100 "crash" ], "scn_emergency_brake");
filter(max, Pmax=? [ F<=100 "crash" ], "scn_imminent_collision");
filter(max, Pmax=? [ F<=100 "crash" ], "scn_post_collision");
filter(max, Pmax=? [ F<=100 "crash" ], "nominal");
filter(max, Pmax=? [ F<=100 "crash" ], "braking");
filter(max, Pmax=? [ F<=100 "crash" ], "emergency")
PROPS

cat > "$LOGDIR/pmin.props" << 'PROPS'
filter(min, Pmin=? [ F<=100 "crash" ], "scn_safe_cruising");
filter(min, Pmin=? [ F<=100 "crash" ], "scn_warning_brake");
filter(min, Pmin=? [ F<=100 "crash" ], "scn_emergency_brake");
filter(min, Pmin=? [ F<=100 "crash" ], "scn_imminent_collision");
filter(min, Pmin=? [ F<=100 "crash" ], "nominal");
filter(min, Pmin=? [ F<=100 "crash" ], "braking");
filter(min, Pmin=? [ F<=100 "crash" ], "emergency")
PROPS

run_batch () {
  local name=$1 props=$2 resolution=$3
  echo "[campaign] $name start $(date -Is)"
  (
    ulimit -v "$MEMCAP_KB"
    timeout "$TIMEOUT" apptainer exec "$SIF" storm \
      --explicit-drn "$DRN" --prop "$props" \
      --uncertainty-resolution "$resolution" --timemem
  ) > "$LOGDIR/$name.txt" 2>&1 || echo "[campaign] $name FAILED exit=$?"
  echo "[campaign] $name done  $(date -Is)"
}
run_batch pmax_batch "$LOGDIR/pmax.props" maximize
run_batch pmin_batch "$LOGDIR/pmin.props" minimize
echo "== results summary =="
grep -H "Result" "$LOGDIR"/pm*_batch.txt || echo "no results found; inspect logs"
echo "[campaign] all done $(date -Is); logs in $LOGDIR"
