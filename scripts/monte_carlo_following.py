"""Monte Carlo corroboration of the Following-family Pmax brackets.

Simulates the MODELLED system of the extra_fine configuration:
  - plant: ego_next (alpha=0.5 gain, floor 0), semi-implicit gap update
    (identical expressions to VehiclePlant.ego_next / gap_next);
  - lead:  v_lead' ~ Normal(max(0, v_lead), sigma_v^2), TRUNCATED to the
    grid band and renormalised (the modelling kernel the containment
    certificate covers; truncate_noise=True);
  - controller: industry AEBSController logic, evaluated at the CELL
    CENTRE of the current state (the quantised controller is the object
    verified; see the paper's remark in the pipeline section);
  - crash: entry into the crash-labelled cell slab (gap below the slab's
    upper edge, gap < r_g/2);
  - safe sink: leaving the grid window (gap at/above the top edge, or
    v_ego outside its band) ends the run as no-crash, mirroring escape
    routing to the absorbing safe sink.

Each empirical crash fraction must lie inside its certified
[Pmin, Pmax] bracket (all Pmin are 0, so the operative check is
fraction <= Pmax, up to sampling error). A fraction ABOVE Pmax + CI
is a soundness alarm: stop and investigate.

Self-check at startup: the vectorised controller logic is asserted
against src.system.controller.AEBSController on random states.

Run from the repo root on betel (scratch-only, do not commit):
    python3 scripts/monte_carlo_following.py [n_runs] [seed]
Defaults: n_runs=10000, seed=20260718.
"""
import os
import sys

import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from src.system.controller import AEBSController  # noqa: E402

# ----- configuration: must match the certified extra_fine model -----
DT = 0.1
ALPHA = 0.5                       # actuator gain (VehiclePlant.alpha)
SIGMA_V = np.sqrt(0.05**2 + 2.0**2) * DT   # 0.2001 m/s (PlantWrapper)
R = 0.1                           # resolution, all three dimensions

# grid bands with the half-cell offset: edges run bound -/+ r/2
GAP_LO, GAP_HI = 0.0 - R / 2, 16.0 + R / 2      # [-0.05, 16.05)
EGO_LO, EGO_HI = 4.0 - R / 2, 16.0 + R / 2      # [ 3.95, 16.05)
VL_LO, VL_HI = 5.0 - R / 2, 15.0 + R / 2        # [ 4.95, 15.05)

CRASH_EDGE = R / 2                # upper edge of the crash cell slab

# industry controller thresholds (AEBSController mode='industry')
TTC_BRAKE, TTC_EMERG = 1.6, 1.0
DIST_BRAKE, DIST_EMERG = 6.0, 2.0
ACC_BRAKE, ACC_EMERG = -4.0, -9.8

SCENARIOS = {
    'Following_Critical': (5.0, 14.0, 10.0),
    'Following_Tight':    (6.0, 14.0, 10.0),
    'Following_Near':     (8.0, 14.0, 10.0),
    'Following_Mid':      (10.0, 14.0, 10.0),
    'Following_Wide':     (12.0, 14.0, 10.0),
}

# certified Pmax at the fractional and adjacent rungs (18 Jul campaign);
# Pmin = 0 at every combination.
TARGETS = {
    'Following_Critical': {10: 0.001859491711, 12: 1.0},
    'Following_Tight':    {10: 1.90329532e-12, 12: 0.4890585366, 14: 1.0},
    'Following_Near':     {12: 2.923025817e-15, 14: 0.0001504356093,
                           16: 0.9893959729, 18: 1.0},
    'Following_Mid':      {14: 6.379221313e-22, 16: 1.834522015e-05,
                           18: 0.239932871, 20: 1.0},
    'Following_Wide':     {16: 4.951962645e-28, 18: 7.815790461e-06,
                           20: 0.2771274783, 22: 0.9998060877, 24: 1.0},
}
K_MAX = 24


def quantise(x):
    """Cell centre of the cell containing x (centres at multiples of R)."""
    return np.round(x / R) * R


def controller_acc(gap_c, vrel_c):
    """Vectorised industry decision logic on centre coordinates."""
    ttc = np.where(vrel_c > 0.1, gap_c / np.maximum(vrel_c, 1e-12), 999.0)
    acc = np.zeros_like(gap_c)
    low = vrel_c < 5.0
    acc = np.where(low & (gap_c < DIST_EMERG), ACC_EMERG, acc)
    acc = np.where(low & (gap_c >= DIST_EMERG) & (gap_c < DIST_BRAKE),
                   ACC_BRAKE, acc)
    acc = np.where(~low & (ttc < TTC_EMERG), ACC_EMERG, acc)
    acc = np.where(~low & (ttc >= TTC_EMERG) & (ttc < TTC_BRAKE),
                   ACC_BRAKE, acc)
    return acc


def self_check(n=2000, seed=1):
    """Assert vectorised logic == AEBSController on random states."""
    rng = np.random.default_rng(seed)
    gaps = rng.uniform(0, 16, n)
    vrels = rng.uniform(-11, 11, n)
    mine = controller_acc(gaps, vrels)
    ctrl = AEBSController(mode='industry')
    for g, vr, a in zip(gaps, vrels, mine):
        ref, _ = ctrl.get_action(True, g, vr)
        assert ref == a, f"controller mismatch at gap={g}, vrel={vr}: " \
                         f"{ref} != {a}"
    print(f"[self-check] vectorised controller == AEBSController "
          f"on {n} random states")


def truncated_normal(rng, mean, sigma, lo, hi):
    """Vectorised rejection sampler for Normal(mean, sigma) | [lo, hi]."""
    out = np.empty_like(mean)
    todo = np.ones(mean.shape, dtype=bool)
    while todo.any():
        draw = rng.normal(mean[todo], sigma)
        ok = (draw >= lo) & (draw < hi)
        idx = np.flatnonzero(todo)
        out[idx[ok]] = draw[ok]
        todo[idx[ok]] = False
    return out


def wilson_ci(k, n, z=1.96):
    """Wilson 95% interval for a binomial fraction."""
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    d = 1 + z * z / n
    c = p + z * z / (2 * n)
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return ((c - h) / d, (c + h) / d)


def run_scenario(name, state0, n_runs, rng):
    gap = np.full(n_runs, state0[0])
    ego = np.full(n_runs, state0[1])
    vlead = np.full(n_runs, state0[2])
    crash_step = np.full(n_runs, -1, dtype=int)   # -1 = no crash yet
    alive = np.ones(n_runs, dtype=bool)           # not crashed, not sunk

    for step in range(1, K_MAX + 1):
        if not alive.any():
            break
        # quantised controller on the live runs
        gc, ec, vc = quantise(gap[alive]), quantise(ego[alive]), \
            quantise(vlead[alive])
        acc = controller_acc(gc, ec - vc)
        # plant step (identical expressions to VehiclePlant)
        ego_n = np.maximum(0.0, ego[alive] + ALPHA * acc * DT)
        vl_n = truncated_normal(rng, np.maximum(0.0, vlead[alive]),
                                SIGMA_V, VL_LO, VL_HI)
        gap_n = gap[alive] + (vl_n - ego_n) * DT
        gap[alive], ego[alive], vlead[alive] = gap_n, ego_n, vl_n
        # crash: entry into the crash cell slab
        idx = np.flatnonzero(alive)
        crashed = gap[idx] < CRASH_EDGE
        crash_step[idx[crashed]] = step
        alive[idx[crashed]] = False
        # safe sink: left the window (gap top edge, ego band)
        idx = np.flatnonzero(alive)
        sunk = (gap[idx] >= GAP_HI) | (ego[idx] < EGO_LO) | \
               (ego[idx] >= EGO_HI)
        alive[idx[sunk]] = False

    print(f"\n=== {name}  start={state0}  n={n_runs} ===")
    print(f"{'k':>4} {'crashes':>8} {'fraction':>12} "
          f"{'Wilson 95% CI':>24} {'Pmax':>16} {'verdict':>10}")
    all_ok = True
    for k in sorted(TARGETS[name]):
        cnt = int(((crash_step > 0) & (crash_step <= k)).sum())
        frac = cnt / n_runs
        lo, hi = wilson_ci(cnt, n_runs)
        pmax = TARGETS[name][k]
        ok = lo <= pmax          # bracket check up to sampling error
        all_ok &= ok
        print(f"{k:>4} {cnt:>8} {frac:>12.6f} "
              f"[{lo:>10.6f}, {hi:>10.6f}] {pmax:>16.10g} "
              f"{'PASS' if ok else 'ALARM':>10}")
    return all_ok


def main():
    n_runs = int(sys.argv[1]) if len(sys.argv) > 1 else 10000
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 20260718
    print(f"[config] sigma_v={SIGMA_V:.6f}  band v_lead=[{VL_LO},{VL_HI})  "
          f"crash edge={CRASH_EDGE}  n={n_runs}  seed={seed}")
    self_check()
    rng = np.random.default_rng(seed)
    ok = True
    for name, s0 in SCENARIOS.items():
        ok &= run_scenario(name, s0, n_runs, rng)
    print("\n" + ("ALL BRACKET CHECKS PASS" if ok
                  else "AT LEAST ONE ALARM: investigate before shipping"))


if __name__ == '__main__':
    main()