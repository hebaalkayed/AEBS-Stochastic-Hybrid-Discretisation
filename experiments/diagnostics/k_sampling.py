"""
K sampling diagnostic — tests whether state-dependent operational noise
introduces spatial heterogeneity in the SA13 per-row error K(i,a).

Background
----------
The constant-sigma sweep (sigma_sweep.py) showed K is uniform across the
state space, which is what kills SA13's Algorithm 3 — adaptive gridding
needs hot spots to refine. SA13's DNA case study has state-dependent
variance baked into the chemistry (sigma_2 depends on x_1, x_2). AEBS
doesn't naturally have this with a constant noise model.

Hypothesis
----------
Operational lead-vehicle acceleration noise grows with lead speed
(Treiber & Kesting 2013, Ch. 11-12; IDM literature). A defensible
model is:

    sigma_a(v_lead) = sigma_0 + alpha * max(0, v_lead)   [m/s^2]
    sigma_v(v_lead) = sigma_a(v_lead) * dt               [m/s]

Calibrated so sigma_v(30) ≈ 0.2 m/s (matches current production constant
at highway speeds) and sigma_v(0) = 0.01 m/s (stopped vehicles don't
wobble).

Go/no-go
--------
If K varies by 3x or more across the v_lead range, adaptive gridding
has something to refine and stage two is worth the engineering. If K
stays nearly flat, rule it out and pursue a different direction.

Run from repo root:
    python experiments/diagnostics/k_sampling.py
"""
import sys
import os
import itertools
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from src.system.vehicle_plant import VehiclePlant
from src.system.controller import AEBSController
from src.abstraction.types.grid import Grid
from src.abstraction.modules.wrappers import PlantWrapper
from src.abstraction.algorithms.discretizer import (
    fast_norm_cdf, compute_range_epsilon_ij,
)


# --- speed-dependent noise model ---
SIGMA_FLOOR = 0.01   # m/s on velocity, at v_lead=0 (stopped vehicle)
SIGMA_SLOPE = 0.0063  # so sigma_v(30) ~ 0.2 m/s (matches production)


def sigma_v_fn(v_lead):
    """sigma_v(v_lead) = sigma_0 + alpha * max(0, v_lead), in m/s."""
    return SIGMA_FLOOR + SIGMA_SLOPE * max(0.0, v_lead)


def compute_K(src_id, act_id, grid, plant_wrapper):
    """
    Compute K(src_id, act_id) for the SA13 range bound.

    Same logic as dump_row.py but returns numbers instead of printing.

    Returns:
        K_in:      sum of per-target epsilons over in-grid targets
        eps_sink:  epsilon for the safe-sink target (or 0 if p_sink negligible)
        K_total:   K_in + eps_sink (this is the K(i,a) used in E = N * max K)
        n_targets: number of in-grid targets touched
        sigma_v:   actual sigma_v used (for reporting)
    """
    n_g, n_v, n_vl = grid.shape
    i_g = src_id // (n_v * n_vl)
    rem = src_id % (n_v * n_vl)
    i_v, i_vl = rem // n_vl, rem % n_vl
    idx = (i_g, i_v, i_vl)
    bins = grid.bins

    centre = np.array([(bins[d][idx[d]] + bins[d][idx[d]+1])/2.0 for d in range(3)])
    half_cell = [(bins[d][idx[d]+1] - bins[d][idx[d]])/2.0 for d in range(3)]

    act_val = plant_wrapper.get_action_space()[act_id]
    next_c, sigma = plant_wrapper.get_next_state_distribution(centre, act_val)

    if next_c[0] <= 0.0:
        return 0.0, 0.0, 0.0, 0, sigma[2]  # crash early return

    stoch = [d for d in range(3) if sigma[d] > 0]
    if not stoch:
        return 0.0, 0.0, 0.0, 0, 0.0
    nrad = 4.0 * max(sigma)

    ranges = []
    for d in range(3):
        if sigma[d] > 0:
            s = np.digitize(next_c[d] - nrad, bins[d]) - 1
            e = np.digitize(next_c[d] + nrad, bins[d]) - 1
        else:
            s = e = np.digitize(next_c[d], bins[d]) - 1
        ranges.append(range(max(0, s), min(grid.shape[d]-1, e) + 1))

    K_in, total_in, n_targets = 0.0, 0.0, 0
    for tgt in itertools.product(*ranges):
        prob = 1.0
        for d in range(3):
            lo, hi = bins[d][tgt[d]], bins[d][tgt[d]+1]
            if sigma[d] > 0:
                prob *= fast_norm_cdf(hi, next_c[d], sigma[d]) \
                      - fast_norm_cdf(lo, next_c[d], sigma[d])
            else:
                prob *= 1.0 if lo <= next_c[d] < hi else 0.0
        if prob <= 0.0:
            continue
        eps = 0.0
        for d in stoch:
            eps += compute_range_epsilon_ij(
                next_c[d] - half_cell[d], next_c[d] + half_cell[d],
                bins[d][tgt[d]], bins[d][tgt[d]+1], sigma[d])
        K_in += eps
        total_in += prob
        n_targets += 1

    p_sink = max(0.0, 1.0 - total_in)
    eps_sink = 0.0
    if p_sink > 1e-6:
        for d in stoch:
            eps_sink += compute_range_epsilon_ij(
                next_c[d] - half_cell[d], next_c[d] + half_cell[d],
                bins[d][0], bins[d][-1], sigma[d])
    K_total = K_in + eps_sink
    return K_in, eps_sink, K_total, n_targets, sigma[2]


def main():
    print("\n" + "=" * 88)
    print("  K SAMPLING — does state-dependent sigma_v create spatial heterogeneity in K?")
    print("=" * 88)

    plant = VehiclePlant(coordinate_system='relative_frame')
    controller = AEBSController(mode='safe')
    grid = Grid(preset='medium_tight')
    w = grid.resolution[2]

    pw_const = PlantWrapper(plant, controller, lead_noise_std=2.0)
    pw_var = PlantWrapper(plant, controller, sigma_v_fn=sigma_v_fn)

    sample_v_leads = [0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 28.0, 30.0]
    gap_fixed, v_ego_fixed = 10.0, 10.0

    print(f"\n[Sampling] grid = medium_tight (w_v_lead = {w})")
    print(f"[Sampling] source cells = (gap={gap_fixed}, v_ego={v_ego_fixed}, v_lead=X)")
    print(f"[Sampling] action = 0 (coast)")
    print(f"[Sampling] state-dep model: sigma_v(v) = {SIGMA_FLOOR} + {SIGMA_SLOPE} * max(0,v)\n")

    header = (f"  {'v_lead':>7} | "
              f"{'sigma(const)':>12} {'sigma/w':>8} {'K(const)':>10} | "
              f"{'sigma(var)':>11} {'sigma/w':>8} {'K(var)':>10}")
    print(header)
    print("-" * len(header))

    rows_const, rows_var = [], []
    for v_lead in sample_v_leads:
        idx = grid.state_to_index((gap_fixed, v_ego_fixed, v_lead))
        if idx is None:
            print(f"  {v_lead:>7.1f} | OUT OF GRID")
            continue
        src_id = grid.get_flat_index(idx)

        _, _, K_const, _, sig_const = compute_K(src_id, 0, grid, pw_const)
        _, _, K_var,   _, sig_var   = compute_K(src_id, 0, grid, pw_var)

        rows_const.append(K_const)
        rows_var.append(K_var)

        print(f"  {v_lead:>7.1f} | "
              f"{sig_const:>12.5f} {sig_const/w:>8.3f} {K_const:>10.5f} | "
              f"{sig_var:>11.5f} {sig_var/w:>8.3f} {K_var:>10.5f}")

    print("\n" + "=" * 88)

    def summarise(label, rows):
        rows = [r for r in rows if r > 0]
        if not rows:
            print(f"  {label}: no data")
            return
        ratio = max(rows) / min(rows)
        print(f"  {label}: K range [{min(rows):.4f}, {max(rows):.4f}], "
              f"heterogeneity ratio = {ratio:.2f}x")

    summarise("constant sigma", rows_const)
    summarise("state-dep sigma", rows_var)

    print("\n  GO/NO-GO:")
    print("    - constant: ratio should be ~1.0 (uniform K, as expected)")
    print("    - state-dep: ratio >= 3.0 means adaptive gridding has leverage")
    print("=" * 88 + "\n")


if __name__ == "__main__":
    main()