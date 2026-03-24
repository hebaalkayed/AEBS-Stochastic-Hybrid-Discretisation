import numpy as np
import itertools
import math
from src.abstraction.types.imdp import IMDP
from src.abstraction.types.grid import Grid
from multiprocessing import Pool, cpu_count
from functools import partial

# --- HELPER: FAST MATH ---
SQRT2   = 1.4142135623730951
SQRT2PI = 2.5066282746310002  # sqrt(2 * pi)

def fast_norm_cdf(x, mean, std):
    return 0.5 * (1 + math.erf((x - mean) / (std * SQRT2)))


# --- BRANCH: soudjani2013-abstraction ---
#
# BUG FIX v2 (mu range): compute epsilon in next-state space.
# BUG FIX v3 (crash wall): use CRASH_WALL = 0.0 instead of bins[0][0].
#
# BUG FIX v4 (crash early return):
#   Previously, crash routing was applied ADDITIVELY at the end of
#   _compute_kernel_accurate. For any state where next_center[0] <= 0,
#   the Gaussian search over v_lead ran first and populated the transitions
#   dict with valid intervals. Then p_crash = 1.0 was added on top, giving:
#
#       {1: (0.49, 1.0), 2: (0.49, 1.0), ..., 0: (1.0, 1.0)}
#
#   where p_min sums >> 1.0 — invalid for PRISM, which requires the
#   lower bounds to admit at least one consistent probability distribution.
#
#   Root cause: gap is purely deterministic (sigma_gap = 0). When centroid
#   dynamics predict next_gap <= 0, the crash probability is exactly 1 with
#   no uncertainty. Gaussian noise in v_lead is irrelevant — the vehicle has
#   already crashed regardless of where v_lead lands.
#
#   Fix: return {0: (1.0, 1.0)} immediately. The Gaussian search is skipped
#   entirely, so there is nothing to add crash to later.


# Physical gap at or below which the ego vehicle has crashed.
CRASH_WALL = 0.0


def compute_range_epsilon_ij(mu_lo: float, mu_hi: float,
                              tgt_lo: float, tgt_hi: float,
                              sigma: float) -> float:
    """
    Per-transition range-based error bound (Soudjani & Abate 2013, Eq. 3.11).

    Computes the range of g(mu) = Phi((tgt_hi-mu)/sigma) - Phi((tgt_lo-mu)/sigma)
    as mu varies over [mu_lo, mu_hi]:

        epsilon_ij = max g(mu) - min g(mu)

    Pass mu_lo = next_center[d] - half_cell[d],
         mu_hi = next_center[d] + half_cell[d]   (NEXT-STATE space)

    g peaks at mu* = (tgt_lo + tgt_hi) / 2 and is monotone on either side.
    """
    if sigma <= 0:
        return 0.0

    def g(mu):
        return fast_norm_cdf(tgt_hi, mu, sigma) - fast_norm_cdf(tgt_lo, mu, sigma)

    mu_peak = (tgt_lo + tgt_hi) / 2.0

    g_max = g(mu_peak) if mu_lo <= mu_peak <= mu_hi else max(g(mu_lo), g(mu_hi))
    g_min = min(g(mu_lo), g(mu_hi))

    return max(0.0, g_max - g_min)


# --- WORKER FUNCTION ---
def process_chunk(chunk_indices, grid_bins, grid_shape, grid_total_states,
                  system_action_space, system_wrapper, grid_resolution):
    results = []

    def get_flat_index(idx_tuple):
        flat = 0
        multiplier = 1
        for i in reversed(range(3)):
            flat += idx_tuple[i] * multiplier
            multiplier *= grid_shape[i]
        return flat

    def index_to_cell_center(idx_tuple):
        return np.array([
            (grid_bins[i][idx_tuple[i]] + grid_bins[i][idx_tuple[i] + 1]) / 2.0
            for i in range(3)
        ])

    for idx in chunk_indices:
        src_id = get_flat_index(idx)

        # Crash state (s=0) is absorbing
        if src_id == 0:
            for act_id in system_action_space.keys():
                results.append((src_id, act_id, {0: (1.0, 1.0)}))
            continue

        center = index_to_cell_center(idx)

        half_cell = [
            (grid_bins[d][idx[d] + 1] - grid_bins[d][idx[d]]) / 2.0
            for d in range(3)
        ]

        for act_id, act_val in system_action_space.items():
            next_c, sigma_per_dim = system_wrapper.get_next_state_distribution(
                center, act_val
            )

            transitions = _compute_kernel_accurate(
                next_c, sigma_per_dim, half_cell,
                grid_bins, grid_shape, grid_total_states
            )

            if src_id == 1 and act_id == 0:
                epsilons = [hi - lo for (lo, hi) in transitions.values()]
                if epsilons:
                    nonzero = sum(1 for e in epsilons if e > 1e-8)
                    print(f"\n[DEBUG soudjani2013 v4] src=1, act=0 | "
                          f"sigma={sigma_per_dim[2]:.4f} | "
                          f"next_c[2]={next_c[2]:.4f} | "
                          f"n_transitions={len(transitions)} | "
                          f"non-trivial intervals: {nonzero}/{len(transitions)} | "
                          f"epsilon range=[{min(epsilons):.5f}, {max(epsilons):.5f}]")
                    sample_tgt, (lo, hi) = next(iter(transitions.items()))
                    status = 'NON-ZERO' if lo > 1e-5 else 'floor (1e-6)'
                    print(f"  sample -> s={sample_tgt}: [{lo:.6f}, {hi:.6f}]  "
                          f"lower bound {status}")

            results.append((src_id, act_id, transitions))
    return results


def _compute_kernel_accurate(next_center, sigma_per_dim, half_cell,
                              bins, grid_shape, total_states):
    """
    Computes IMDP transitions with SA13 Eq. 3.11 range-based epsilon bounds.

    BUG FIX v4 — Crash early return:
        Gap is deterministic. When next_center[0] <= CRASH_WALL the transition
        to s=0 is certain. We return immediately so the Gaussian search over
        v_lead never runs, preventing the invalid additive combination:

            Gaussian intervals  +  p_crash=1.0  =>  p_min sum >> 1.0

        After this guard, crash can never occur inside the loop below, so the
        additive crash block at the end of the old code is removed entirely.

    Args:
        next_center   : deterministic next state (centroid dynamics)
        sigma_per_dim : [0, 0, sigma_v_lead] for AEBS
        half_cell     : per-dim half-widths of source cell
        bins          : grid bin edges per dimension
        grid_shape    : number of cells per dimension
        total_states  : safe_sink id = total_states

    Returns:
        dict { target_state_id: (p_min, p_max) }
    """
    # --- BUG FIX v4: crash early return ---
    # Gap is deterministic (sigma[0] = 0). If centroid dynamics predict
    # next_gap <= 0, the crash is certain — skip the Gaussian search entirely.
    if next_center[0] <= CRASH_WALL:
        return {0: (1.0, 1.0)}

    # --- Grid search (only reached when next_gap > 0) ---
    stochastic_dims = [d for d in range(3) if sigma_per_dim[d] > 0]
    max_sigma = max(sigma_per_dim) if stochastic_dims else 0.0
    noise_radius = 4.0 * max_sigma

    ranges = []
    for i in range(3):
        if sigma_per_dim[i] > 0:
            s = np.digitize(next_center[i] - noise_radius, bins[i]) - 1
            e = np.digitize(next_center[i] + noise_radius, bins[i]) - 1
        else:
            s = np.digitize(next_center[i], bins[i]) - 1
            e = s
        ranges.append(range(max(0, s), min(grid_shape[i] - 1, e) + 1))

    transitions = {}
    total_inside_mass = 0.0

    for tgt_idx in itertools.product(*ranges):

        prob = 1.0
        for d in range(3):
            lo   = bins[d][tgt_idx[d]]
            hi   = bins[d][tgt_idx[d] + 1]
            mean = next_center[d]
            if sigma_per_dim[d] > 0:
                prob *= fast_norm_cdf(hi, mean, sigma_per_dim[d]) \
                      - fast_norm_cdf(lo, mean, sigma_per_dim[d])
            else:
                prob *= 1.0 if lo <= mean < hi else 0.0

        if prob <= 0.0:
            continue

        epsilon_ij = 0.0
        for d in stochastic_dims:
            mu_lo    = next_center[d] - half_cell[d]
            mu_hi    = next_center[d] + half_cell[d]
            tgt_lo_d = bins[d][tgt_idx[d]]
            tgt_hi_d = bins[d][tgt_idx[d] + 1]
            epsilon_ij += compute_range_epsilon_ij(
                mu_lo, mu_hi, tgt_lo_d, tgt_hi_d, sigma_per_dim[d]
            )

        p_min = max(0.0, prob - epsilon_ij)
        p_max = min(1.0, prob + epsilon_ij)

        flat = 0
        mult = 1
        for i in reversed(range(3)):
            flat += tgt_idx[i] * mult
            mult *= grid_shape[i]

        transitions[flat] = (max(1e-6, p_min), p_max)
        total_inside_mass += prob

    # --- Safe sink: residual Gaussian tail outside grid ---
    # next_gap > 0 so p_crash = 0; all residual goes to safe_sink.
    p_safe_sink = max(0.0, 1.0 - total_inside_mass)
    if p_safe_sink > 1e-6:
        epsilon_sink = 0.0
        for d in stochastic_dims:
            mu_lo     = next_center[d] - half_cell[d]
            mu_hi     = next_center[d] + half_cell[d]
            grid_lo_d = bins[d][0]
            grid_hi_d = bins[d][-1]
            epsilon_sink += compute_range_epsilon_ij(
                mu_lo, mu_hi, grid_lo_d, grid_hi_d, sigma_per_dim[d]
            )
        s_min = max(1e-6, p_safe_sink - epsilon_sink)
        s_max = min(1.0,  p_safe_sink + epsilon_sink)
        if total_states in transitions:
            lo, hi = transitions[total_states]
            transitions[total_states] = (min(1.0, lo + s_min), min(1.0, hi + s_max))
        else:
            transitions[total_states] = (s_min, s_max)

    return transitions


# --- MAIN CLASS ---
class DiscretizationAlgorithm:
    def __init__(self, grid: Grid):
        self.grid = grid

    def run(self, system, name: str) -> IMDP:
        print(f"[Discretizer] Parallel Processing '{name}' over {self.grid.total_states} states...")

        imdp = IMDP(name=name)
        actions = system.get_action_space()

        all_indices = list(itertools.product(*[range(d) for d in self.grid.shape]))
        num_cores = max(1, cpu_count() - 1)
        fixed_chunk_size = 1000
        chunks = [all_indices[i:i + fixed_chunk_size]
                  for i in range(0, len(all_indices), fixed_chunk_size)]
        total_chunks = len(chunks)

        print(f"[Discretizer] Using {num_cores} cores. "
              f"Mode: Range-based epsilon (Soudjani & Abate 2013, Eq. 3.11) — "
              f"mu range in next-state space (v4).")

        with Pool(processes=num_cores) as pool:
            func = partial(process_chunk,
                           grid_bins=self.grid.bins,
                           grid_shape=self.grid.shape,
                           grid_total_states=self.grid.total_states,
                           system_action_space=actions,
                           system_wrapper=system,
                           grid_resolution=np.array(self.grid.resolution))

            counter = 0
            for batch_results in pool.imap_unordered(func, chunks):
                counter += 1
                if counter % 10 == 0 or counter == total_chunks:
                    percent = (counter / total_chunks) * 100
                    print(f"[Discretizer] Progress: {percent:.1f}% "
                          f"({counter}/{total_chunks})", end='\r')

                for (src_id, act_id, transitions) in batch_results:
                    if not transitions:
                        continue
                    updates = []
                    for tgt_id, (p_min, p_max) in transitions.items():
                        updates.append(f"[{p_min:.8f}, {p_max:.8f}] : (s'={tgt_id})")
                    imdp.add_transition(src_id, act_id, " + ".join(updates))

        print("\n[Discretizer] Aggregation Complete!")
        return imdp