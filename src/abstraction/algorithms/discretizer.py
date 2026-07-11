import numpy as np
import itertools
import math
from src.abstraction.types.imdp import IMDP
from src.abstraction.types.grid import Grid
from src.abstraction.types.cell_topology import (
    BOUNDARY_EPS, certainly_within, cells_touched, escapes_top,
    fully_outside_top, escapes_bottom, fully_outside_bottom, flat_index,
)
from multiprocessing import Pool, cpu_count
from functools import partial

# =====================================================================
#  BRANCH: soudjani2013-abstraction  —  SOUND OVER-APPROXIMATION VERSION
#
#  Construction = Abate 2011 Markov set-chain (interval transitions)
#                 populated by the SA13 Eq.3.11 per-transition RANGE bound.
#
#  Soundness condition (per-transition containment):
#     T(A_j | s, a) in [P_lo, P_hi]  for EVERY continuous s in source cell A_i.
#  Verified by tests/test_containment.py.
#
#  Key change vs the old kernel: deterministic dimensions (gap, v_ego) are no
#  longer collapsed to the cell-centre image. The IMAGE BOX of the whole source
#  cell is computed (corner evaluation; exact for the monotone AEBS map) and
#  every target cell the image touches is bracketed. v_lead uses the tight
#  [m, M] range bound.
# =====================================================================

SQRT2   = 1.4142135623730951
SQRT2PI = 2.5066282746310002

def fast_norm_cdf(x, mean, std):
    return 0.5 * (1 + math.erf((x - mean) / (std * SQRT2)))

# Physical gap at or below which the ego vehicle has crashed.
CRASH_WALL = 0.0

# Truncate + renormalise the v_lead Gaussian so all mass lands in-grid.
#   True  -> no safe-sink tail for the noise; cleanly sound (recommended).
#   False -> residual tail routed to the safe sink (small ~1e-6 approximation).
TRUNCATE_NOISE = True

# v_lead enumeration half-width, in sigmas (cells beyond this are negligible).
SIGMA_CUTOFF = 6.0

# BOUNDARY_EPS and every boundary-convention comparison (cell assignment,
# certain coverage, escape, enumeration, flat indexing) live in
# src.abstraction.types.cell_topology -- the single source of truth for the
# half-open [lo, hi) convention. Do not reintroduce inline comparisons here.


# ---------------------------------------------------------------------
#  SA13 Eq. 3.11 range, as the tight [m, M] of the box-probability factor
# ---------------------------------------------------------------------
def box_prob_minmax(mu_lo, mu_hi, tgt_lo, tgt_hi, sigma):
    """
    (min, max) of  g(mu) = Phi((tgt_hi-mu)/sigma) - Phi((tgt_lo-mu)/sigma)
    as the kernel MEAN mu ranges over [mu_lo, mu_hi].  M - m == the SA13 epsilon.
    g peaks at mu* = (tgt_lo+tgt_hi)/2 and is monotone on either side.
    """
    if sigma <= 0:
        return (0.0, 0.0)
    def g(mu):
        return fast_norm_cdf(tgt_hi, mu, sigma) - fast_norm_cdf(tgt_lo, mu, sigma)
    mu_peak = (tgt_lo + tgt_hi) / 2.0
    M = g(mu_peak) if mu_lo <= mu_peak <= mu_hi else max(g(mu_lo), g(mu_hi))
    m = min(g(mu_lo), g(mu_hi))
    return (m, M)


def compute_range_epsilon_ij(mu_lo, mu_hi, tgt_lo, tgt_hi, sigma):
    """Kept as a diagnostic: epsilon = M - m (SA13 Eq. 3.11)."""
    m, M = box_prob_minmax(mu_lo, mu_hi, tgt_lo, tgt_hi, sigma)
    return max(0.0, M - m)


# ---------------------------------------------------------------------
#  Image box of the source cell under the deterministic dynamics.
#  Evaluates the wrapper at the 8 corners; exact for monotone dynamics.
# ---------------------------------------------------------------------
def make_image_box(system_wrapper, cell_lo, cell_hi, act_val):
    """Returns (next_lo, next_hi, sigma_per_dim).

    cell_lo and cell_hi are the EXACT per-dimension cell edge floats
    (bins[d][k] and bins[d][k+1]), not a centre and half-width. Corners are
    evaluated at these exact floats. Reconstructing corners as centre +/-
    half-width is NOT bit-identical to the edges: (lo+hi)/2 - (hi-lo)/2 can
    differ from lo by ulps, and a lower corner drifting ABOVE an included
    lower face means the image box can exclude the image of a state that is
    genuinely in the half-open cell. Evaluating at the exact edges makes the
    box cover the closure of the cell exactly, which covers every in-cell
    state with no ulp gap on the included faces.

    SOUNDNESS GUARD (constant sigma per cell):
        The kernel below uses ONE sigma per dimension for the whole source cell.
        That is exact only when the noise std is constant across the cell. We
        therefore read sigma at every corner and assert it equals the centre
        sigma. With constant noise this is a no-op. The moment sigma becomes
        state-dependent (Track 2: speed-dependent process noise, or perception
        noise injected per dimension), the corners disagree and this raises --
        a deliberate trip-wire, because the kernel would otherwise be UNSOUND:
        box_prob would need to be ranged over the sigma INTERVAL across the
        cell (worst-case width), not evaluated at a single sigma. Do not remove
        this guard to silence it; implement the sigma-ranging instead.
    """
    center = np.array([(cell_lo[d] + cell_hi[d]) / 2.0 for d in range(3)])
    _, sigma_center = system_wrapper.get_next_state_distribution(center, act_val)
    next_lo = [math.inf, math.inf, math.inf]
    next_hi = [-math.inf, -math.inf, -math.inf]
    sig_lo = [float(s) for s in sigma_center]
    sig_hi = [float(s) for s in sigma_center]
    for picks in itertools.product((0, 1), repeat=3):
        corner = np.array([cell_lo[d] if picks[d] == 0 else cell_hi[d]
                           for d in range(3)])
        nd, sig = system_wrapper.get_next_state_distribution(corner, act_val)
        for d in range(3):
            if nd[d] < next_lo[d]: next_lo[d] = nd[d]
            if nd[d] > next_hi[d]: next_hi[d] = nd[d]
            if sig[d] < sig_lo[d]: sig_lo[d] = sig[d]
            if sig[d] > sig_hi[d]: sig_hi[d] = sig[d]

    for d in range(3):
        if sig_hi[d] - sig_lo[d] > 1e-12:
            raise ValueError(
                f"sigma varies across the source cell in dim {d} "
                f"(range [{sig_lo[d]:.6g}, {sig_hi[d]:.6g}]). The current kernel "
                f"assumes a single sigma per cell and is only SOUND for constant "
                f"noise. Before enabling state-dependent or perception noise, "
                f"range box_prob_minmax over the sigma interval (use sig_hi for "
                f"the upper transition bound, sig_lo for the lower).")

    return next_lo, next_hi, sigma_center


# ---------------------------------------------------------------------
#  Sound kernel
# ---------------------------------------------------------------------
def _compute_kernel_accurate(next_lo, next_hi, sigma_per_dim,
                             bins, grid_shape, total_states):
    if next_hi[0] <= CRASH_WALL:
        return {0: (1.0, 1.0)}                 # whole image has crashed
    crash_possible = (next_lo[0] <= CRASH_WALL)

    # ---- deterministic-dimension grid escape (top edge; bottom for non-gap dims) ----
    # Cells are half-open [lo, hi), so an image point exactly on the grid's top
    # edge is OUTSIDE the grid. Escaping mass must be routed to the safe sink
    # (state id == total_states): leaving it unassigned makes the true kernel
    # non-representable in the interval row, and a WHOLLY escaped image would
    # otherwise produce an EMPTY row, i.e. a missing PRISM command and a
    # deadlock in the composed model. Gap (d == 0) has no bottom escape: the
    # region at and below the crash wall is handled by the crash routing above.
    escape_possible = False
    for d in range(3):
        if sigma_per_dim[d] > 0:
            continue
        lo_orig, hi = next_lo[d], next_hi[d]
        if fully_outside_top(lo_orig, bins[d]) or \
                (d != 0 and fully_outside_bottom(hi, bins[d])):
            return {total_states: (1.0, 1.0)}   # whole image outside the grid
        if escapes_top(hi, bins[d]) or \
                (d != 0 and escapes_bottom(lo_orig, bins[d])):
            escape_possible = True

    stoch = [d for d in range(3) if sigma_per_dim[d] > 0]
    if len(stoch) > 1:
        raise NotImplementedError(
            "Truncation renormalisation assumes exactly one stochastic "
            "dimension; extend the in-grid-mass bracketing before enabling "
            "noise on a second dimension (Track 2).")

    # in-grid v_lead mass over the mean range (for truncation renormalisation).
    # ins(mu) = Phi(b_max - mu) - Phi(b_min - mu) is unimodal in mu with its
    # peak at the grid-band midpoint (same structure as box_prob_minmax), so:
    #   - its MIN over [mu_lo, mu_hi] is at an endpoint (inside_lo below is a
    #     true lower bound -> M / inside_lo is a sound upper bound), and
    #   - its MAX may be INTERIOR; the midpoint check below makes inside_hi a
    #     true upper bound on ins over the whole mean range, so m / inside_hi
    #     is a sound lower bound by  g(mu) >= m  and  ins(mu) <= inside_hi,
    #     with no auxiliary lemma about the renormalised ratio required.
    inside_lo, inside_hi = 1.0, 0.0
    if stoch:
        d = stoch[0]
        def _ins(mu):
            return fast_norm_cdf(bins[d][-1], mu, sigma_per_dim[d]) \
                 - fast_norm_cdf(bins[d][0],  mu, sigma_per_dim[d])
        for mu in (next_lo[d], next_hi[d]):
            ins = _ins(mu)
            inside_lo = min(inside_lo, ins)
            inside_hi = max(inside_hi, ins)
        band_mid = (bins[d][0] + bins[d][-1]) / 2.0
        if next_lo[d] <= band_mid <= next_hi[d]:
            inside_hi = max(inside_hi, _ins(band_mid))
        if TRUNCATE_NOISE and inside_lo <= 0.0:
            raise ValueError(
                "In-grid mass lower bound is 0 for this source cell: the mean "
                "range lies so far outside the grid band that truncation "
                "renormalisation is ill-defined. Widen the grid band; do not "
                "silently skip the renormalisation of the upper bound.")

    # per-dimension (cell, factor_lo, factor_hi)
    dim_cells = []
    for d in range(3):
        lo_orig, hi = next_lo[d], next_hi[d]                 # UNCLAMPED image
        lo_enum = max(lo_orig, CRASH_WALL) if d == 0 else lo_orig  # for enumeration
        if sigma_per_dim[d] > 0:
            nr  = SIGMA_CUTOFF * sigma_per_dim[d]
            klo, khi = cells_touched(lo_enum, hi, bins[d], pad=nr)
            cells = []
            for k in range(klo, khi + 1):
                m, M = box_prob_minmax(lo_orig, hi, bins[d][k], bins[d][k + 1],
                                       sigma_per_dim[d])
                if TRUNCATE_NOISE and inside_hi > 0.0:
                    m = m / inside_hi
                    M = M / inside_lo if inside_lo > 0.0 else M
                if M > 1e-12:
                    cells.append((k, m, M))
            dim_cells.append(cells)
        else:
            klo, khi = cells_touched(lo_enum, hi, bins[d])
            cells = []
            for k in range(klo, khi + 1):
                # Certain coverage requires the FULL UNCLAMPED image inside the
                # half-open cell (strict upper inequality; see cell_topology.
                # certainly_within for the rationale and the containment
                # violation the non-strict version causes). If part of the
                # source image crosses the crash wall, lo_orig <= 0 < clo for
                # every non-crash cell, so the lower factor is 0 automatically.
                f_lo = 1.0 if certainly_within(lo_orig, hi,
                                               bins[d][k], bins[d][k + 1]) else 0.0
                cells.append((k, f_lo, 1.0))
            dim_cells.append(cells)

    transitions = {}
    for (kg, glo, ghi) in dim_cells[0]:
        for (kv, vlo, vhi) in dim_cells[1]:
            for (kl, llo, lhi) in dim_cells[2]:
                p_lo = glo * vlo * llo
                p_hi = ghi * vhi * lhi
                if p_hi <= 1e-12:
                    continue
                flat = flat_index(kg, kv, kl, grid_shape)
                if flat in transitions:
                    a, b = transitions[flat]
                    transitions[flat] = (min(a, p_lo), max(b, p_hi))
                else:
                    transitions[flat] = (p_lo, p_hi)

    if crash_possible:
        transitions[0] = (0.0, 1.0)

    # safe sink for the v_lead tail (only when NOT truncating)
    if stoch and not TRUNCATE_NOISE:
        tail_lo = max(0.0, 1.0 - inside_hi)
        tail_hi = max(0.0, 1.0 - inside_lo)
        if tail_hi > 1e-9:
            if total_states in transitions:
                lo, hi = transitions[total_states]
                transitions[total_states] = (min(1.0, lo + tail_lo),
                                             min(1.0, hi + tail_hi))
            else:
                transitions[total_states] = (tail_lo, tail_hi)

    # partial deterministic escape: some (not all) states' images leave the grid
    if escape_possible:
        if total_states in transitions:
            lo, hi = transitions[total_states]
            transitions[total_states] = (lo, 1.0)
        else:
            transitions[total_states] = (0.0, 1.0)

    return transitions


# ---------------------------------------------------------------------
#  Worker
# ---------------------------------------------------------------------
def process_chunk(chunk_indices, grid_bins, grid_shape, grid_total_states,
                  system_action_space, system_wrapper, grid_resolution):
    results = []

    def get_flat_index(idx_tuple):
        return flat_index(idx_tuple[0], idx_tuple[1], idx_tuple[2], grid_shape)

    def index_to_cell_center(idx_tuple):
        return np.array([
            (grid_bins[i][idx_tuple[i]] + grid_bins[i][idx_tuple[i] + 1]) / 2.0
            for i in range(3)
        ])

    for idx in chunk_indices:
        src_id = get_flat_index(idx)

        if src_id == 0:                          # crash is absorbing
            for act_id in system_action_space.keys():
                results.append((src_id, act_id, {0: (1.0, 1.0)}))
            continue

        cell_lo = [grid_bins[d][idx[d]] for d in range(3)]
        cell_hi = [grid_bins[d][idx[d] + 1] for d in range(3)]

        for act_id, act_val in system_action_space.items():
            next_lo, next_hi, sigma_per_dim = make_image_box(
                system_wrapper, cell_lo, cell_hi, act_val)
            transitions = _compute_kernel_accurate(
                next_lo, next_hi, sigma_per_dim,
                grid_bins, grid_shape, grid_total_states)
            results.append((src_id, act_id, transitions))
    return results


# ---------------------------------------------------------------------
#  Diagnostic only — E is NOT the soundness mechanism (kept for reporting).
#  NOTE ON NAMING: K below is the SUM over the row's targets of the interval
#  half-widths (p_max - p_min)/2, i.e. half the row's total interval width.
#  It is not a mean; the pipeline log line calling it a "mean interval
#  half-width" should be corrected, and the paper must not describe it as one.
# ---------------------------------------------------------------------
def compute_global_error_bound(imdp: IMDP, N_horizon: int) -> dict:
    max_K = 0.0; worst_state = None; worst_action = None
    for src, actions in imdp.transitions.items():
        for act, dist_str in actions.items():
            K = 0.0
            for part in dist_str.split(' + '):
                try:
                    bracket = part.strip().split(']')[0].lstrip('[')
                    p_min_str, p_max_str = bracket.split(',')
                    K += (float(p_max_str) - float(p_min_str)) / 2.0
                except (ValueError, IndexError):
                    continue
            if K > max_K:
                max_K, worst_state, worst_action = K, src, act
    return {'E': N_horizon * max_K, 'max_K': max_K,
            'worst_state': worst_state, 'worst_action': worst_action}


# ---------------------------------------------------------------------
class DiscretizationAlgorithm:
    def __init__(self, grid: Grid):
        self.grid = grid

    def run(self, system, name: str) -> IMDP:
        print(f"[Discretizer] Parallel Processing '{name}' over {self.grid.total_states} states...")
        imdp = IMDP(name=name)
        actions = system.get_action_space()

        all_indices = list(itertools.product(*[range(d) for d in self.grid.shape]))
        num_cores = max(1, cpu_count() - 1)
        chunks = [all_indices[i:i + 1000] for i in range(0, len(all_indices), 1000)]
        total_chunks = len(chunks)

        print(f"[Discretizer] Using {num_cores} cores. "
              f"Sound over-approximation (image-box deterministic dims, "
              f"SA13 range on v_lead, truncate_noise={TRUNCATE_NOISE}).")

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
                    print(f"[Discretizer] Progress: {(counter/total_chunks)*100:.1f}% "
                          f"({counter}/{total_chunks})", end='\r')
                for (src_id, act_id, transitions) in batch_results:
                    if not transitions:
                        continue
                    updates = [f"[{p_min:.8f}, {p_max:.8f}] : (s'={tgt_id})"
                               for tgt_id, (p_min, p_max) in transitions.items()]
                    imdp.add_transition(src_id, act_id, " + ".join(updates))

        print("\n[Discretizer] Aggregation Complete!")
        return imdp