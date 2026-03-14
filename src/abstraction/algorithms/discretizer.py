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

def compute_lq(sigma: float) -> float:
    """
    Lipschitz constant of the Gaussian kernel interval probability
    w.r.t. a shift in the kernel centre (mean).

    For P = Phi((high-mu)/sigma) - Phi((low-mu)/sigma),
    the maximum |dP/dmu| equals the PDF peak = 1/(sigma*sqrt(2pi)).
    Both cell boundaries contribute, giving 2/(sigma*sqrt(2pi)).
    """
    if sigma <= 0:
        return 0.0
    return 2.0 / (sigma * SQRT2PI)

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
        return np.array([(grid_bins[i][idx_tuple[i]] + grid_bins[i][idx_tuple[i]+1])/2.0 for i in range(3)])

    for idx in chunk_indices:
        src_id = get_flat_index(idx)

        # Crash state (s=0) is absorbing
        if src_id == 0:
            for act_id in system_action_space.keys():
                results.append((src_id, act_id, {0: (1.0, 1.0)}))
            continue

        center = index_to_cell_center(idx)

        for act_id, act_val in system_action_space.items():
            # A. Query Physics
            # sigma_per_dim: per-dimension noise std (0.0 = deterministic indicator kernel)
            next_c, sigma_per_dim, L_t, L_q = system_wrapper.get_next_state_distribution(center, act_val)

            # B. Calculate Formal Error Bound (Abate et al. Theorem 1)
            #
            # CORRECT FORMULA:  epsilon = (L_t + L_q) * delta_stochastic
            # where delta_stochastic = resolution[stochastic_dim] / 2.0
            #
            # This is the per-dimension form of the Abate bound, valid for
            # separable kernels where noise enters only one dimension.
            #
            # WHY NOT global cell_diameter:
            #   The naive global form epsilon = (L_t + L_q) * cell_diameter
            #   uses cell_diameter = 2 * ||resolution/2|| which mixes physical
            #   units (metres for gap, m/s for velocities) in a Euclidean norm.
            #   For AEBS with resolution ≈ [3m, 0.6m/s, 0.25m/s]:
            #       cell_diameter = 2 * ||[1.5, 0.3, 0.125]|| ≈ 3.08  (incoherent units)
            #       L_q           = 2 / (0.2 * sqrt(2pi))     ≈ 4.0   (1/m/s units)
            #       epsilon       = (1 + 4) * 3.08             ≈ 18.5  >> 1
            #   → every interval collapses to [1e-6, 1.0] — formally sound, informationally void.
            #
            # OLD (INCORRECT) FORMULA — kept for reference:
            #   cell_volume = np.prod(grid_resolution)
            #   cell_radius = np.linalg.norm(grid_resolution / 2.0)
            #   epsilon = cell_volume * (L_t + L_q) * cell_radius
            # This had no basis in Abate et al. but accidentally suppressed blowup
            # because cell_volume << 1 partially compensated for the error.
            #
            # WITH THIS FIX:
            #   stochastic_dim    = 2  (v_lead — only noisy dimension)
            #   delta_stochastic  = resolution[2] / 2.0 = 0.125 m/s
            #   epsilon           = (1.0 + 4.0) * 0.125 = 0.625   ← meaningful ✓
            #   Central cell prob ≈ 0.68  →  interval [0.055, 1.0] ✓
            #   Neighbour cell prob ≈ 0.16 → interval [0.0, 0.785] ✓
            #
            stochastic_dims = np.where(sigma_per_dim > 0)[0]
            if len(stochastic_dims) > 0:
                # Use the stochastic dimension with the largest cell half-width
                # (conservative choice when multiple stochastic dims exist)
                stochastic_dim = stochastic_dims[np.argmax(grid_resolution[stochastic_dims])]
                delta_stochastic = grid_resolution[stochastic_dim] / 2.0
            else:
                # Fully deterministic system: use minimum cell half-width
                delta_stochastic = np.min(grid_resolution) / 2.0

            epsilon = (L_t + L_q) * delta_stochastic
            # DEBUG — remove after confirming
            if src_id == 1:
                print(f"\n[DEBUG] sigma_per_dim={sigma_per_dim}, delta_stochastic={delta_stochastic:.4f}, L_t={L_t:.3f}, L_q={L_q:.3f}, epsilon={epsilon:.4f}")
    
            # C. Compute Transition Kernel (per-dimension sigma, strict crash priority)
            transitions = _compute_kernel_accurate(
                next_c, sigma_per_dim, epsilon,
                grid_bins, grid_shape, grid_total_states
            )

            results.append((src_id, act_id, transitions))
    return results


def _compute_kernel_accurate(next_center, sigma_per_dim, epsilon, bins, grid_shape, total_states):
    """
    Computes transitions using per-dimension Gaussian kernels.

    For each dimension:
        sigma > 0: integrate Gaussian CDF over target cell bounds (stochastic)
        sigma = 0: indicator function — 1.0 if next_center falls in cell, else 0.0

    This correctly models the AEBS plant where only v_lead (dim 2) is stochastic.
    Deterministic dimensions (gap, v_ego) use indicator kernels to avoid leaking
    mass into physically unreachable cells.
    """
    # Search radius: use max stochastic sigma, or small fallback for deterministic dims
    max_sigma = np.max(sigma_per_dim)
    noise_radius = 4 * max_sigma if max_sigma > 0 else 0.0

    ranges = []
    for i in range(3):
        if sigma_per_dim[i] > 0:
            # Stochastic dim: search within 4-sigma radius
            s = np.digitize(next_center[i] - noise_radius, bins[i]) - 1
            e = np.digitize(next_center[i] + noise_radius, bins[i]) - 1
        else:
            # Deterministic dim: only the cell containing next_center
            s = np.digitize(next_center[i], bins[i]) - 1
            e = s
        ranges.append(range(max(0, s), min(grid_shape[i] - 1, e) + 1))

    transitions = {}
    total_inside_mass = 0.0
    candidates = []

    for idx in itertools.product(*ranges):
        prob = 1.0

        for d in range(3):
            low  = bins[d][idx[d]]
            high = bins[d][idx[d] + 1]
            mean = next_center[d]

            if sigma_per_dim[d] > 0:
                # Gaussian CDF integration for stochastic dimension
                p_high = fast_norm_cdf(high, mean, sigma_per_dim[d])
                p_low  = fast_norm_cdf(low,  mean, sigma_per_dim[d])
                prob  *= (p_high - p_low)
            else:
                # Indicator function for deterministic dimension
                prob  *= (1.0 if low <= mean < high else 0.0)

        p_min = max(0.0, prob - epsilon)
        p_max = min(1.0, prob + epsilon)

        if p_max > 0:
            flat = 0
            mult = 1
            for i in reversed(range(3)):
                flat += idx[i] * mult
                mult *= grid_shape[i]

            candidates.append({'id': flat, 'min': p_min, 'max': p_max, 'prob': prob})

    # NO PRUNING — retain all candidates within the search radius.
    # Abate et al. require full kernel integration with no mass discarded.
    for c in candidates:
        safe_min = max(c['min'], 1e-6)  # PRISM requires strictly positive lower bounds
        transitions[c['id']] = (safe_min, c['max'])
        total_inside_mass += c['prob']

    # STRICT PRIORITY LOGIC — crash and safe sink
    wall_loc = bins[0][0]

    if sigma_per_dim[0] > 0:
        p_crash = fast_norm_cdf(wall_loc, next_center[0], sigma_per_dim[0])
    else:
        p_crash = 1.0 if next_center[0] < wall_loc else 0.0

    p_safe_sink = max(0.0, 1.0 - total_inside_mass - p_crash)

    # A. Crash State (s=0)
    if p_crash > 1e-6:
        c_min = max(1e-6, p_crash - epsilon)
        c_max = min(1.0,  p_crash + epsilon)
        if 0 in transitions:
            curr_min, curr_max = transitions[0]
            transitions[0] = (min(1.0, curr_min + c_min), min(1.0, curr_max + c_max))
        else:
            transitions[0] = (c_min, c_max)

    # B. Safe Sink (s=total_states)
    if p_safe_sink > 1e-6:
        s_min = max(1e-6, p_safe_sink - epsilon)
        s_max = min(1.0,  p_safe_sink + epsilon)
        if total_states in transitions:
            curr_min, curr_max = transitions[total_states]
            transitions[total_states] = (min(1.0, curr_min + s_min), min(1.0, curr_max + s_max))
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
        chunks = [all_indices[i:i + fixed_chunk_size] for i in range(0, len(all_indices), fixed_chunk_size)]
        total_chunks = len(chunks)

        print(f"[Discretizer] Using {num_cores} cores. Mode: Per-dim sigma kernel, stochastic-dim epsilon.")

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
                    print(f"[Discretizer] Progress: {percent:.1f}% ({counter}/{total_chunks})", end='\r')

                for (src_id, act_id, transitions) in batch_results:
                    if not transitions:
                        continue
                    updates = []
                    for tgt_id, (p_min, p_max) in transitions.items():
                        updates.append(f"[{p_min:.8f}, {p_max:.8f}] : (s'={tgt_id})")
                    imdp.add_transition(src_id, act_id, " + ".join(updates))

        print("\n[Discretizer] Aggregation Complete!")
        return imdp