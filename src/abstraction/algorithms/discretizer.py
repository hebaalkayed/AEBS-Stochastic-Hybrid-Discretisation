import numpy as np
import itertools
import math
from src.abstraction.types.imdp import IMDP
from src.abstraction.types.grid import Grid
from multiprocessing import Pool, cpu_count
from functools import partial

# --- HELPER: FAST MATH ---
# Standard error function for Gaussian integration
SQRT2 = 1.4142135623730951

def fast_norm_cdf(x, mean, std):
    return 0.5 * (1 + math.erf((x - mean) / (std * SQRT2)))

# --- WORKER FUNCTION ---
def process_chunk(chunk_indices, grid_bins, grid_shape, grid_total_states, 
                  system_action_space, system_wrapper, grid_resolution):
    results = []
    
    # 1. Pre-calculate Geometric Constants based on the Paper
    # Volume of the target cell (Lebesgue measure L_{<s'>})
    # Equation (6): Error scales linearly with the volume of the target set.
    cell_volume = np.prod(grid_resolution)
    
    # Radius of the cell (Euclidean distance from center to corner)
    # ||x - x_bar|| <= radius
    cell_radius = np.linalg.norm(grid_resolution / 2.0)

    # Local helper for index conversion
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
        center = index_to_cell_center(idx)
        
        for act_id, act_val in system_action_space.items():
            # A. Query Physics
            # Returns next_center, noise_std (sigma), and Lipschitz constant (L)
            next_c, sigma, L = system_wrapper.get_next_state_distribution(center, act_val)
            
            # B. Calculate Formal Error Bound (Paper Eq. 6)
            # epsilon = Volume * L * Radius
            # This is the "Lipschitz Expansion" term.
            epsilon = cell_volume * L * cell_radius
            
            # C. Compute Transition Kernel
            transitions = _compute_kernel_accurate(
                next_c, sigma, epsilon, 
                grid_bins, grid_shape, grid_total_states
            )
            
            results.append((src_id, act_id, transitions))
    return results

def _compute_kernel_accurate(next_center, sigma, epsilon, bins, grid_shape, total_states):
    """
    Computes transitions with the formal error bound applied to the probability mass.
    """
    # 1. Determine Search Window
    # We look 4 standard deviations out to capture >99.9% of mass
    radius = 4 * sigma if sigma > 0 else 0.0
    min_c = next_center - radius
    max_c = next_center + radius
    
    # 2. Identify Candidate Grid Cells
    ranges = []
    for i in range(3):
        s = np.digitize(min_c[i], bins[i]) - 1
        e = np.digitize(max_c[i], bins[i]) - 1
        # Clamp to grid boundaries
        ranges.append(range(max(0, s), min(grid_shape[i] - 1, e) + 1))

    transitions = {}
    total_prob_mass = 0.0
    
    # 3. Integrate Gaussian Kernel
    for idx in itertools.product(*ranges):
        # Boundaries of the target cell
        bounds = [(bins[i][idx[i]], bins[i][idx[i]+1]) for i in range(3)]
        prob = 1.0
        
        # Compute mass in this cell (Dimension by Dimension)
        for d in range(3):
            low, high = bounds[d]
            mean = next_center[d]
            if sigma > 0:
                p_high = fast_norm_cdf(high, mean, sigma)
                p_low = fast_norm_cdf(low, mean, sigma)
                prob *= (p_high - p_low)
            else:
                # Deterministic case (Dirac delta)
                prob *= (1.0 if low <= mean <= high else 0.0)
        
        # 4. Apply Formal Error & Pruning
        # We prune only if the *upper bound* is negligible to save space.
        # Paper Definition 7: Interval is [max(0, p - e), min(1, p + e)]
        p_min = max(0.0, prob - epsilon)
        p_max = min(1.0, prob + epsilon)
        
        if p_max > 1e-5: # Keep if there's any chance of transition
            flat = 0
            mult = 1
            for i in reversed(range(3)):
                flat += idx[i] * mult
                mult *= grid_shape[i]
            
            transitions[flat] = (p_min, p_max)
            total_prob_mass += prob

    # 5. Sink State Logic
    # Any probability mass not captured by the grid (out of bounds) goes to Sink.
    # We apply the error bound to the sink transition as well.
    sink_prob = max(0.0, 1.0 - total_prob_mass)
    if sink_prob > 1e-5:
        # The error for the sink is the accumulation of errors? 
        # For simplicity, we treat the sink as a single "rest of world" cell.
        # Conservative approach: Sink inherits the same epsilon uncertainty.
        s_min = max(0.0, sink_prob - epsilon)
        s_max = min(1.0, sink_prob + epsilon)
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
        
        # Prepare parameters for workers
        all_indices = list(itertools.product(*[range(d) for d in self.grid.shape]))
        num_cores = max(1, cpu_count() - 1)
        
        fixed_chunk_size = 1000 
        chunks = [all_indices[i:i + fixed_chunk_size] for i in range(0, len(all_indices), fixed_chunk_size)]
        total_chunks = len(chunks)
        
        print(f"[Discretizer] Using {num_cores} cores. Mode: Formal Lipschitz Bound (Eq. 6).")

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
                        # Format as interval for PRISM: [min, max]
                        updates.append(f"[{p_min:.5f}, {p_max:.5f}] : (s'={tgt_id})")
                    
                    distribution_str = " + ".join(updates)
                    imdp.add_transition(src_id, act_id, distribution_str)

        print("\n[Discretizer] Aggregation Complete!")
        return imdp