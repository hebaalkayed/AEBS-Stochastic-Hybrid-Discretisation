import numpy as np
import itertools
import math
from src.abstraction.types.imdp import IMDP
from src.abstraction.types.grid import Grid
from multiprocessing import Pool, cpu_count
from functools import partial

# --- HELPER: FAST MATH (The Speed Fix) ---
# Scipy is too slow for 1 million calls. We use raw C-based math.erf.
SQRT2 = 1.4142135623730951

def fast_norm_cdf(x, mean, std):
    """
    A 20x faster implementation of Cumulative Density Function 
    using the standard Error Function (erf) from Python's math library.
    """
    return 0.5 * (1 + math.erf((x - mean) / (std * SQRT2)))

# --- WORKER FUNCTION ---
def process_chunk(chunk_indices, grid_bins, grid_shape, grid_total_states, 
                  system_action_space, system_wrapper, max_cell_size):
    results = []
    
    # Local helper for speed (re-implemented to avoid pickling overhead)
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
            # 1. Query Physics
            next_c, sigma, L = system_wrapper.get_next_state_distribution(center, act_val)
            
            # 2. Compute Kernel
            transitions = _compute_kernel_static(next_c, sigma, L, max_cell_size, 
                                               grid_bins, grid_shape, grid_total_states)
            results.append((src_id, act_id, transitions))
    return results

def _compute_kernel_static(next_center, sigma, L, max_cell_size, bins, grid_shape, total_states):
    # 1. Determine Search Window
    radius = 4 * sigma if sigma > 0 else 0.0
    min_c = next_center - radius
    max_c = next_center + radius
    
    # 2. Identify Candidate Grid Cells & CLAMP
    ranges = []
    for i in range(3):
        # We use np.digitize.
        s = np.digitize(min_c[i], bins[i]) - 1
        e = np.digitize(max_c[i], bins[i]) - 1
        ranges.append(range(max(0, s), min(grid_shape[i] - 1, e) + 1))

    transitions = {}
    total_prob_mass = 0.0
    error = L * (max_cell_size / 2.0)
    
    # 3. Integrate
    for idx in itertools.product(*ranges):
        # Local bounds extraction
        bounds = [(bins[i][idx[i]], bins[i][idx[i]+1]) for i in range(3)]
        prob = 1.0
        
        for d in range(3):
            low, high = bounds[d]
            mean = next_center[d]
            if sigma > 0:
                # USE FAST MATH HERE
                p_high = fast_norm_cdf(high, mean, sigma)
                p_low = fast_norm_cdf(low, mean, sigma)
                prob *= (p_high - p_low)
            else:
                prob *= (1.0 if low <= mean <= high else 0.0)
        
        # --- CRITICAL FIX: AGGRESSIVE PRUNING ---
        # Changed from 1e-12 to 1e-4. 
        # This reduces file size from 3GB to ~150MB by ignoring "0.001%" probabilities.
        if prob > 1e-4:
            flat = 0
            mult = 1
            for i in reversed(range(3)):
                flat += idx[i] * mult
                mult *= grid_shape[i]
            
            transitions[flat] = (max(0.0, prob - error), prob + error)
            total_prob_mass += prob

    # 4. Sink State
    sink_prob = 1.0 - total_prob_mass
    if sink_prob > 1e-9:
        transitions[total_states] = (max(0.0, sink_prob - error), sink_prob + error)
        
    return transitions

# --- MAIN CLASS ---
class DiscretizationAlgorithm:
    def __init__(self, grid: Grid):
        self.grid = grid

    def run(self, system, name: str) -> IMDP:
        print(f"[Discretizer] Parallel Processing '{name}' over {self.grid.total_states} states...")
        
        imdp = IMDP(name=name, num_states=self.grid.total_states + 1)
        actions = system.get_action_space()
        max_cell_size = np.max(self.grid.resolution)
        
        # 1. Prepare Chunks
        all_indices = list(itertools.product(*[range(d) for d in self.grid.shape]))
        num_cores = max(1, cpu_count() - 1)
        
        fixed_chunk_size = 1000  # Process 1000 states per batch
        chunks = [all_indices[i:i + fixed_chunk_size] for i in range(0, len(all_indices), fixed_chunk_size)]
        total_chunks = len(chunks)
        
        print(f"[Discretizer] Using {num_cores} cores. Turbo Mode: ON. Pruning: 1e-4.")

        # 2. Run Parallel Pool
        with Pool(processes=num_cores) as pool:
            func = partial(process_chunk, 
                           grid_bins=self.grid.bins,
                           grid_shape=self.grid.shape,
                           grid_total_states=self.grid.total_states,
                           system_action_space=actions, 
                           system_wrapper=system, 
                           max_cell_size=max_cell_size)
            
            counter = 0
            for batch_results in pool.imap_unordered(func, chunks):
                counter += 1
                if counter % 10 == 0 or counter == total_chunks:
                    percent = (counter / total_chunks) * 100
                    print(f"[Discretizer] Progress: {percent:.1f}% ({counter}/{total_chunks})", end='\r')

                for (src_id, act_id, transitions) in batch_results:
                    for tgt_id, (p_min, p_max) in transitions.items():
                        imdp.add_transition(src_id, act_id, tgt_id, p_min, p_max)

        print("\n[Discretizer] Aggregation Complete!")
        return imdp