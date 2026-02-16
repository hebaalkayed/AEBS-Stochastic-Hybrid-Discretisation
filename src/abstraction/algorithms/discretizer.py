import numpy as np
import itertools
import math
from src.abstraction.types.imdp import IMDP
from src.abstraction.types.grid import Grid
from multiprocessing import Pool, cpu_count
from functools import partial

# --- HELPER: FAST MATH ---
SQRT2 = 1.4142135623730951

def fast_norm_cdf(x, mean, std):
    return 0.5 * (1 + math.erf((x - mean) / (std * SQRT2)))

# --- WORKER FUNCTION ---
def process_chunk(chunk_indices, grid_bins, grid_shape, grid_total_states, 
                  system_action_space, system_wrapper, grid_resolution):
    results = []
    
    # Pre-calculate Geometric Constants
    cell_volume = np.prod(grid_resolution)
    cell_radius = np.linalg.norm(grid_resolution / 2.0)

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

        # --- FIX: MAKE CRASH ABSORBING ---
        if src_id == 0:
            for act_id in system_action_space.keys():
                results.append((src_id, act_id, {0: (1.0, 1.0)}))
            continue
        # ---------------------------------

        center = index_to_cell_center(idx)
        
        for act_id, act_val in system_action_space.items():
            # A. Query Physics
            next_c, sigma, L = system_wrapper.get_next_state_distribution(center, act_val)
            
            # B. Calculate Formal Error Bound
            epsilon = cell_volume * L * cell_radius
            
            # C. Compute Transition Kernel (Strict Crash Priority)
            transitions = _compute_kernel_accurate(
                next_c, sigma, epsilon, 
                grid_bins, grid_shape, grid_total_states
            )
            
            results.append((src_id, act_id, transitions))
    return results

def _compute_kernel_accurate(next_center, sigma, epsilon, bins, grid_shape, total_states):
    """
    Computes transitions with STRICT CRASH PRIORITY and PRISM-safe lower bounds.
    """
    radius = 4 * sigma if sigma > 0 else 0.0
    min_c = next_center - radius
    max_c = next_center + radius
    
    ranges = []
    for i in range(3):
        s = np.digitize(min_c[i], bins[i]) - 1
        e = np.digitize(max_c[i], bins[i]) - 1
        ranges.append(range(max(0, s), min(grid_shape[i] - 1, e) + 1))

    transitions = {}
    total_inside_mass = 0.0
    
    candidates = []

    for idx in itertools.product(*ranges):
        bounds = [(bins[i][idx[i]], bins[i][idx[i]+1]) for i in range(3)]
        prob = 1.0
        
        for d in range(3):
            low, high = bounds[d]
            mean = next_center[d]
            if sigma > 0:
                p_high = fast_norm_cdf(high, mean, sigma)
                p_low = fast_norm_cdf(low, mean, sigma)
                prob *= (p_high - p_low)
            else:
                prob *= (1.0 if low <= mean <= high else 0.0)
        
        p_min = max(0.0, prob - epsilon)
        p_max = min(1.0, prob + epsilon)
        
        if p_max > 0:
            flat = 0
            mult = 1
            for i in reversed(range(3)):
                flat += idx[i] * mult
                mult *= grid_shape[i]
            
            candidates.append({
                'id': flat, 
                'min': p_min, 
                'max': p_max, 
                'prob': prob
            })

    # SORT AND PRUNE (Top-3)
    candidates.sort(key=lambda x: x['prob'], reverse=True)
    top_k = candidates[:3]
    
    for c in top_k:
        safe_min = max(c['min'], 1e-6) # PRISM safety
        transitions[c['id']] = (safe_min, c['max'])
        total_inside_mass += c['prob']

    # STRICT PRIORITY LOGIC
    wall_loc = bins[0][0]
    
    if sigma > 0:
        p_crash = fast_norm_cdf(wall_loc, next_center[0], sigma)
    else:
        p_crash = 1.0 if next_center[0] < wall_loc else 0.0

    p_safe_sink = max(0.0, 1.0 - total_inside_mass - p_crash)
    
    # A. Crash State (State 0)
    if p_crash > 1e-6:
        # PRISM FIX: Floor at 1e-6 instead of 0.0
        c_min = max(1e-6, p_crash - epsilon)
        c_max = min(1.0, p_crash + epsilon)
        
        if 0 in transitions:
            curr_min, curr_max = transitions[0]
            transitions[0] = (min(1.0, curr_min + c_min), min(1.0, curr_max + c_max))
        else:
            transitions[0] = (c_min, c_max)

    # B. Safe Sink (State total_states)
    if p_safe_sink > 1e-6:
        # PRISM FIX: Floor at 1e-6 instead of 0.0
        s_min = max(1e-6, p_safe_sink - epsilon)
        s_max = min(1.0, p_safe_sink + epsilon)
        
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
        
        print(f"[Discretizer] Using {num_cores} cores. Mode: Strict Crash Priority + Absorbing Crash.")

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