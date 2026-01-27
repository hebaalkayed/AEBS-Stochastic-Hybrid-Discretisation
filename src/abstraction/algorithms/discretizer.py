import numpy as np
import itertools
from scipy.stats import norm
from src.abstraction.types.imdp import IMDP
from src.abstraction.types.grid import Grid
from src.abstraction.types.interfaces import StochasticHybridSystem

class DiscretizationAlgorithm:
    def __init__(self, grid: Grid):
        self.grid = grid

    def run(self, system: StochasticHybridSystem, name: str) -> IMDP:
        """
        The Core Pipeline: 'Give me a dt-SHS, I give you a Finite IMDP'.
        """
        print(f"[Discretizer] Processing '{name}' over {self.grid.total_states} states...")
        
        imdp = IMDP(name=name, num_states=self.grid.total_states)
        actions = system.get_action_space()
        max_cell_size = np.max(self.grid.resolution)
        
        iterator = itertools.product(*[range(d) for d in self.grid.shape])
        count = 0
        
        for idx in iterator:
            src_id = self.grid.get_flat_index(idx)
            center = self.grid.index_to_cell_center(idx)
            
            for act_id, act_val in actions.items():
                # 1. Query Physics (Where do we go?)
                next_c, sigma, L = system.get_next_state_distribution(center, act_val)
                
                # 2. Compute Kernel (Integration + Error)
                transitions = self._compute_kernel(next_c, sigma, L, max_cell_size)
                
                # 3. Store in Model
                for tgt, (p_min, p_max) in transitions.items():
                    imdp.add_transition(src_id, act_id, tgt, p_min, p_max)
            
            count += 1
            if count % 10000 == 0:
                print(f"    ... processed {count}/{self.grid.total_states}")
                    
        return imdp

    def _compute_kernel(self, next_center, sigma, L, max_cell_size):
        """Computes T(s,a,s') = Integral +/- Error."""
        search_radius = 4 * sigma if sigma > 0 else 0.0
        min_c = next_center - search_radius
        max_c = next_center + search_radius
        
        start = self.grid.state_to_index(min_c)
        end = self.grid.state_to_index(max_c)
        
        if start is None or end is None: 
            return {} # Strictly out of bounds
        
        transitions = {}
        error = L * (max_cell_size / 2.0)
        
        ranges = [range(start[i], end[i]+1) for i in range(3)]
        
        for tgt_idx in itertools.product(*ranges):
            # Clip bounds
            if any(x < 0 or x >= self.grid.shape[i] for i, x in enumerate(tgt_idx)): 
                continue
            
            bounds = self.grid.get_cell_bounds(tgt_idx)
            prob = 1.0
            
            # Compute Gaussian Integral per dimension
            for d in range(3):
                low, high = bounds[d]
                mean = next_center[d]
                if sigma > 0:
                    prob *= (norm.cdf(high, mean, sigma) - norm.cdf(low, mean, sigma))
                else:
                    prob *= (1.0 if low <= mean <= high else 0.0)
            
            if prob > 1e-6:
                transitions[self.grid.get_flat_index(tgt_idx)] = (prob - error, prob + error)
                
        return transitions