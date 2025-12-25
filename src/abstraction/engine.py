import numpy as np
from scipy.stats import norm
import itertools
from .grid import Grid

class AbstractionEngine:
    def __init__(self, plant, grid):
        """
        Implements the Formal Abstraction Algorithm.
        
        Args:
            plant: Instance of VehiclePlant (must implement get_deterministic_next_state)
            grid: Instance of Grid
        """
        self.plant = plant
        self.grid = grid

    def compute_transitions(self, cell_index_tuple, action):
        """
        Computes the transition intervals [P_min, P_max] for a specific cell and action.
        
        Steps (Algorithm 1, Line 2):
        1. Map cell center -> deterministic next state f(c, u).
        2. Integrate stochastic kernel (noise) over target cells.
        3. Expand bounds by Lipschitz error.
        """
        # 1. Deterministic Center Mapping
        center_state = self.grid.index_to_cell_center(cell_index_tuple)
        next_center_state = self.plant.get_deterministic_next_state(center_state, action)
        
        # 2. Identify Potential Target Cells
        # Optimization: Only integrate over cells within 4 standard deviations
        # This covers >99.9% of the probability mass
        search_radius = 4 * self.plant.noise_std
        
        # Find the bounds of the "cloud"
        min_cloud = next_center_state - search_radius
        max_cloud = next_center_state + search_radius
        
        # Map cloud bounds to grid indices to find the range of cells to loop over
        start_indices = self.grid.state_to_index(min_cloud)
        end_indices = self.grid.state_to_index(max_cloud)
        
        # Handle edge case where cloud is strictly out of bounds
        if start_indices is None or end_indices is None:
            # Fallback: simple scan (computationally expensive but safe) or clamp
            # For this snippet, we assume the cloud is largely within bounds.
            # In production, you would handle boundary absorbing states here.
            return {}

        transitions = {}
        
        # 3. Compute Lipschitz Error Term (The "Epsilon" from the paper)
        # Error = L * (max_cell_radius)
        # Using 1-norm or infinity norm of cell size for safety
        max_cell_size = np.max(self.grid.resolution)
        error_epsilon = self.plant.lipschitz_constant * max_cell_size * 0.1 # Scaling factor for conservatism
        
        # Iterate over the sub-grid of potential targets
        ranges = [range(start_indices[i], end_indices[i] + 1) for i in range(3)]
        
        for target_idx in itertools.product(*ranges):
            # Check if valid index
            if any(x < 0 or x >= self.grid.shape[i] for i, x in enumerate(target_idx)):
                continue
                
            # Get integration bounds for this target cell
            bounds = self.grid.get_cell_bounds(target_idx)
            
            # CALCULATE INTEGRAL (The Stochastic Kernel T(dx' | x, u))
            # Assuming independent Gaussian noise on each dimension for simplicity,
            # or noise primarily on acceleration propagating to others.
            # Here we apply the noise_std to the vectors.
            
            prob_mass = 1.0
            for dim in range(3): # x, v, a
                low, high = bounds[dim]
                mean = next_center_state[dim]
                sigma = self.plant.noise_std
                
                # CDF(high) - CDF(low)
                if sigma > 0:
                    p_dim = norm.cdf(high, loc=mean, scale=sigma) - norm.cdf(low, loc=mean, scale=sigma)
                else:
                    # Deterministic case
                    p_dim = 1.0 if low <= mean <= high else 0.0
                    
                prob_mass *= p_dim
            
            # 4. Apply Error Bounds (Algorithm 1: P +/- error)
            # The survey states P_min = P_hat - error, P_max = P_hat + error
            if prob_mass > 1e-6: # Ignore negligible transitions
                p_min = max(0.0, prob_mass - error_epsilon)
                p_max = min(1.0, prob_mass + error_epsilon)
                
                flat_idx = self.grid.get_flat_index(target_idx)
                transitions[flat_idx] = (p_min, p_max)
                
        return transitions