import numpy as np
import itertools

class GridWorld:
    """
    Defines the discrete state space for the verification.
    Based on Algorithm 1: State Space Partitioning.
    """
    def __init__(self):
        # --- 1. DEFINE BOUNDS ---
        # We focus on the "danger zone" (0 to 100m)
        self.d_min, self.d_max = 0.0, 100.0
        self.v_min, self.v_max = 0.0, 40.0   # 0 to 144 km/h
        
        # --- 2. DEFINE RESOLUTION (The "Cell Size") ---
        # Coarse grid for initial testing (make these smaller for final results)
        self.d_step = 5.0   # Every 5 meters
        self.v_step = 2.0   # Every 2 m/s
        
        # --- 3. GENERATE POINTS ---
        self.d_grid = np.arange(self.d_min, self.d_max + self.d_step, self.d_step)
        self.ve_grid = np.arange(self.v_min, self.v_max + self.v_step, self.v_step)
        self.vl_grid = np.arange(self.v_min, self.v_max + self.v_step, self.v_step)
        
        # Pre-calculate shape for easier indexing
        self.shape = (len(self.d_grid), len(self.ve_grid), len(self.vl_grid))
        self.n_states = np.prod(self.shape)
        
        print(f"Grid Initialized: {self.shape} -> {self.n_states} total discrete states.")

    def state_to_index(self, d, ve, vl):
        """
        Maps continuous values (12.5m) to a grid index (Cell #2).
        """
        # Clamp values to stay inside grid
        d = np.clip(d, self.d_min, self.d_max)
        ve = np.clip(ve, self.v_min, self.v_max)
        vl = np.clip(vl, self.v_min, self.v_max)
        
        # Find nearest index
        idx_d = int(round((d - self.d_min) / self.d_step))
        idx_ve = int(round((ve - self.v_min) / self.v_step))
        idx_vl = int(round((vl - self.v_min) / self.v_step))
        
        # Ensure we don't go out of bounds (due to rounding)
        idx_d = min(idx_d, len(self.d_grid) - 1)
        idx_ve = min(idx_ve, len(self.ve_grid) - 1)
        idx_vl = min(idx_vl, len(self.vl_grid) - 1)
        
        return (idx_d, idx_ve, idx_vl)

    def index_to_state(self, idx_tuple):
        """
        Returns the 'Center Point' of a grid cell.
        Used to simulate transitions from this cell.
        """
        d = self.d_grid[idx_tuple[0]]
        ve = self.ve_grid[idx_tuple[1]]
        vl = self.vl_grid[idx_tuple[2]]
        return np.array([d, ve, vl])

    def get_all_states(self):
        """
        Iterator to loop through every single cell in the grid.
        """
        ranges = [range(s) for s in self.shape]
        return itertools.product(*ranges)