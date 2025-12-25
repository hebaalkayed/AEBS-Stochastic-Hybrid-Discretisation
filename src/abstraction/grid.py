import numpy as np

# ==============================================================================
#  GRID CONFIGURATIONS (PRESETS)
#  Use these to compare different levels of abstraction granularity.
# ==============================================================================
GRID_PRESETS = {
    'debug': {
        'resolution': (5.0, 2.0, 1.0),  # Very coarse, minimal states
        'description': "Super fast generation, physics will be very rough."
    },
    'coarse': {
        'resolution': (2.0, 1.0, 0.5),  # Low resolution
        'description': "Good for initial connectivity checks."
    },
    'medium': {
        'resolution': (1.0, 0.5, 0.25), # Standard Thesis Baseline
        'description': "Balanced trade-off between accuracy and state space size."
    },
    'fine': {
        'resolution': (0.5, 0.1, 0.1),  # High Fidelity
        'description': "High precision, captures subtle physics (approx. 50k+ states)."
    }
}

class Grid:
    def __init__(self, preset='medium', custom_resolution=None, x_bounds=(0, 100), v_bounds=(0, 30), a_bounds=(-10, 5)):
        """
        Defines the discrete partition of the state space.
        
        Args:
            preset (str): Name of the configuration to use ('debug', 'coarse', 'medium', 'fine').
            custom_resolution (tuple): Optional override (x_res, v_res, a_res).
            x_bounds, v_bounds, a_bounds: Min/Max limits for the world.
        """
        # 1. Load Resolution from Preset or Custom Override
        if custom_resolution:
            self.resolution = custom_resolution
            self.description = "Custom User Resolution"
        else:
            if preset not in GRID_PRESETS:
                raise ValueError(f"Unknown preset '{preset}'. Available: {list(GRID_PRESETS.keys())}")
            
            config = GRID_PRESETS[preset]
            self.resolution = config['resolution']
            self.description = config['description']

        self.bounds = [x_bounds, v_bounds, a_bounds]
        
        # 2. Create Bins with Center-Offset Logic
        # We offset by -res/2 so that integer values (e.g., v=20.0) land in the CENTER of a cell.
        self.bins = [
            np.arange(b[0] - r/2, b[1] + r + r/2, r) 
            for b, r in zip(self.bounds, self.resolution)
        ]
        
        # 3. Calculate Properties
        self.shape = tuple(len(b) - 1 for b in self.bins)
        self.total_states = np.prod(self.shape)
        
    def __repr__(self):
        return f"<Grid: {self.description} | Res: {self.resolution} | States: {self.total_states}>"

    def state_to_index(self, continuous_state):
        """
        Maps [x, v, a] -> (ix, iv, ia). Returns None if out of bounds.
        """
        indices = []
        for i, val in enumerate(continuous_state):
            idx = np.digitize(val, self.bins[i]) - 1
            if idx < 0 or idx >= self.shape[i]:
                return None
            indices.append(idx)
        return tuple(indices)

    def index_to_cell_center(self, index_tuple):
        """Returns the continuous center point of a cell."""
        center = []
        for i, idx in enumerate(index_tuple):
            low = self.bins[i][idx]
            high = self.bins[i][idx+1]
            center.append((low + high) / 2.0)
        return np.array(center)

    def get_cell_bounds(self, index_tuple):
        """Returns list of (min, max) for integration."""
        bounds = []
        for i, idx in enumerate(index_tuple):
            low = self.bins[i][idx]
            high = self.bins[i][idx+1]
            bounds.append((low, high))
        return bounds
    
    def get_flat_index(self, index_tuple):
        """(ix, iv, ia) -> int ID"""
        return np.ravel_multi_index(index_tuple, self.shape)
    
    def get_tuple_index(self, flat_index):
        """int ID -> (ix, iv, ia)"""
        return np.unravel_index(flat_index, self.shape)