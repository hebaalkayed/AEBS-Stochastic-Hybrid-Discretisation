import numpy as np

GRID_PRESETS = {
    'debug':  {'res': (5.0, 2.0, 1.0), 'desc': "Fast Debug"},
    'coarse': {'res': (2.0, 1.0, 0.5), 'desc': "Initial Checks"},
    'medium': {'res': (1.0, 0.5, 0.25), 'desc': "Baseline"},
    'fine':   {'res': (0.5, 0.1, 0.1), 'desc': "High Fidelity"},
    'optimized': {'res': (1.5, 0.75, 0.5), 'desc': "Memory Safe Medium"},
    'light':  {'res': (1.5, 1.0, 1.0), 'desc': "Memory Safe Verification"}
}

class Grid:
    def __init__(self, preset='medium', x_bounds=(0, 100), v_bounds=(0, 30), a_bounds=(-10, 5)):
        if preset not in GRID_PRESETS: 
            raise ValueError(f"Unknown preset {preset}")
        
        self.resolution = GRID_PRESETS[preset]['res']
        self.bounds = [x_bounds, v_bounds, a_bounds]
        
        # Center-offset bins so integer values land in the middle
        self.bins = [
            np.arange(b[0] - r/2, b[1] + r + r/2, r) 
            for b, r in zip(self.bounds, self.resolution)
        ]
        self.shape = tuple(len(b) - 1 for b in self.bins)
        self.total_states = np.prod(self.shape)

    def state_to_index(self, state):
        """Maps continuous [gap, v, a] to grid indices (ix, iv, ia)."""
        indices = []
        for i, val in enumerate(state):
            idx = np.digitize(val, self.bins[i]) - 1
            if idx < 0 or idx >= self.shape[i]: 
                return None
            indices.append(idx)
        return tuple(indices)

    def index_to_cell_center(self, idx_tuple):
        """Returns the continuous center point of a cell."""
        center = []
        for i, idx in enumerate(idx_tuple):
            low, high = self.bins[i][idx], self.bins[i][idx+1]
            center.append((low + high) / 2.0)
        return np.array(center)

    def get_cell_bounds(self, idx_tuple):
        """Returns list of (min, max) for integration."""
        return [(self.bins[i][idx], self.bins[i][idx+1]) for i, idx in enumerate(idx_tuple)]
    
    def get_flat_index(self, idx_tuple):
        return np.ravel_multi_index(idx_tuple, self.shape)