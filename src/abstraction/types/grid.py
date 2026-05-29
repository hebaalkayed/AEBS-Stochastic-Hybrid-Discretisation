import numpy as np

GRID_PRESETS = {
    'debug':  {'res': (5.0, 2.0, 1.0), 'desc': "Fast Debug"},
    'coarse': {'res': (2.0, 1.0, 0.5), 'desc': "Initial Checks"},
                                                                  # res = (2.0, 1.0, 0.5) — v_lead cells are 0.5 m/s
                                                                  # sigma/cell_width = 0.2/0.5 = 0.4 — meaningful ratio

    'medium': {'res': (1.0, 0.5, 0.25), 'desc': "Baseline (epsilon < 1)"},
    'fast_medium': {'res': (1.0, 0.2, 0.25), 'desc': "Fast Medium"},
    'medium_tight': {'res': (1.0, 0.5, 0.1), 'desc': "Tight Pmin (sigma/cell=2.0)"},
    
    
    'fine':   {'res': (0.5, 0.1, 0.1), 'desc': "High Fidelity"},
    'optimized': {'res': (1.5, 0.75, 0.5), 'desc': "Memory Safe Medium"},
    'light':  {'res': (1.5, 1.0, 1.0), 'desc': "Memory Safe Verification"},
    'toy':    {'res': (10.0, 10.0, 2.0), 'desc': "Toy POC (16 states, hand-traceable)"},
    'toy_v2': {'res': (5.0, 5.0, 2.0), 'desc': "Toy POC v2 (56 states, actions distinguishable)"},
    'micro':  {'res': (2.0, 2.0, 1.0), 'desc': "Micro-world POC (72 states, full fidelity)"},
    # 'micro_fine': {'res': (1.0, 2.0, 1.0), 'desc': "Micro-world fine gap (408 states, shows interval divergence)"}
    'micro_fine': {'res': (1.0, 1.0, 1.0), 'desc': "Micro-world fine (714 states, boundary resolution)"}
}

class Grid:
    """
    3D grid over (gap [m], v_ego [m/s], v_lead [m/s]).

    BUG FIX: The third dimension is v_lead, NOT acceleration.
    The old default was a_bounds=(-15, 10), which put v_lead cells
    over a physically wrong range. States with v_lead > 10 m/s would
    silently route to safe_sink.

    Correct default: vl_bounds=(-1, 31).
        - Lower margin (-1): gives one cell of headroom below v_lead=0 so
          the Gaussian tail at v_lead=0 doesn't immediately escape the grid.
          With cell width 0.5 and sigma=0.2, Phi(-0.25/0.2) ≈ 10.6% would
          escape each step without this margin.
        - Upper bound (31): covers the full physical range [0, 30] with
          one cell of headroom above.

    State layout: flat index = ix * (nv * nvl) + iv * nvl + ivl
    """

    def __init__(self, preset='medium',
                 x_bounds=(0, 100),
                 v_bounds=(0, 30),
                 vl_bounds=(-1, 31)):          # BUG FIX: was a_bounds=(-15, 10)
        if preset not in GRID_PRESETS:
            raise ValueError(f"Unknown preset {preset}")

        # 1. Load Resolution from Preset
        res_tuple = GRID_PRESETS[preset]['res']

        # 2. Store Parameters
        self.resolution = res_tuple
        self.bounds = [x_bounds, v_bounds, vl_bounds]   # BUG FIX: dim 2 is v_lead

        # 3. Create Bins (Cell Edges)
        # We offset by r/2 so that integer values (like 0.0, 1.0) land in
        # the centre of a cell rather than on a boundary.
        self.bins = []
        for (low, high), r in zip(self.bounds, self.resolution):
            edges = np.arange(low - r/2, high + r + r/2, r)
            self.bins.append(edges)

        # 4. Calculate Shape
        self.shape = tuple(len(b) - 1 for b in self.bins)
        self.total_states = np.prod(self.shape)

        # Sanity print so mismatch is immediately visible
        print(f"[Grid] preset={preset} | resolution={self.resolution}")
        print(f"[Grid] bounds: gap={x_bounds}, v_ego={v_bounds}, v_lead={vl_bounds}")
        print(f"[Grid] shape={self.shape} | total_states={self.total_states}")

    def state_to_index(self, state):
        """Maps continuous [gap, v_ego, v_lead] to grid indices (ix, iv, ivl)."""
        indices = []
        for i, val in enumerate(state):
            idx = np.digitize(val, self.bins[i]) - 1
            if idx < 0 or idx >= self.shape[i]:
                return None
            indices.append(idx)
        return tuple(indices)

    def index_to_cell_center(self, idx_tuple):
        """Returns the continuous centre point of a cell."""
        center = []
        for i, idx in enumerate(idx_tuple):
            low, high = self.bins[i][idx], self.bins[i][idx+1]
            center.append((low + high) / 2.0)
        return np.array(center)

    def get_cell_bounds(self, idx_tuple):
        """Returns list of (min, max) per dimension."""
        return [(self.bins[i][idx], self.bins[i][idx+1]) for i, idx in enumerate(idx_tuple)]

    def get_flat_index(self, idx_tuple):
        return np.ravel_multi_index(idx_tuple, self.shape)