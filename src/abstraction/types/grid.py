import numpy as np

from src.abstraction.types.cell_topology import (
    cell_of, flat_index, make_edges, edges_hash, n_cells_for,
)

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
    'micro_fine': {'res': (1.0, 1.0, 1.0), 'desc': "Micro-world fine (714 states, boundary resolution)"},
    
    'extra_fine': {'res': (0.1, 0.1, 0.1), 'desc': "Following window (control-visible ego, sigma/cell=2)"}
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
        # make_edges produces values bit-identical to the legacy
        # np.arange(low - r/2, high + r + r/2, r) but fixes the edge COUNT by
        # integer arithmetic, so it cannot drift by one across platforms
        # (laptop vs cluster); see cell_topology.make_edges.
        self.bins = []
        for (low, high), r in zip(self.bounds, self.resolution):
            edges = make_edges(low, high, r)
            assert len(edges) == n_cells_for(low, high, r) + 1, (
                f"edge count {len(edges)} inconsistent with n_cells_for "
                f"for bounds ({low}, {high}), resolution {r}")
            self.bins.append(edges)

        # 4. Calculate Shape
        self.shape = tuple(len(b) - 1 for b in self.bins)
        self.total_states = np.prod(self.shape)

        # Fingerprint of the exact edge floats. Exact-edge behaviour is a
        # property of specific float values, so a generation run (cluster) and
        # a containment run (laptop) certify the same partition iff their
        # hashes match. Record this in artifact metadata.
        self.bins_hash = edges_hash(self.bins)

        # Sanity print so mismatch is immediately visible
        print(f"[Grid] preset={preset} | resolution={self.resolution}")
        print(f"[Grid] bounds: gap={x_bounds}, v_ego={v_bounds}, v_lead={vl_bounds}")
        print(f"[Grid] shape={self.shape} | total_states={self.total_states}")
        print(f"[Grid] bins_hash={self.bins_hash[:16]}... (partition fingerprint)")

    def state_to_index(self, state):
        """Maps continuous [gap, v_ego, v_lead] to grid indices (ix, iv, ivl).

        Half-open assignment via cell_topology.cell_of: a value on a shared
        edge belongs to the cell above; a value on the top edge is outside.
        """
        indices = []
        for i, val in enumerate(state):
            idx = cell_of(val, self.bins[i])
            if idx is None:
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
        # Same row-major layout as np.ravel_multi_index, but through the
        # single shared definition (also used by the discretizer and the
        # containment test). Returns a plain int.
        return flat_index(idx_tuple[0], idx_tuple[1], idx_tuple[2], self.shape)