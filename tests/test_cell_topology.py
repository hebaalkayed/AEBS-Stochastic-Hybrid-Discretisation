"""Contract test for src.abstraction.types.cell_topology.

Pins the half-open [lo, hi) boundary convention. If any of these assertions
fails, someone has changed the convention's semantics; the discretizer, the
Grid, and the containment test all import these functions, so a change here
changes the meaning of the abstraction and invalidates any existing
containment certificate.

Run:  python -m tests.test_cell_topology
"""
import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np

from src.abstraction.types.cell_topology import (
    BOUNDARY_EPS, cell_of, certainly_within, escapes_top, fully_outside_top,
    escapes_bottom, fully_outside_bottom, cells_touched, flat_index,
    make_edges, edges_hash,
)

RNG = np.random.default_rng(20260706)


def test_cell_of_convention():
    edges = np.array([-0.5, 0.5, 1.5, 2.5])
    # interior points
    assert cell_of(0.0, edges) == 0
    assert cell_of(1.0, edges) == 1
    # a value EXACTLY on an interior edge belongs to the cell ABOVE
    assert cell_of(0.5, edges) == 1
    assert cell_of(1.5, edges) == 2
    # the bottom edge is INSIDE cell 0; the top edge is OUTSIDE the grid
    assert cell_of(-0.5, edges) == 0
    assert cell_of(2.5, edges) is None
    # clearly outside
    assert cell_of(-0.6, edges) is None
    assert cell_of(3.0, edges) is None


def test_cell_of_random_roundtrip():
    edges = make_edges(0.0, 100.0, 1.0)
    n = len(edges) - 1
    # random interior values plus every exact edge value
    values = np.concatenate([RNG.uniform(edges[0], edges[-1], 5000), edges])
    for v in values:
        k = cell_of(v, edges)
        if v >= edges[-1] or v < edges[0]:
            assert k is None
        else:
            assert k is not None
            assert edges[k] <= v < edges[k + 1], (v, k)


def test_certainly_within_strictness():
    # the case behind the original soundness bug: image ending EXACTLY on the
    # cell's upper boundary is NOT certain coverage
    assert certainly_within(1.0, 1.5, 0.5, 1.5) is False
    # strictly inside is certain
    assert certainly_within(1.0, 1.4999999, 0.5, 1.5) is True
    # the LOWER boundary is owned by the cell, so an image starting exactly
    # on it is still certainly within
    assert certainly_within(0.5, 1.0, 0.5, 1.5) is True
    # straddling below is not
    assert certainly_within(0.4999999, 1.0, 0.5, 1.5) is False


def test_escape_predicates():
    edges = np.array([-0.5, 0.5, 1.5, 2.5])
    # the top edge itself is outside: touching it escapes
    assert escapes_top(2.5, edges) is True
    assert escapes_top(2.4999999, edges) is False
    assert fully_outside_top(2.5, edges) is True
    assert fully_outside_top(2.4999999, edges) is False
    # the bottom edge itself is inside cell 0: touching it does NOT escape
    assert escapes_bottom(-0.5, edges) is False
    assert escapes_bottom(-0.5000001, edges) is True
    assert fully_outside_bottom(-0.5000001, edges) is True
    assert fully_outside_bottom(-0.5, edges) is False


def test_cells_touched_eps_expansion():
    edges = np.array([-0.5, 0.5, 1.5, 2.5, 3.5])
    # image ending exactly on an interior edge: the cell above is enumerated
    # (it receives factor [0,1]; the exact-edge point belongs to it)
    assert cells_touched(1.0, 1.5, edges) == (1, 2)
    # image starting exactly on an interior edge: EPS expansion also
    # enumerates the cell below (harmless, factor [0,1]); pinned behaviour
    assert cells_touched(1.5, 2.0, edges) == (1, 2)
    # strictly interior image
    assert cells_touched(0.6, 1.4, edges) == (1, 1)
    # clipping at the grid limits
    assert cells_touched(-5.0, 50.0, edges) == (0, 3)
    # padded (stochastic window) enumeration
    assert cells_touched(1.0, 1.0, edges, pad=1.0) == (0, 2)


def test_flat_index_matches_numpy_and_is_bijective():
    for _ in range(200):
        shape = tuple(int(x) for x in RNG.integers(2, 30, size=3))
        idx = tuple(int(RNG.integers(0, s)) for s in shape)
        assert flat_index(idx[0], idx[1], idx[2], shape) == \
            int(np.ravel_multi_index(idx, shape))
    # bijectivity on a small full grid (why interval 'merging' cannot occur)
    shape = (4, 3, 5)
    seen = {flat_index(a, b, c, shape)
            for a in range(4) for b in range(3) for c in range(5)}
    assert len(seen) == 4 * 3 * 5
    assert min(seen) == 0 and max(seen) == 4 * 3 * 5 - 1


def test_make_edges_count_matches_legacy_and_values_are_accurate():
    """Count semantics must match the legacy arange (including non-divisible
    bounds, where the grid over-covers rather than truncates); values must
    stay within a tight ulp bound of the exact edge value, unlike legacy
    arange whose accumulation drifts up to ~250 ulps; construction must be
    bitwise deterministic."""
    from fractions import Fraction
    from src.abstraction.types.grid import GRID_PRESETS
    bounds_sets = [
        [(0, 100), (0, 30), (-1, 31)],      # production
        [(0, 16), (0, 6), (-2, 3)],         # POC micro-world
    ]
    worst_new = 0.0
    checked = 0
    for bounds in bounds_sets:
        for preset, spec in GRID_PRESETS.items():
            for (low, high), r in zip(bounds, spec['res']):
                legacy = np.arange(low - r / 2, high + r + r / 2, r)
                new = make_edges(low, high, r)
                assert len(new) == len(legacy), (preset, low, high, r)
                # bitwise determinism of repeated construction
                assert new.tobytes() == make_edges(low, high, r).tobytes()
                # accuracy against the exact rational edge values
                F, R = Fraction(low - r / 2), Fraction(r)
                for k in range(len(new)):
                    exact = float(F + k * R)
                    dev = abs(new[k] - exact) / np.spacing(max(abs(exact), 1.0))
                    worst_new = max(worst_new, dev)
                checked += 1
    assert worst_new <= 32.0, worst_new
    print(f"  {checked} configurations: counts match legacy; "
          f"worst edge deviation {worst_new:.1f} ulps (legacy arange: ~252)")


def test_edges_hash_detects_drift():
    bins_a = [make_edges(0, 100, 1.0), make_edges(0, 30, 0.5), make_edges(-1, 31, 0.1)]
    bins_b = [b.copy() for b in bins_a]
    assert edges_hash(bins_a) == edges_hash(bins_b)
    bins_b[2][5] = np.nextafter(bins_b[2][5], np.inf)   # one-ulp drift
    assert edges_hash(bins_a) != edges_hash(bins_b)


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"PASS  {t.__name__}")
    print(f"\nAll {len(tests)} contract tests passed. "
          f"Convention: half-open [lo, hi), BOUNDARY_EPS={BOUNDARY_EPS}.")


if __name__ == "__main__":
    main()