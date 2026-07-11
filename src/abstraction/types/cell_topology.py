"""Single source of truth for the grid's boundary convention.

Cells are HALF-OPEN, [lo, hi): a point on a shared edge belongs to the
cell ABOVE it (np.digitize right=False semantics: bins[k] <= x < bins[k+1]
puts x in cell k). Every site that assigns states to cells, tests certain
coverage, tests grid escape, enumerates touched cells, or flattens grid
indices MUST go through these functions. Do not write inline comparisons
or inline digitize calls: three sites agreeing by luck is exactly how the
exact-edge certain-coverage bug happened.

The convention is load-bearing in this system because the gap and v_ego
kernels are Dirac point masses: a boundary-exact image places probability
ONE on whichever cell owns the boundary, so ownership decides actual
transition probabilities (unlike a density kernel, where boundaries carry
zero mass and the convention would be unobservable).

tests/test_cell_topology.py pins these semantics; it must fail if anyone
edits them.
"""

import hashlib
import numpy as np

CONVENTION = "half-open [lo, hi): lower edge in, upper edge out (np.digitize right=False)"

# Enumeration is expanded by this much before digitize() so that a cell the
# image touches only at an exact grid boundary is still enumerated. The image
# extremes can land on a bin edge (e.g. next_gap == 0.5 exactly), where float
# rounding would otherwise place the boundary state one cell outside the
# enumerated range. EPS (>> float64 noise ~1e-16, << one cell width) closes
# that gap; the boundary cell enters with factor [0, 1], so it is sound.
BOUNDARY_EPS = 1e-9


# ---------------------------------------------------------------------
#  State-to-cell assignment
# ---------------------------------------------------------------------
def cell_of(value, edges):
    """Index of the half-open cell containing value; None if outside the grid.

    A value exactly on an interior edge belongs to the cell ABOVE it.
    A value exactly on the TOP edge is outside the grid (returns None).
    A value exactly on the BOTTOM edge is inside cell 0.
    """
    k = int(np.digitize(value, edges) - 1)
    return k if 0 <= k < len(edges) - 1 else None


# ---------------------------------------------------------------------
#  Certain coverage (deterministic-dimension lower factor)
# ---------------------------------------------------------------------
def certainly_within(img_lo, img_hi, cell_lo, cell_hi):
    """True iff the closed image interval [img_lo, img_hi] lies wholly inside
    the half-open cell [cell_lo, cell_hi).

    The upper inequality is STRICT: an image endpoint landing exactly on
    cell_hi belongs to the cell above, so certain coverage of THIS cell is
    false there. With a non-strict inequality the kernel claims a lower
    bound of ~1 for a cell that a boundary-exact state reaches with
    probability 0 (a containment violation).
    """
    return bool(img_lo >= cell_lo) and bool(img_hi < cell_hi)


# ---------------------------------------------------------------------
#  Grid escape (deterministic dimensions)
# ---------------------------------------------------------------------
def escapes_top(img_hi, edges):
    """True iff some image point is outside the grid above.

    The top edge itself is OUTSIDE the grid (the last cell is
    [edges[-2], edges[-1]) ), hence >= not >.
    """
    return bool(img_hi >= edges[-1])


def fully_outside_top(img_lo, edges):
    """True iff the WHOLE image interval is outside the grid above."""
    return bool(img_lo >= edges[-1])


def escapes_bottom(img_lo, edges):
    """True iff some image point is outside the grid below.

    The bottom edge itself is INSIDE cell 0, hence < not <=.
    """
    return bool(img_lo < edges[0])


def fully_outside_bottom(img_hi, edges):
    """True iff the WHOLE image interval is outside the grid below."""
    return bool(img_hi < edges[0])


# ---------------------------------------------------------------------
#  Enumeration of touched cells
# ---------------------------------------------------------------------
def cells_touched(img_lo, img_hi, edges, pad=0.0):
    """Inclusive, grid-clipped index range (klo, khi) of the cells the closed
    interval [img_lo - pad, img_hi + pad] touches.

    pad carries the SIGMA_CUTOFF window for the stochastic dimension
    (pad = k * sigma); pass pad=0.0 for deterministic dimensions.
    The range is BOUNDARY_EPS-expanded on both sides so that a cell touched
    only at an exact grid boundary is still enumerated (with factor [0, 1],
    which is sound). May return klo > khi when the padded interval lies
    wholly outside the grid; callers treat that as an empty range.
    """
    n_cells = len(edges) - 1
    klo = max(0, int(np.digitize(img_lo - pad - BOUNDARY_EPS, edges) - 1))
    khi = min(n_cells - 1, int(np.digitize(img_hi + pad + BOUNDARY_EPS, edges) - 1))
    return klo, khi


# ---------------------------------------------------------------------
#  Flat state identifiers
# ---------------------------------------------------------------------
def flat_index(kg, kv, kl, shape):
    """Row-major flat state id: kg*(nv*nvl) + kv*nvl + kl.

    Single definition of the state layout (previously duplicated in the
    discretizer product loop, the discretizer worker, the Grid class, and
    the containment test). Distinct index triples map to distinct ids
    (bijection onto [0, n_g*n_v*n_vl)), which is why per-target interval
    'merging' can never occur.
    """
    return (kg * shape[1] + kv) * shape[2] + kl


# ---------------------------------------------------------------------
#  Deterministic edge construction and fingerprint
# ---------------------------------------------------------------------
def n_cells_for(low, high, r, tol=1e-9):
    """Number of grid cells for bounds [low, high] and resolution r, by
    integer arithmetic with a divisibility tolerance.

    Matches the legacy arange count semantics: when (high - low)/r is an
    integer m (to tolerance), the grid has m + 1 cells (the half-cell offset
    adds one); when it is not, the count rounds UP so the last cell extends
    past 'high' rather than truncating the state space.
    """
    q = (high - low) / r
    m = round(q)
    cells_across = int(m) if abs(q - m) < tol else int(np.ceil(q))
    return cells_across + 1


def make_edges(low, high, r):
    """Cell edges for bounds [low, high] and resolution r, with the half-cell
    offset placing integer coordinate values at cell centres.

    Each edge is the direct per-element expression  (low - r/2) + k*r  in
    float64 (one multiply, one add). This differs from the legacy
    np.arange(low - r/2, high + r + r/2, r) in two deliberate ways:
      1. COUNT: fixed by integer arithmetic (n_cells_for), identical to the
         legacy count but immune to the float stop-comparison that lets an
         arange length drift by one across platforms or numpy versions
         (laptop vs cluster).
      2. VALUES: numpy's arange ACCUMULATES, drifting up to ~250 ulps from
         the exact edge value across the presets; the direct expression stays
         within ~16 ulps. The values therefore differ from legacy at the ulp
         level; the partition fingerprint (edges_hash) changes at the
         refactor that introduced this function, and artifacts generated
         before it are not bit-comparable (they are already invalidated by
         the escape-routing fix).
    tests/test_cell_topology.py pins the count compatibility, the accuracy
    bound, and bitwise determinism of repeated construction.
    """
    n_edges = n_cells_for(low, high, r) + 1
    first = low - r / 2.0
    return first + np.arange(n_edges, dtype=np.float64) * r


def edges_hash(bins):
    """SHA-256 fingerprint of the exact edge floats, for artifact metadata.

    Two runs (e.g. generation on the cluster, containment testing locally)
    operate on the same partition iff their hashes match; comparing hashes
    turns a silent cross-platform drift into a detectable mismatch.
    """
    h = hashlib.sha256()
    for b in bins:
        h.update(np.ascontiguousarray(np.asarray(b, dtype=np.float64)).tobytes())
    return h.hexdigest()