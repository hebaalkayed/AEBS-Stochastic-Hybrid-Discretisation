import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.abstraction.algorithms.discretizer import compute_range_epsilon_ij, fast_norm_cdf

sigma = 0.2   # baseline noise
w     = 0.25  # cell width

# Helper: nominal prob at centroid
def p_nominal(src_centre, tgt_lo, tgt_hi, sigma):
    return fast_norm_cdf(tgt_hi, src_centre, sigma) \
         - fast_norm_cdf(tgt_lo, src_centre, sigma)

cases = [
    ("Central cell  (kernel centred ON target)",
     9.875, 10.125,   # source cell
     9.875, 10.125),  # target cell = same
    ("Adjacent cell (kernel centred ONE cell away)",
     9.875, 10.125,
     10.125, 10.375),
    ("Far cell      (kernel centred TWO cells away)",
     9.875, 10.125,
     10.375, 10.625),
]

print(f"{'Case':<45} {'p_nom':>6} {'eps_new':>8} {'p_min':>7} {'p_max':>7}")
print("-" * 80)
for name, s_lo, s_hi, t_lo, t_hi in cases:
    src_c = (s_lo + s_hi) / 2
    p     = p_nominal(src_c, t_lo, t_hi, sigma)
    eps   = compute_range_epsilon_ij(s_lo, s_hi, t_lo, t_hi, sigma)
    print(f"{name:<45} {p:>6.4f} {eps:>8.4f} {max(0,p-eps):>7.4f} {min(1,p+eps):>7.4f}")

# Assertions: these must hold or something is wrong
assert compute_range_epsilon_ij(9.875, 10.125, 9.875, 10.125, 0.2) < 0.63, \
    "New epsilon must be tighter than old Lipschitz epsilon"
assert max(0, p_nominal(10.0, 9.875, 10.125, 0.2)
           - compute_range_epsilon_ij(9.875, 10.125, 9.875, 10.125, 0.2)) > 0.3, \
    "Central cell lower bound must be non-trivially positive"
assert compute_range_epsilon_ij(9.875, 10.125, 9.875, 10.125, 0.0) == 0.0, \
    "Deterministic dimension must give zero epsilon"
print("\nAll assertions passed.")