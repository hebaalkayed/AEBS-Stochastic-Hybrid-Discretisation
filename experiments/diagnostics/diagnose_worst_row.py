"""
Diagnose where the worst row of the IMDP lives.

Reports max_K (worst per-row epsilon sum) over four restrictions
of the state space, so you can see whether the SA13 bound is loose
because of one pathological corner or because of a broader region.

Usage: import and call from pipeline.py after finalize_sink_state().
"""


def per_row_K(dist_str: str) -> float:
    """Recover sum_j epsilon_ij from a stored interval string.

    The IMDP stores transitions as strings like
        "[p_min, p_max] : (s'=j) + [p_min, p_max] : (s'=k) + ..."
    For each interval, epsilon_ij = (p_max - p_min) / 2.
    """
    K = 0.0
    for part in dist_str.split(' + '):
        try:
            bracket = part.strip().split(']')[0].lstrip('[')
            p_min_str, p_max_str = bracket.split(',')
            p_min = float(p_min_str.strip())
            p_max = float(p_max_str.strip())
            K += (p_max - p_min) / 2.0
        except (ValueError, IndexError):
            continue
    return K


def diagnose(plant_imdp, grid, N_horizon=80):
    """
    Inspect worst rows under various restrictions of the state space.

    Args:
        plant_imdp: the IMDP object (after finalize_sink_state)
        grid:       the Grid object
        N_horizon:  horizon for E computation
    """
    n_g, n_v, n_vl = grid.shape
    bins_g, bins_v, bins_vl = grid.bins

    def cell_centre(flat_id):
        """Decode a flat state id into (g, v_ego, v_lead, i_g, i_v, i_vl)."""
        i_g = flat_id // (n_v * n_vl)
        rem = flat_id % (n_v * n_vl)
        i_v = rem // n_vl
        i_vl = rem % n_vl
        if i_g >= n_g or i_v >= n_v or i_vl >= n_vl:
            return None  # sink state, skip
        g  = (bins_g[i_g]   + bins_g[i_g + 1])   / 2.0
        v  = (bins_v[i_v]   + bins_v[i_v + 1])   / 2.0
        vl = (bins_vl[i_vl] + bins_vl[i_vl + 1]) / 2.0
        return g, v, vl, i_g, i_v, i_vl

    # Four filters of increasing physical strictness.
    # c = (g, v_ego, v_lead, i_g, i_v, i_vl)
    filters = {
        '(a) all states':
            lambda c: True,
        '(b) exclude v_lead edges (3 cells from each end)':
            lambda c: 3 <= c[5] <= n_vl - 4,
        '(c) exclude v_lead edges AND gap=0':
            lambda c: 3 <= c[5] <= n_vl - 4 and c[3] >= 1,
        '(d) physically realistic (g>1, 0<=v_lead<=28)':
            lambda c: c[0] > 1.0 and 0.0 <= c[2] <= 28.0,
    }

    results = {name: {'max_K': 0.0, 'state': None, 'action': None, 'centre': None}
               for name in filters}

    n_scanned = 0
    for src, actions in plant_imdp.transitions.items():
        c = cell_centre(src)
        if c is None:
            continue  # skip sink
        n_scanned += 1
        for act, dist_str in actions.items():
            K = per_row_K(dist_str)
            for name, keep in filters.items():
                if keep(c) and K > results[name]['max_K']:
                    results[name]['max_K']  = K
                    results[name]['state']  = src
                    results[name]['action'] = act
                    results[name]['centre'] = (c[0], c[1], c[2])

    print(f"\n[Diagnose] Scanned {n_scanned} non-sink states\n")
    header = f"{'Filter':<55} {'max_K':>10} {'E (N=' + str(N_horizon) + ')':>12}  worst centre"
    print(header)
    print("-" * len(header))
    for name, r in results.items():
        E = N_horizon * r['max_K']
        if r['centre']:
            g, v, vl = r['centre']
            cent = f"g={g:.1f}, v={v:.1f}, vl={vl:.1f} (act={r['action']})"
        else:
            cent = "-"
        print(f"{name:<55} {r['max_K']:>10.4f} {E:>12.2f}  {cent}")
    print()
    return results

