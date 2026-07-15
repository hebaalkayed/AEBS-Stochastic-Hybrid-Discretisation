"""Ego-centric verification scenarios, decoupled from lead behaviour.

Scenarios are START STATES of the composed system, chosen for what the EGO
vehicle's situation is at time zero: cruising with no need to brake, inside
the warning region, inside the emergency region, an unavoidable approach,
and a post-collision state used as an end-to-end sanity check. They are
properties of the grid and the controller regions, not of any lead profile;
a LeadModel supplies dynamics (the v_lead rule, its noise, the grid band)
and nothing else.

Each scenario is (name, (gap, v_ego, v_lead)) in continuous coordinates;
resolve() maps it to the flat cell id of a given grid (the cell whose centre
is the stated state, by the half-open assignment). On the certified
medium_tight partition (bins_hash 3672963e58eb75b5) the ids are pinned in
tests and artifact metadata:
    Safe_Cruising      1967740
    Warning_Brake       591934
    Emergency_Brake     202240
    Imminent_Collision  117175
    Post_Collision          10
"""

# The canonical catalogue. v_lead = 0 states these against a stationary
# lead (Euro NCAP CCR staging); scenarios for other initial lead speeds can
# be added here without touching any lead model.
SCENARIOS = [
    ("Safe_Cruising",      (100.0, 15.0, 0.0)),
    ("Warning_Brake",      ( 30.0,  7.0, 0.0)),
    ("Emergency_Brake",    ( 10.0, 10.0, 0.0)),
    ("Imminent_Collision", (  5.0, 30.0, 0.0)),
    ("Post_Collision",     (  0.0,  0.0, 0.0)),
]


def resolve(grid, scenarios=None):
    """Map scenarios onto a grid: {name: flat_state_id}.

    Skips (and reports) any scenario whose continuous state falls outside
    the grid bounds, mirroring the pipeline's OUT OF BOUNDS handling.
    """
    out = {}
    for name, state in (scenarios or SCENARIOS):
        idx = grid.state_to_index(state)
        if idx is None:
            print(f"[Scenarios] {name}: {state} is OUT OF BOUNDS on this "
                  f"grid; skipped")
            continue
        out[name] = grid.get_flat_index(idx)
    return out