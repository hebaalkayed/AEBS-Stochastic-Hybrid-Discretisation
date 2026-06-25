import math
import numpy as np

class VehiclePlant:
    """
    The "Real" Physics Plant.

    Coordinate Systems:
    - 'world_frame': x is position on track. v is velocity. (x increases with v)
    - 'relative_frame': x is distance to obstacle (Gap). v is closing speed.

    EGO vs LEAD (refactor, behaviour-preserving):
        The ego velocity update is controller-driven (consumes the commanded
        acceleration). The lead velocity update is EXOGENOUS — it does not run the
        AEBS controller. The plant exposes the two as separate methods (ego_next,
        gap_next) plus a default lead rule (lead_next_default = max(0, v_lead)).
        get_deterministic_next_state(state, action) keeps its exact original
        signature and output; it now simply composes ego_next + lead_next_default
        + gap_next. An injected LeadModel can override the lead rule by passing
        v_lead_next; when omitted, the default reproduces the original numbers.
    """
    def __init__(self, x=0.0, y=0.0, theta=0.0, v=0.0, a=0.0, alpha=0.5, dt=0.1, coordinate_system='world_frame'):
        self.x = x
        self.y = y
        self.theta = theta
        self.actual_velocity = v
        self.actual_acceleration = a
        self.alpha = alpha
        self.dt = dt
        self.coordinate_system = coordinate_system
        self.process_noise_std = 0.05

    def step(self, action_acceleration):
        """Continuous simulation only — NOT called by the abstraction."""
        self.actual_acceleration = self.actual_acceleration + self.alpha * (action_acceleration - self.actual_acceleration)
        self.actual_velocity += self.actual_acceleration * self.dt
        if self.actual_velocity < 0.0:
            self.actual_velocity = 0.0
            self.actual_acceleration = 0.0
        if self.coordinate_system == 'world_frame':
            self.x += self.actual_velocity * self.dt
        else:
            self.x -= self.actual_velocity * self.dt

    # =========================================================
    #  ABSTRACTION INTERFACE  (split, behaviour-preserving)
    # =========================================================

    def ego_next(self, v_ego, action_acceleration):
        """Ego velocity update (controller-driven). First-order lag from a=0
        baseline: a_ego = alpha*action; floor at 0. Identical to the original."""
        a_ego = self.alpha * action_acceleration
        v_ego_next = v_ego + a_ego * self.dt
        if v_ego_next < 0.0:
            v_ego_next = 0.0
        return v_ego_next

    def lead_next_default(self, v_lead):
        """Default exogenous lead rule: hold speed, floored at 0. This is the
        original plant behaviour; an injected LeadModel may replace it."""
        return max(0.0, v_lead)

    def gap_next(self, gap, v_ego_next, v_lead_next):
        """Relative-frame gap update. Identical to the original expression."""
        return gap + (v_lead_next - v_ego_next) * self.dt

    def get_deterministic_next_state(self, state_vector, action_acceleration, v_lead_next=None):
        """
        Deterministic next (gap, v_ego, v_lead).

        ORIGINAL SIGNATURE PRESERVED. With v_lead_next=None (every current
        caller) this composes ego_next + lead_next_default + gap_next, which is
        byte-identical to the previous implementation:

            For (40, 15, 0), brake=-4:
                v_ego_next = 14.8, v_lead_next = 0.0, gap_next = 38.52
                -> [38.52, 14.8, 0.0]

        An injected LeadModel passes v_lead_next to override only the lead rule.
        """
        gap, v_ego, v_lead = state_vector
        v_ego_n = self.ego_next(v_ego, action_acceleration)
        if v_lead_next is None:
            v_lead_next = self.lead_next_default(v_lead)
        gap_n = self.gap_next(gap, v_ego_n, v_lead_next)
        return np.array([gap_n, v_ego_n, v_lead_next])

    @property
    def noise_std(self):
        return self.process_noise_std