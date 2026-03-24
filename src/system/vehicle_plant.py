import math
import numpy as np

class VehiclePlant:
    """
    The "Real" Physics Plant.
    
    Coordinate Systems:
    - 'world_frame': x is position on track. v is velocity. (x increases with v)
      -> Used for Visual Simulation.
      
    - 'relative_frame': x is distance to obstacle (Gap). v is closing speed. (x decreases with v)
      -> Used for Formal Verification (PRISM).
    """
    def __init__(self, x=0.0, y=0.0, theta=0.0, v=0.0, a=0.0, alpha=0.5, dt=0.1, coordinate_system='world_frame'):
        self.x = x
        self.y = y
        self.theta = theta
        self.actual_velocity = v
        self.actual_acceleration = a
        
        # Dynamics Parameters
        self.alpha = alpha  # Lag factor
        self.dt = dt # Time step,  decision loop runs 10 times a second
        self.coordinate_system = coordinate_system 
        
        # Abstraction Parameters
        self.process_noise_std = 0.05 

    def step(self, action_acceleration):
        """
        Advances the system by one time step (dt).
        Used for continuous simulation only — NOT called by the abstraction.
        """
        # 1. Update Acceleration (First-Order Lag)
        self.actual_acceleration = self.actual_acceleration + self.alpha * (action_acceleration - self.actual_acceleration)

        # 2. Update Velocity (Or Closing Speed)
        self.actual_velocity += self.actual_acceleration * self.dt
        
        # Constraints: No negative velocity
        if self.actual_velocity < 0.0:
            self.actual_velocity = 0.0
            self.actual_acceleration = 0.0

        # 3. Update Position (Longitudinal)
        if self.coordinate_system == 'world_frame':
            # World Frame: x increases as we drive forward
            self.x += self.actual_velocity * self.dt
        else:
            # Relative Frame: x is GAP. Positive closing speed REDUCES the gap.
            self.x -= self.actual_velocity * self.dt

    # =========================================================
    #  ABSTRACTION INTERFACE
    # =========================================================

    def get_deterministic_next_state(self, state_vector, action_acceleration):
        """
        Computes the deterministic next state for the AEBS 3D state space.

        State space: (gap [m], v_ego [m/s], v_lead [m/s])

        BUG FIX — complete rewrite:
        -----------------------------------------------------------------------
        ORIGINAL BUG:
            The method unpacked state_vector as (x, v, a), treating the three
            AEBS state dimensions as (position, velocity, ego_acceleration).
            This caused v_lead (dim 2) to be fed into the first-order lag
            formula as if it were ego acceleration:

                a_next = v_lead + alpha * (action - v_lead)   ← WRONG

            and the returned third component was next_a_ego, not next_v_lead.

            Example: state=(40, 15, 0), brake action=-4
                a_next = 0 + 0.5*(-4-0) = -2.0
                returns [38.52, 14.8, -2.0]   ← v_lead=-2.0 is physically impossible
                Gaussian centred at -2.0 → 99.7% mass outside grid → safe_sink

        CORRECT AEBS DYNAMICS:
            gap:    next_gap   = gap + (v_lead - v_ego_next) * dt
                    (positive closing speed of lead relative to ego increases gap)

            v_ego:  next_v_ego = max(0, v_ego + a_ego * dt)
                    where a_ego = alpha * action  (first-order lag from a=0 baseline;
                    ego acceleration is NOT in the state vector so we assume a=0
                    at the start of each decision step, giving half the commanded
                    deceleration in the first step — physically conservative)

            v_lead: next_v_lead = max(0, v_lead)
                    Lead vehicle drives at constant speed deterministically.
                    Stochastic uncertainty (sigma_on_velocity) is added externally
                    by PlantWrapper and handled via the Gaussian kernel in the
                    discretiser. No deterministic change to v_lead here.

        RESULT:
            For state=(40, 15, 0), brake action=-4:
                a_ego     = 0.5 * (-4)   = -2.0 m/s²
                v_ego_next= 15 + (-2.0)*0.1 = 14.8 m/s
                v_lead_next = 0.0 m/s  (no change)
                gap_next  = 40 + (0 - 14.8)*0.1 = 38.52 m
                returns [38.52, 14.8, 0.0]   ← v_lead=0.0 stays in grid ✓
        -----------------------------------------------------------------------
        """
        gap, v_ego, v_lead = state_vector

        # 1. Ego velocity: first-order lag from a=0 baseline
        #    (ego acceleration is not stored in the state vector; assume a=0
        #    at the start of each decision step)
        a_ego = self.alpha * action_acceleration
        v_ego_next = v_ego + a_ego * self.dt
        if v_ego_next < 0.0:
            v_ego_next = 0.0

        # 2. Lead velocity: constant speed deterministically
        #    Gaussian noise (sigma_on_velocity) is applied externally by PlantWrapper.
        #    Physical floor: v_lead cannot go negative.
        v_lead_next = max(0.0, v_lead)

        # 3. Gap: relative frame
        #    gap increases when lead is faster than ego, decreases otherwise
        gap_next = gap + (v_lead_next - v_ego_next) * self.dt

        return np.array([gap_next, v_ego_next, v_lead_next])

    # @property
    # def lipschitz_constant(self):
    #     # Direction depends on frame: +1 for World, -1 for Relative
    #     direction = -1.0 if self.coordinate_system == 'relative_frame' else 1.0
        
    #     A = np.array([
    #         [1.0, direction * self.dt, direction * (1 - self.alpha) * (self.dt ** 2)],
    #         [0.0, 1.0,                 (1 - self.alpha) * self.dt],
    #         [0.0, 0.0,                 (1 - self.alpha)]
    #     ])
    #     return np.linalg.norm(A, ord=2) # Lipschitz Constant

    @property
    def noise_std(self):
        return self.process_noise_std