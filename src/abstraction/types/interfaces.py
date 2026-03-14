from abc import ABC, abstractmethod
import numpy as np
from typing import Dict, Tuple

class StochasticHybridSystem(ABC):
    """
    Interface for any system (Plant, Controller, Robot) that 
    needs to be discretized by the Generic Algorithm.
    """
    
    @abstractmethod
    def get_action_space(self) -> Dict[int, float]:
        """
        Returns map of {discrete_id: continuous_value}.
        Example: {0: 0.0, 1: -4.0}
        """
        pass

    @abstractmethod
    def get_next_state_distribution(self, state: np.ndarray, action: float) -> Tuple[np.ndarray, np.ndarray, float, float]:
        """
        Returns the parameters for the transition kernel:

            1. deterministic_next_state (np.ndarray, shape (n,)):
               Nominal next state from deterministic dynamics.

            2. sigma_per_dim (np.ndarray, shape (n,)):
               Per-dimension Gaussian noise standard deviation.
               Set to 0.0 for deterministic dimensions (delta-function kernel).
               This is physically correct — noise only enters the dimensions
               where it physically occurs (e.g. v_lead in the AEBS plant).

               CRITICAL: Applying a single global sigma to all dimensions
               causes mass to leak into wrong cells in deterministic dimensions
               AND inflates epsilon via dimensional mismatch (see below).

            3. L_t (float):
               Lipschitz constant of deterministic dynamics.
               Spectral norm of the Jacobian df/dx.

            4. L_q (float):
               Lipschitz constant of the stochastic kernel.
               For a Gaussian with std sigma_noise:
                   L_q = 2 / (sigma_noise * sqrt(2*pi))
               Bounds how much transition probabilities change per unit
               shift in the kernel centre (mean).

        EPSILON BOUND (Abate et al. Theorem 1):
            epsilon = (L_t + L_q) * delta_stochastic
        where delta_stochastic = resolution[stochastic_dim] / 2.0
        (half cell width IN THE STOCHASTIC DIMENSION ONLY).

        This is the correct dimensional form for systems where noise enters
        only one dimension. Using a global Euclidean cell_diameter that mixes
        physical units (e.g. metres and m/s) inflates epsilon >> 1, collapsing
        all IMDP intervals to [1e-6, 1.0] — formally sound but void.

        With correct per-dimension sigma and epsilon:
            L_q  = 2 / (0.2 * sqrt(2pi)) ≈ 4.0
            L_t  ≈ 1.0
            delta_vlead = 0.125 m/s  (for 121-cell grid over 30 m/s)
            epsilon = (1.0 + 4.0) * 0.125 = 0.625  → meaningful intervals ✓
        """
        pass