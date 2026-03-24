from abc import ABC, abstractmethod
import numpy as np
from typing import Dict, Tuple


class StochasticHybridSystem(ABC):
    """
    Interface for any system (Plant, Controller, Robot) that
    needs to be discretised by the IMDP construction algorithm.
    """

    @abstractmethod
    def get_action_space(self) -> Dict[int, float]:
        """
        Returns a map of {discrete_action_id: continuous_action_value}.

        Example: {0: 0.0, 1: -4.0, 2: -9.8}
        """
        pass

    @abstractmethod
    def get_next_state_distribution(
        self,
        state:  np.ndarray,
        action: float
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Returns the parameters that characterise the one-step transition kernel.

        Return value: (deterministic_next_state, sigma_per_dim)

        ── deterministic_next_state  (np.ndarray, shape (n,)) ───────────────
        Nominal next state from the deterministic plant dynamics f(state, action).
        This is the mean of the stochastic kernel (the point around which the
        Gaussian is centred in the stochastic dimensions).

        ── sigma_per_dim  (np.ndarray, shape (n,)) ──────────────────────────
        Per-dimension Gaussian noise standard deviation.
        Set to 0.0 for deterministic dimensions (delta / indicator kernel).
        Set to sigma > 0 for stochastic dimensions (Gaussian CDF kernel).

        This is physically correct for the AEBS plant:
            sigma_per_dim = [0.0, 0.0, sigma_v_lead]
        because noise only enters through the lead vehicle's uncertain acceleration,
        which propagates to v_lead (dimension 2) over one timestep. Gap (dim 0)
        and v_ego (dim 1) are deterministic given the state and action.

        CRITICAL: Using a single global sigma for all dimensions causes probability
        mass to leak into physically unreachable cells in the deterministic dimensions
        AND prevents the range-based epsilon from being tight (see below).

        ── Epsilon computation (soudjani2013-abstraction branch) ─────────────
        The discretiser no longer needs L_t or L_q. Instead, the per-transition
        error bound epsilon_ij is computed directly from sigma_per_dim and the
        source/target cell boundaries using the range-based formula of
        Soudjani & Abate (2013), Equation 3.11:

            epsilon_ij = max_{s in Ai} p(Xj|s) - min_{s in Ai} p(Xj|s)

        where p(Xj|s) = Phi((hi_j - mu(s))/sigma) - Phi((lo_j - mu(s))/sigma)
        is the Gaussian CDF integral over target cell j given source point s.

        For the AEBS plant with sigma = 0.2 m/s and cell width 0.25 m/s:
            epsilon_ij (central cell) ~ 0.08   [vs 0.63 with Lipschitz bound]
            p_min (central cell)      ~ 0.39   [vs 0.0 with Lipschitz bound]
            p_max (central cell)      ~ 0.55   [tighter than before]

        Both Pmax and Pmin PRISM queries are now informative.
        """
        pass