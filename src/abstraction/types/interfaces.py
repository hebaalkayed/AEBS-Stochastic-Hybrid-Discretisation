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
    def get_next_state_distribution(self, state: np.ndarray, action: float) -> Tuple[np.ndarray, float, float]:
        """
        Returns the parameters for the transition kernel:
            1. deterministic_next_state (array)
            2. noise_std (float)
            3. lipschitz_constant (float)
        """
        pass