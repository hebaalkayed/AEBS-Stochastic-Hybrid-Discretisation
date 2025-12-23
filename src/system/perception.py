import numpy as np

class PerceptionSystem:
    """
    Simulates a Neural Network-based perception system.
    Now includes PARAMETRIZED robustness.
    """
    def __init__(self, false_negative_rate=0.05, false_positive_rate=0.01):
        # Parametrize the NN's quality
        self.fn_rate = false_negative_rate # Dangerous: Misses the car
        self.fp_rate = false_positive_rate # Annoying: Ghost braking
        
    def observe(self, true_distance):
        """
        Returns: 1 (OBSTACLE) or 0 (CLEAR)
        """
        # 1. Determine Ground Truth
        # (Assuming sensor range is 100m)
        if true_distance <= 100.0:
            ground_truth = 1
        else:
            ground_truth = 0
            
        # 2. Apply Confusion Matrix Logic
        if ground_truth == 1:
            # Chance to miss the car (False Negative)
            if np.random.rand() < self.fn_rate:
                return 0 
            else:
                return 1
        else:
            # Chance to hallucinate a car (False Positive)
            if np.random.rand() < self.fp_rate:
                return 1
            else:
                return 0