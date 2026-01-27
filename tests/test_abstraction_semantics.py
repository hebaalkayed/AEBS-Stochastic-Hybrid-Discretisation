import sys
import os
import unittest

# Ensure we can import from src
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# IMPORT THE REAL CLASSES
from src.system.controller import AEBSController
from src.utils.mdp_labeling import LabelingGrammar, Labels

class TestRealLabelingIntegration(unittest.TestCase):
    
    def test_safe_controller_binding(self):
        """
        Verifies that LabelingGrammar correctly pulls parameters 
        from the REAL Safe Controller.
        """
        print("\n--- Testing SAFE Controller Integration ---")
        
        # 1. Instantiate the Real Controller
        real_safe_controller = AEBSController(mode='safe')
        
        # 2. Instantiate the Grammar with it
        grammar = LabelingGrammar(real_safe_controller)
        
        # 3. Check Parameter Binding (The "Handshake")
        # We verify the Grammar actually read the Controller's attributes
        print(f"Controller Warn Dist: {real_safe_controller.dist_warn}")
        print(f"Grammar Threshold:    {grammar.thresh_warn}")
        
        self.assertEqual(grammar.thresh_warn, real_safe_controller.dist_warn, 
                         "CRITICAL FAIL: Grammar did not read 'dist_warn' from Safe Controller!")
        
        # 4. Check Semantic Logic
        # Safe Controller warns at 15m. Test at 12m.
        gap_12m = (12.0, 10.0, 0.0) # Gap, V_rel, Accel
        labels = grammar.get_labels(gap_12m)
        
        print(f"Input Gap: 12.0m -> Labels: {labels}")
        self.assertIn(Labels.BRAKING, labels, 
                      "FAIL: Safe Grammar should label 12m as 'braking' (Threshold is 15m)")

    def test_industry_controller_binding(self):
        """
        Verifies that the SAME Grammar class behaves differently 
        when connected to the Industry Controller.
        """
        print("\n--- Testing INDUSTRY Controller Integration ---")
        
        # 1. Instantiate Real Industry Controller
        real_industry_controller = AEBSController(mode='industry')
        
        # 2. Instantiate Grammar
        grammar = LabelingGrammar(real_industry_controller)
        
        # 3. Check Parameter Binding
        self.assertEqual(grammar.thresh_warn, real_industry_controller.dist_warn, 
                         "CRITICAL FAIL: Grammar did not read 'dist_warn' from Industry Controller!")
        
        # 4. Check Semantic Logic (The "Blind Spot" Test)
        # Industry Controller warns at 10m. Test at 12m.
        gap_12m = (12.0, 10.0, 0.0)
        labels = grammar.get_labels(gap_12m)
        
        print(f"Input Gap: 12.0m -> Labels: {labels}")
        self.assertNotIn(Labels.BRAKING, labels, 
                         "FAIL: Industry Grammar should NOT label 12m as 'braking' (Threshold is 10m)")
        self.assertIn(Labels.SAFE, labels)

    def test_crash_boundary(self):
        """
        Verifies the physical ground truth logic (independent of controller).
        """
        print("\n--- Testing Physical Boundaries ---")
        controller = AEBSController(mode='safe')
        grammar = LabelingGrammar(controller)
        
        # Exact Zero
        labels_zero = grammar.get_labels((0.0, 10.0, 0.0))
        self.assertIn(Labels.CRASH, labels_zero, "Gap 0.0 must be CRASH")
        
        # Slightly Negative
        labels_neg = grammar.get_labels((-0.001, 10.0, 0.0))
        self.assertIn(Labels.CRASH, labels_neg, "Gap -0.001 must be CRASH")

if __name__ == "__main__":
    unittest.main()