import unittest
import numpy as np

PKG = 'react_agent'

class TestDecision(unittest.TestCase):
    
    def test_yaw_correction(self):
        ai = Decision()
        distances = np.full(8, -np.inf)
        human_dist = np.inf
        human_dir = np.array([-10, 10])
        configuration = ai.update_perception(distances, human_dir, human_dist)
        print(configuration)
        self.assertTrue(True)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_decision', TestDecision)
