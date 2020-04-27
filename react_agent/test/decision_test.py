#!/usr/bin/env python3
import unittest
import numpy as np
#import sys
#sys.path.append('../src')
from react_agent.decision import Decision
import os
import rospkg 
import yaml

PKG = 'react_agent'
SOFTWARE_CONFIG = '../config/software_config.yaml'

class TestDecision(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        """
        Reads the configuration from the config file and adds them to the tree
        """
        super(TestDecision, self).__init__(*args, **kwargs)
        rospack = rospkg.RosPack()
        path = '../'
        path = rospack.get_path('react_agent') + os.path.sep
        with open(path + SOFTWARE_CONFIG, 'r') as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            self.obstacle_threshold = config['obstacle_threshold']
            self.human_threshold = config['human_threshold']
            self.flee_dist = config['flee_dist']
            self.search_rotation = config['search_rotation']

    def test_yaw_correction(self):
        ai = Decision()
        distances = np.full(8, -1.0)
        human_dist = np.inf
        human_dir = np.array([np.pi/4, -np.pi/4])
        _, yaw = ai.update_perception(distances, human_dir, human_dist)
        self.assertTrue(np.all(human_dir == -yaw))
    
    def test_flee(self):
        ai = Decision()
        distances = np.full(8, -1.0)
        human_dist = self.human_threshold
        human_dir = np.zeros(2)
        position, _ = ai.update_perception(distances, human_dir, human_dist)
        self.assertEqual(position[1], self.flee_dist) 

if __name__ == '__main__':
    #unittest.main()
    import rostest
    rostest.rosrun(PKG, 'test_decision', TestDecision)
