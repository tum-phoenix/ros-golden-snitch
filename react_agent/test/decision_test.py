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
        #path = '../'
        path = rospack.get_path('react_agent') + os.path.sep
        with open(path + SOFTWARE_CONFIG, 'r') as f:
            config = yaml.load(f, Loader=yaml.SafeLoader)
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
        self.assertTrue(np.all(position == np.array([0, -self.flee_dist]))) 

    def test_avoid_wall(self):
        ai = Decision()
        distances = np.full(8, -1.0)
        distances[5] = self.obstacle_threshold * 0.9
        human_dist = np.inf
        human_dir = np.zeros(2)
        position, _ = ai.update_perception(distances, human_dir, human_dist)
        self.assertTrue(np.all(position > 0))

    def test_flee_along_obstacle(self):
        ai = Decision()
        distances = np.full(8, -1.0)
        distances[3] = self.obstacle_threshold * 0.9
        distances[4] = self.obstacle_threshold * 0.9
        human_dist = self.human_threshold * 0.9
        human_dir = np.zeros(2)
        position, _ = ai.update_perception(distances, human_dir, human_dist)
        self.assertEqual(position[0], -self.flee_dist)
        self.assertTrue(position[1] > 0)
    
    def test_adversarial_direction(self):
        ai = Decision()
        distances = np.full(8, -1.0)
        distances[0] = self.obstacle_threshold * 0.9
        distances[4] = self.obstacle_threshold * 0.9
        human_dist = np.inf
        human_dir = np.zeros(2)
        position, _ = ai.update_perception(distances, human_dir, human_dist)
        self.assertTrue(np.all(position == np.zeros(2)))
    
if __name__ == '__main__':
    #unittest.main()
    import rostest
    rostest.rosrun(PKG, 'test_decision', TestDecision)
