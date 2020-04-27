#!/usr/bin/env python
import yaml
import numpy as np
import os
import rospkg 

HARDWARE_CONFIG = '../config/hardware_config.yaml'
SOFTWARE_CONFIG = '../config/software_config.yaml'

def get_rad(x):
    return np.pi * x/180

class Decision:

    def _read_config(self):
        """
        Reads the configuration from the config file and adds them to the tree
        """
        rospack = rospkg.RosPack()
        #path = '../'
        path = rospack.get_path('react_agent') + os.path.sep
        with open(path + HARDWARE_CONFIG, 'r') as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            self.sensor_angle = get_rad(np.array(config['sensor_angle']))
        with open(path + SOFTWARE_CONFIG, 'r') as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            self.obstacle_threshold = config['obstacle_threshold']
            self.human_threshold = config['human_threshold']
            self.flee_dist = config['flee_dist']
            self.search_rotation = config['search_rotation']

    def __init__(self):
        self.current_distances = None 
        self.last_direction = None
        self.current_dist_human = -np.inf
        self.pos = [0.0, 0.0]
        self.yaw = [0.0, 0.0]
        self._read_config()

    def update_perception(self, distances, dir_human, dist_human):
        """
        Returns a python list of length 4 on the format : [vel_x, vel_y, vel_z, yaw] in the local frame. Yaw is in degrees
        dir_human, distances, dist_human 2-dimensional array
        Distances is a 8th dimensional array consisting of the measurments of the 8 sensors

        case = 0 => No human detected 
        case = 1 => human is close
        case = 2 => human is far away
        """
        
        # update perception
        assert isinstance(distances, np.ndarray), f'Expected distances to be of type ndarray but got {type(distances)}'
        self.current_distances = distances
        # TODO check if assumption holds
        if self.current_dist_human < 0:             
            # human detected -> save direction if human is lost
            assert isinstance(dir_human, np.ndarray), f'Expected dir_human to be of type ndarray but got {type(dir_human)}'
            self.last_direction = dir_human
        self.current_dist_human = dist_human
        
        # decide action
        if self.current_dist_human == -1:
            # case 0
            self.search_for_human()
        elif self.is_to_close():
            # case 1
            self.flee()
        else:
            # case 2
            self.adjust_yaw()

        return [self.pos, self.yaw]
    
    def is_to_close(self):
        """
        Checks if the human is to close to the drone
        """
        return self.current_dist_human <= self.human_threshold

    def flee(self):
        """
        Flee from human (backwards). Consider envirnment. 
        """
        self.adjust_yaw() #TODO ensure that vertical adjustment is ignored 
        # ignore sensors that are directed forwards
        distances = self.current_distances[2:7]
        # get correction distance to the sides
        # filter where distances are negative
        # TODO check if this holds
        distance_mask = distances > 0
        avoidance_vec = np.sin(self.sensor_angle[2:7] - np.pi)
        avoidance_vec = np.multiply(avoidance_vec, distances, where=distance_mask)
        
        # positive -> obstacle to the left, negative -> obstacle to the right
        
        left_mask = np.logical_and(self.obstacle_threshold < avoidance_vec, avoidance_vec > 0)
        left_mask = np.logical_and(left_mask, distance_mask)
        
        right_mask = np.logical_and(-self.obstacle_threshold > avoidance_vec, avoidance_vec < 0)
        right_mask = np.logical_and(right_mask, distance_mask)

        min_dist_left = np.where(left_mask, avoidance_vec, np.inf).min()
        min_dist_right = np.where(right_mask, avoidance_vec, -np.inf).max()
        
        # positive -> move right, negative -> move left
        if min_dist_left != np.inf and min_dist_right != -np.inf:
            # constriction
            raise NotImplementedError()
        elif min_dist_left != np.inf:
            # move right
            correction = self.obstacle_threshold - min_dist_left
        elif min_dist_right != -np.inf:
            # move left
            correction = -(self.obstacle_threshold + min_dist_right)
        else:
            correction = 0
        
        # check for obstacle behind
        avoidance_vec = np.cos(self.sensor_angle[3:6] - np.pi) * distances[1:-1]
        if np.any(avoidance_vec >= self.obstacle_threshold):
            # TODO: check expected movement
            correction = (correction, 0)
        else:
            correction = (correction, self.flee_dist)

        self.pos = correction        
    
    def adjust_yaw(self):
        """
        Adjust the current yaw based on the human direction
        """
        self.yaw = -self.last_direction
    
    def search_for_human(self):
        """
        Adjust the yaw.
        Sets the roation to 90 in the last direction of the human
        """
        self.yaw = -np.sign(self.last_direction) * self.search_rotation
