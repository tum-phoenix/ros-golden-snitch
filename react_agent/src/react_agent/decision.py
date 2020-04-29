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
            self.sensor_angle = get_rad(np.array(config['direction_of_range_sensors']))
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
        
        self.check_for_obstacles()
        self.adjust_yaw() 
        
        # decide action
        if self.current_dist_human == -1:
            # case 0
            self.search_for_human()
        elif self.is_to_close():
            # case 1
            self.flee()

        return [self.pos, self.yaw]
 
    def check_for_obstacles(self):
        """
        Returns the correction vlaues based on the range sensor information
        Return: [vertical_correction, horizontal_correction]
        """
        # filter mask to ignore invalid or irrelevant distances
        distances_mask = np.logical_and(
            self.current_distances > 0,
            self.current_distances < self.obstacle_threshold
        )
        if not np.any(distances_mask):
            # no obstacle
            self.pos = np.zeros(2)
        else: 
            # Dim: (2 x 8)
            distances = self.sensor_angle[distances_mask][np.newaxis, :].repeat(2, 0)
            distances[0] = np.sin(distances[0]) # horizontal
            distances[1] = np.cos(distances[1]) # vertical
            distances *= self.current_distances[distances_mask]
           
            # check for adversarial directions
            adversarial = np.logical_and(np.any(distances < 0, axis=1), np.any(distances > 0, axis=1))
            if np.any(adversarial):
                # TODO just stop ?
                max_correction = np.zeros(2)
            else:
                max_correction_idx = np.abs(distances).argmax(axis=1)
                max_correction = -distances[max_correction_idx][:, 0]
            self.pos = max_correction

    def is_to_close(self):
        """
        Checks if the human is to close to the drone
        """
        return self.current_dist_human <= self.human_threshold

    def flee(self):
        """
        Flee from human (backwards). Consider envirnment.
        Flys in the horizontal direction backward path is blocked
        """
        vertical_correction, horizontal_correction = self.pos
        if horizontal_correction > 0:
            # obstacle in our way -> increase horizontal correction
            vertical_correction = np.sign(vertical_correction) * self.flee_dist
        else:
            horizontal_correction = -self.flee_dist
        self.pos = np.array([vertical_correction, horizontal_correction])
    
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
