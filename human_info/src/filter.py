"""
Contains filters used in human pose estimation

"""
import copy


def normsq(x, y):
    return (x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2


class Keypoint_Filter:
    """
    Filter apllied to all keypoints in pixel coordinates
    """

    def __init__(self, k, num_of_continuity_frames, FEATURES, OUTLIER_THRESHOLD):
        """
        @param k: (0, 1) The filter constant. Small value -> Slow dynamics, Large value -> More noise.
        @param max_dist_pixels: (0, ->) The threshold for deviation from previous estimate for discarding is as an outlier.
        """
        self.k = k
        self.num_of_continuity_frames = num_of_continuity_frames
        self.FEATURES = FEATURES
        self.OUTLIER_THRESHOLD = OUTLIER_THRESHOLD
        self.keypoints = None

    def update(self, keypoints):
        if self.keypoints is None:
            self.keypoints = copy.deepcopy(keypoints)
            self.repetitions_left = {}
            for k in keypoints.keys():
                self.repetitions_left[k] = self.num_of_continuity_frames
            for f in self.FEATURES:
                if self.repetitions_left[f]  != self.num_of_continuity_frames:
                    self.repetitions_left[f] = 0

        for key in self.FEATURES:
            if key in keypoints:
                if key not in self.keypoints:
                    self.keypoints[key] = keypoints[key]
                if normsq(self.keypoints[key].yx, keypoints[key].yx) > self.OUTLIER_THRESHOLD:
                    # This is probably an outlier
                    if self.repetitions_left[key] >0:
                        self.repetitions_left[key] -=1
                    else:
                        self.keypoints.pop(key, None)

                self.keypoints[key].yx[0] = keypoints[key].yx[0] * self.k + self.keypoints[key].yx[0] * (1 - self.k)
                self.keypoints[key].yx[1] = keypoints[key].yx[1] * self.k + self.keypoints[key].yx[1] * (1 - self.k)
                self.keypoints[key].score = keypoints[key].score
                self.repetitions_left[key] = self.num_of_continuity_frames
            else:
                # Track is lost
                if self.repetitions_left[key] > 0:
                    self.repetitions_left[key] -= 1
                else:
                    self.keypoints.pop(key, None)

        return copy.deepcopy(self.keypoints)


class Average_Filter:
    """
    Filter apllied to calculated distances from human pose estimation.
    First iteration calculates average distance over samplesize n = 10
    only last 10 distances are considered
    """

    def __init__(self):
        self.lst = []
        # samplesize n = 10
        self.n = 10

    def update(self, new_dist):
        # global average_distance
        self.lst.append(new_dist)
        average_distance = sum(self.lst) / len(self.lst)
        if len(self.lst) > self.n:
            self.lst.remove(self.lst[0])
        return average_distance


class Outlier_Rejection:
    """
    outlier rejection of extremes in calculated distance from human pose estimation
    Discards all outputs that are further away than
        max_difference
    compared to previous timestep.
    """

    def __init__(self):
        # maximal difference allowed between two timesteps
        self.max_difference = 50
        self.lst = {}

    def update(self, new_dist, keypoint_type):

        # if there is no entry, for that feature, in the dictionary, this is the first pose with that feature
        if not keypoint_type in self.lst:
            self.lst[keypoint_type] = new_dist
            return self.lst[keypoint_type]
        # otherwise check the difference between this new pose distance, and the old one; if it's bigger that max_difference, this is an outlier, and should be ignored
        elif abs(new_dist - self.lst[keypoint_type]) > self.max_difference:
            return -1
        # otherwise it's within the acceptable range, and we return the value
        return self.lst[keypoint_type]
