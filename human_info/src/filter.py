class Filter:
    """
    Filter apllied to calculated distances from human pose estimation.
    First iteration calculates average distance over samplesize n = 10
    """
    def __init__(self):
        self.lst = []
        # samplesize n = 10
        self.n = 10

    def update(self, new_dist):
        if len(self.lst) < self.n:
            lst.append(new_dist)
        else:
            lst.remove(self.lst[0])
            lst.append(new_dist)
            average_distance = sum(self.list)/len(self.list)
        return average_distance
