import unittest
import numpy as np
#TODO make filter import statement work from generic path outside this directory
import filter

class TestHumanInfo(unittest.TestCase):
    def setUp(self):
        self.input = np.random.normal(0, 1, 1000)
        self.max_input = max(self.input)
        self.min_input = min(self.input)

    def test_average_filter(self):
        self.average_filter = filter.Average_Filter()
        for i in self.input:
            self.distance = i
            self.updated_distance = self.average_filter.update(self.distance)
        self.assertTrue(abs(self.updated_distance) < 1)

    def test_outlier_rejection(self):
        self.outlier_rejection= filter.Outlier_Rejection()
        for i in self.input:
            self.distance = 100*i
            self.updated_distance = self.outlier_rejection.update(self.distance)
        self.assertTrue(abs(max(self.outlier_rejection.lst)-min(self.outlier_rejection.lst))/100 < 0.9*(self.max_input -self.min_input))

if __name__ == '__main__':
    unittest.main()