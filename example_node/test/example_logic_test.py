#!/usr/bin/env python3
import unittest
import sys

from example_node.example_logic import Example_logic_class

PKG='example_node'


class TestExampleNode(unittest.TestCase):

    def setUp(self) -> None:
        pass

    def test_example(self):
        # TODO: Write your unittests here
        pass




if __name__ == '__main__':
    import rosunit
    import rostest
    # TODO: Write your package name here:
    rostest.rosrun(PKG, 'example_node_logic', TestExampleNode, sys.argv)