#!/usr/bin/env python3
import unittest
import sys

from local_map.map_logic import Mapper

PKG='local_map'


class TestLocalMapper(unittest.TestCase):

    def setUp(self) -> None:
        pass

    def test_import(self):
        mapper = Mapper()
        mapper.update([],[])
        self.assertTrue(True,"Shoul be true if the import is working")




if __name__ == '__main__':
    import rosunit
    import rostest
    #rosunit.unitrun(PKG, 'test_fsm_code_level', TestFsm)
    rostest.rosrun(PKG, 'test_map_logic', TestLocalMapper, sys.argv)