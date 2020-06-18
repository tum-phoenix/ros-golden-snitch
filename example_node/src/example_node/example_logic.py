#!/usr/bin/env python3


class Example_logic_class:
    def __init__(self, example_parameter):
        self.example = example_parameter
        self.static_variable = 0


    def update(self, stamp, seq):
        """
        This function does some meaningless logic just to demonstrate how to make ros nodes.
        @param stamp: Just an example
        @param seq: Just another example
        """
        self.static_variable = _update_map(self.static_variable, self.example, seq, stamp)
        return self.static_variable

def _update_map(static,variable, parameter1, parameetr2):
    # TODO: Create the logic of the node
    if parameetr2 > 1000000:
        return static + variable + parameter1
    else:
        return static + variable