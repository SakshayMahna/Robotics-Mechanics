"""
Unit Testing Rigid Body Transformations

MATLAB
...

Robotics Toolbox
---
trvec2tform
eul2tform
tform2eul
"""
# To import for testing
import sys
sys.path.insert(0, './../')

import unittest
from test_transformations import *
from test_links import * 
from test_robot import *                             

if __name__ == "__main__":
    unittest.main()