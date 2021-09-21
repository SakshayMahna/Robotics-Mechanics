"""
Unit Testing Rigid Body Transformations
"""

import unittest
import numpy as np
from tools.transformations import *

class TestTransformations(unittest.TestCase):

    def test_transl(self):
        x = 1
        y = 2
        z = 3

        transformation = transl(x, y, z)
        actual_transformation = np.array([[1, 0, 0, 1],
                                          [0, 1, 0, 2],
                                          [0, 0, 1, 3],
                                          [0, 0, 0, 1]])

        np.testing.assert_almost_equal(actual_transformation, transformation)

    def test_rpy2tr(self):
        roll = np.pi / 3
        pitch = np.pi / 4
        yaw = np.pi / 5

        transformation = rpy2tr(roll, pitch, yaw)
        actual_transformation = np.array([[0.5721, 0.2015, 0.7951, 0],
                                          [0.4156, 0.7645, -0.4928, 0],
                                          [-0.7071, 0.6124, 0.3536, 0],
                                          [0, 0, 0, 1.0000]])

        np.testing.assert_almost_equal(actual_transformation, transformation, 
                                       decimal = 4)

    def test_tr2rpy(self):
        transformation = np.array([[0.5721, 0.2015, 0.7951, 0],
                                   [0.4156, 0.7645, -0.4928, 0],
                                   [-0.7071, 0.6124, 0.3536, 0],
                                   [0, 0, 0, 1.0000]])
        
        r, p, y = tr2rpy(transformation)
        angles = np.array([r, p, y])
        actual_angles = np.array([np.pi / 3, np.pi / 4, np.pi / 5])

        np.testing.assert_almost_equal(actual_angles, angles, decimal = 4)

    def test_rot(self):
        rotation_x = rotx(np.pi/3)
        rotation_y = roty(np.pi/3)
        rotation_z = rotz(np.pi/3)

        actual_rotation_x = np.array([[1.0000, 0, 0],
                                      [0, 0.5000, -0.8660],
                                      [0, 0.8660, 0.5000]])
        actual_rotation_y = np.array([[0.5000, 0, 0.8660],
                                      [0, 1.0000, 0],
                                      [-0.8660, 0, 0.5000]])   
        actual_rotation_z = np.array([[0.5000, -0.8660, 0],
                                      [0.8660, 0.5000, 0],
                                      [0, 0, 1.0000]])

        np.testing.assert_almost_equal(actual_rotation_z, rotation_z, decimal = 4)
        np.testing.assert_almost_equal(actual_rotation_y, rotation_y, decimal = 4)
        np.testing.assert_almost_equal(actual_rotation_x, rotation_x, decimal = 4) 
