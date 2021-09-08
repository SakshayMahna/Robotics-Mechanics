"""
Unit Testing RObot Module
"""

import unittest
from robot.robot import Robot
from robot.link import Link
from tools.transformations import tr2rpy
import numpy as np

class TestRobot(unittest.TestCase):

    def test_initialization(self):
        links = [
            Link(name = 'link_0', alpha = 1, a = 1, d = 2, theta = 2),
            Link(name = 'link_1', alpha = 2, a = 2, d = 1, theta = 1),
            Link(name = 'link_2', alpha = 1, a = 1, d = 1, theta = 1, joint_type='prismatic')
        ]

        robot_link = Robot()
        robot_link.init_link(links)

        robot_dh = Robot()
        robot_dh.init_dh(np.array([
            [1, 1, 2, 2, 'revolute'],
            [2, 2, 1, 1, 'revolute'],
            [1, 1, 1, 1, 'prismatic']
        ]))

        self.assertEqual(str(robot_dh), str(robot_link))

    def test_transform(self):
        robot = Robot()
        robot.init_dh(np.array([
            [1, 1, 2, 2, 'revolute'],
            [2, 2, 1, 1, 'revolute'],
            [1, 1, 1, 1, 'prismatic']
        ]))

        q = robot.get_configuration(0.1, 0.1, 0.1)

        transformation = robot.transform("link_1", q)
        actual_transformation = np.array([[0.9942, -0.0580, 0.0908, 3.0808],
                                          [-0.0451, -0.9893, -0.1387, -1.7137],
                                          [0.0979, 0.1338, -0.9862, 0.2624],
                                          [0, 0, 0, 1.0000]])
        
        np.testing.assert_almost_equal(actual_transformation, transformation, decimal = 4)

    def test_forward_kinematics(self):
        robot = Robot()
        robot.init_dh(np.array([
            [1, 1, 2, 2, 'revolute'],
            [2, 2, 1, 1, 'revolute'],
            [1, 1, 1, 1, 'prismatic']
        ]))

        q = robot.get_configuration(0.1, 0.1, 0.1)

        transformation = robot.forward_kinematics(q)
        actual_transformation = np.array([[0.5751, -0.8122, 0.0979, 4.0848],
                                          [-0.5723, -0.3139, 0.7576, -1.6830],
                                          [-0.5846, -0.4916, -0.6454, 0.2958],
                                          [0, 0, 0, 1.0000]])
        
        np.testing.assert_almost_equal(actual_transformation, transformation, decimal = 4)

    def test_inverse_kinematics(self):
        robot = Robot()
        robot.init_dh(np.array([
            [1, 1, 2, 2, 'revolute'],
            [2, 2, 1, 1, 'revolute'],
            [1, 1, 1, 1, 'prismatic']
        ]))

        q = robot.get_configuration(0.1, 0.1, 0.1)
        transformation = robot.forward_kinematics(q)
        pos = transformation[:3, 3]
        r, p, y = tr2rpy(transformation)

        value = robot.inverse_kinematics(pos)
        np.testing.assert_almost_equal(value, q, decimal = 4)

        value = robot.inverse_kinematics(np.concatenate([pos, [r, p, y]]))
        np.testing.assert_almost_equal(value, q, decimal = 4)

