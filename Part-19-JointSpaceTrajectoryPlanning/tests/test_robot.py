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

        q = robot.get_configuration(0.1, 0.2, 0.3)

        transformation = robot.transform("link_1", q)
        actual_transformation = np.array([[0.9834, -0.1570, 0.0908, 3.0808],
                                          [-0.1436, -0.9799, -0.1387, -1.7137],
                                          [0.1107, 0.1233, -0.9862, 0.2624],
                                          [0, 0, 0, 1.0000]])
        
        np.testing.assert_almost_equal(actual_transformation, transformation, decimal = 4)

    def test_forward_kinematics(self):
        robot = Robot()
        robot.init_dh(np.array([
            [1, 1, 2, 2, 'revolute'],
            [2, 2, 1, 1, 'revolute'],
            [1, 1, 1, 1, 'prismatic']
        ]))

        q = robot.get_configuration(0.1, 0.2, 0.3)

        transformation = robot.forward_kinematics(q)
        actual_transformation = np.array([[0.5243, -0.8321, 0.1811, 4.1185],
                                          [-0.6213, -0.2283, 0.7496, -1.6324],
                                          [-0.5824, -0.5055, -0.6366, 0.1822],
                                          [0, 0, 0, 1.0000]])
        
        np.testing.assert_almost_equal(actual_transformation, transformation, decimal = 4)

    def test_inverse_kinematics(self):
        robot = Robot()
        robot.init_dh(np.array([
            [1, 1, 2, 2, 'revolute'],
            [2, 2, 1, 1, 'revolute'],
            [1, 1, 1, 1, 'prismatic']
        ]))

        q = robot.get_configuration(0.1, 0.2, 0.3)
        transformation = robot.forward_kinematics(q)
        pos = transformation[:3, 3]
        r, p, y = tr2rpy(transformation)

        value = robot.inverse_kinematics(pos)
        np.testing.assert_almost_equal(value, q, decimal = 4)

        value = robot.inverse_kinematics(np.concatenate([pos, [r, p, y]]))
        np.testing.assert_almost_equal(value, q, decimal = 4)

    def test_jacobian(self):
        robot = Robot()
        robot.init_dh(np.array([
            [1, 1, 2, 2, 'revolute'],
            [2, 2, 1, 1, 'revolute'],
            [1, 1, 1, 1, 'prismatic']
        ]))

        q = robot.get_configuration(0.1, 0.2, 0.3)
        jacobian = robot.calculate_jacobian(q)
        actual_jacobian = np.array([[-0.0000, 0.0908, 0],
                                   [-0.8415, -0.1387, 0],
                                   [0.5403, -0.9862, 0],
                                   [0.7287, 0.0913, 0.1811],
                                   [1.6850, -1.0161, 0.7496],
                                   [2.6242, 0.1513, -0.6366]])
        
        np.testing.assert_almost_equal(actual_jacobian, jacobian, decimal = 4)

