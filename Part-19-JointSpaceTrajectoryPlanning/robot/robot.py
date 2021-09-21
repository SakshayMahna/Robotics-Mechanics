"""
Robot Module

Methods
---
init_link
init_dh
transform
_transform
forward_kinematics
str, repr
"""

import numpy as np
from robot.link import Link
from tools.transformations import tr2rpy
from tools.visualize import plot_transformations, plot_points
from tools.optimizers import newton_method_vector
from robot.trajectory import (cubic_trajectory_planning, linear_trajectory_planning,
                              cubic_spline_interpolation, linear_spline_interpolation)

class Robot:
    """
    Robot Module

    ...

    Parmeters
    ---

    Methods
    ---
    init_link : Initialize robot class with links
    init_dh : Initialize robot class with dh parameters
    transform: Calculate transform till given link
    forward_kinematics: Calculate forward kinematics for the robot
    plot: Plot the robot frames

    _transform: Private function to calculate intermediate transform
    """
    def __str__(self):
        string = f"Robot Links\n"
        string += f"Link Name\ta\talpha\td\ttheta\tJoint Type\n"
        string += f"---\t\t---\t---\t---\t---\t---\n"
        for link in self._links:
            string += f"{link.name}\t\t{link.a}\t{link.alpha}\t{link.d}\t"
            string += f"{link.theta}\t{link.joint_type}\n"
        return string

    def __repr__(self):
        string = "Robot()"
        return string

    def __init__(self):
        """
        Initialize robot class

        ...

        Parameters
        ---
        None
        """
        self._links = []
        self._link_names = []

    def init_link(self, link_list):
        """
        Initialize robot with list of links

        ...

        Parameters
        ---
        link_list: List of link objects

        Returns
        ---
        None
        """
        self._links = []
        self._link_names = []

        for link in link_list:
            try:
                assert(isinstance(link, Link))
            except AssertionError:
                raise("Links should be of Link Object type")

            self._links.append(link)
            self._link_names.append(link.name)

    def init_dh(self, dh_parameters):
        """
        Initialize robot with dh parameter table

        ...

        Parameters
        ---
        dh_parameters: nx5 array of dh parameters
        [a, alpha, d, theta, joint_type]

        Returns
        ---
        None
        """
        self._links = []
        self._link_names = []

        dof = dh_parameters.shape[0]

        try:
            assert(dh_parameters.shape[1] == 5)
        except AssertionError:
            raise("DH Parameters Table not correct")

        for i in range(dof):
            name = f"link_{i}"
            a = dh_parameters[i, 0]
            alpha = dh_parameters[i, 1]
            d = dh_parameters[i, 2]
            theta = dh_parameters[i, 3]
            joint_type = dh_parameters[i, 4]

            link = Link(name = name, alpha = alpha, d = d, a = a,
                        theta = theta, joint_type = joint_type)
            
            self._links.append(link)
            self._link_names.append(name)

    def _transform(self, index, q):
        """
        Private Function to calculate intermediate transformation

        ...

        Parameters
        ---
        index : index of the link to calculate transformation for
        q : transformation parameter

        Returns
        ---
        transformation: 4x4 transformation matrix
        """
        link = self._links[index]
        transformation = link.transform(q)

        return transformation

    def transform(self, link_name, q):
        """
        Calculate transform till given link

        ...

        Parameters
        ---
        link_name: Name of the link till transformation is required
        q: List of transformation parameters

        Returns
        ---
        transformation: 4x4 transformation matrix
        """
        try:
            link_index = self._link_names.index(link_name)
        except ValueError:
            raise("Link Name not present in Robot")

        try:
            assert(q.shape == (len(self._links), ))
        except AssertionError:
            raise("Parameter q is of not correct size")

        transformation = np.eye(4)
        for i in range(0, link_index + 1):
            transformation = transformation @ self._transform(i, q[i])

        return transformation

    def forward_kinematics(self, q):
        """
        Calculate forward kinematics of the robot

        ...

        Parameters
        ---
        q : List of transformation parameters

        Returns
        ---
        transformation: 4x4 transformation matrix
        """
        end_effector = self._link_names[-1]
        transformation = self.transform(end_effector, q)

        return transformation

    def _position_function(self, q):
        """
        Private Function to compute position
        given the joint values

        ...

        Parameters
        ---
        q: List of transformation parameters

        Returns
        ---
        pos: Final position vector
        """
        transformation = self.forward_kinematics(q)
        pos = transformation[:3, 3]

        return pos

    def _position_orientation_function(self, q):
        """
        Private function to compute position and
        orientation given the joint values

        ...

        Parameters
        ---
        q: List of transformation parameters

        Returns
        ---
        transform: Final position and orientation vector
        """
        transformation = self.forward_kinematics(q)
        pos = transformation[:3, 3]

        r, p, y = tr2rpy(transformation)
        transform = np.concatenate([pos, [r, p, y]])

        return transform

    def _ik_difference_function(self, f, q, desired_config):
        """
        Helper function to solve inverse kinematics

        ...

        Parameters
        ---
        f : Position Function or Position Orientation Function
        q : List of transformation parameters
        desired_config: Desired configuration [x, y, z]
        or [x, y, z, r, p, y]

        Returns
        ---
        d: Difference between f and desired_config
        """
        d = desired_config - f(q)
        return d

    def inverse_kinematics(self, desired_config):
        """
        Function to solve inverse kinematics

        ...

        Parameters
        ---
        desired_config: Desired configuration [x, y, z]
        or [x, y, z, r, p, y]

        Returns
        ---
        q: joint values
        """
        try:
            assert(desired_config.shape == (3, ) or desired_config.shape == (6, ))
        except AssertionError:
            raise("Desired Configuration should be [x, y, z] or [x, y, z, r, p, y]")

        # Dynamically define a function to find roots for
        if desired_config.shape == (3, ):
            def ik_function(q):
                d = desired_config - self._position_function(q)
                return d
        else:
            def ik_function(q):
                d = desired_config - self._position_orientation_function(q)
                return d

        q = newton_method_vector(ik_function, np.zeros(len(self._links)))

        return q

    def calculate_jacobian(self, q):
        """
        Function to calculate and return the jacobian

        ...

        Parameters
        ---
        q : List of transformation parameters

        Returns
        ---
        jacobian: 6xdof jacobian matrix of the robot
        """
        try:
            assert(q.shape == (len(self._links), ))
        except AssertionError:
            raise("Parameter q is of not correct size")

        transformations = [np.eye(4)]
        for i in range(len(self._links)):
            transformation = transformations[i] @ self._transform(i, q[i])
            transformations.append(transformation)

        jacobian = np.zeros((6, len(self._links)))
        transl_end_effector = transformations[-1][:3, 3]

        for i in range(len(self._links)):
            rotation_vector = transformations[i + 1][:3, 2]
            transl_vector = transformations[i + 1][:3, 3]

            if self._links[i].joint_type == "revolute":
                jacobian[3:, i] = np.cross(rotation_vector, transl_end_effector - transl_vector)
                jacobian[:3, i] = rotation_vector
            else:
                jacobian[3:, i] = rotation_vector
                jacobian[:3, i] = np.zeros(3)

        return jacobian

    def joint_space_trajectory(self, positions, time_segments = None, interpolation = 'cubic'):
        """
        Function to plan joint space trajectory
        going through points

        ...

        Parameters
        ---
        positions: Array of position of robot (N x 3) or (N x 6)
        time_segments: Required if using linear interpolation with more than 2 points
        interpolation(optional): Interpolation for Trajectory, 'cubic' or 'linear'

        Returns
        ---
        q, qd, qdd : Position, Velocity and Acceleration (Dof x m)
        """
        try:
            assert(positions.shape[0] > 1)
            assert(positions.shape[1] == 3 or positions.shape[1] == 6)
        except AssertionError:
            raise("Size of positions array is not correct")

        # try:
        #     assert(time_segments.shape == (positions.shape[0] - 1, ))
        # except AssertionError:
        #     raise("Shape of Time Segments is not correct")

        try:
            assert(interpolation in ['cubic', 'linear'])
        except AssertionError:
            raise("Interpolation is not correct")

        # Convert Position to Joint Values
        q_ = np.zeros((positions.shape[0], len(self._links)))
        for i in range(positions.shape[0]):
            q_[i] = self.inverse_kinematics(positions[i])

        # Calculate Trajectory
        if positions.shape[0] == 2:
            if interpolation == 'cubic':
                zero_array = np.zeros(len(self._links))
                q, qd, qdd = cubic_trajectory_planning(q_[0], q_[1], zero_array, zero_array)
            else:
                q, qd, qdd = linear_trajectory_planning(q_[0], q_[1])
        else:
            if interpolation == 'cubic':
                q, qd, qdd = cubic_spline_interpolation(q_)
            else:
                q, qd, qdd = linear_spline_interpolation(q_, time_segments)

        return q, qd, qdd

    def get_configuration(self, *joint_values):
        """
        Function to get a configuration array

        ...

        Parameters
        ---
        *joint_values: Different Joint values

        Returns
        ---
        q: Numpy array of joint values 
        """
        try:
            assert(len(joint_values) == len(self._links))
        except AssertionError:
            raise("Number of parameters provided are not equal to DOF")

        q = np.array(joint_values)
        return q

    def plot(self, q):
        """
        Function to plot robot frames

        ...

        Parameters
        ---
        q : List of transformation parameters

        Returns
        ---
        None
        """
        try:
            assert(q.shape == (len(self._links), ))
        except AssertionError:
            raise("Parameter q is of not correct size")

        transformations = []

        transformations.append(np.eye(4))
        for i in range(len(self._links)):
            transformation = transformations[i] @ self._transform(i, q[i])
            transformations.append(transformation)

        plot_transformations(transformations)

    def plot_trajectory(self, q):
        """
        Function to plot trajectory of a robot

        ...

        Parameters
        ---
        q: List of joint values (Dof x N)

        Returns
        ---
        None
        """
        try:
            assert(q.shape[0] == len(self._links))
        except AssertionError:
            raise("Parameter q is not of correct size")

        transformations = []

        for i in range(q.shape[1]):
            transformation = self.forward_kinematics(q[:, i])
            transformations.append(transformation)

        plot_points(transformations)
        