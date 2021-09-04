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
from tools.visualize import plot_transformations
from tools.optimizers import newton_method_vector

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