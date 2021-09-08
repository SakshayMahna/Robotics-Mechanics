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
from link import Link

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

    _transform: Private function to calculate intermediate transform
    """
    def __str__(self):
        string = f"Robot Links\n"
        string += f"Link Name\ta\talpha\td\ttheta\tJoint Type\n"
        string += f"---\t---\t---\t---\t---\t---\n"
        for link in self._links:
            string += f"{link.name}\t{link.a}\t{link.alpha}\t{link.d}\t"
            string += f"{link.theta}\t{link.joint_type}\n"
        return ""

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
            assert(dh_parameters.shape[1] == 4)
        except:
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
        if index == 0:
            transformation = np.eye(4)
        else:
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
            assert(q.shape == (len(self._links),))
        except AssertionError:
            raise("Parameter q is of not correct size")

        transformation = self._transform(0, 0)
        for i in range(1, link_index + 1):
            transformation = transformation @ self._transform(i, q[i-1])

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