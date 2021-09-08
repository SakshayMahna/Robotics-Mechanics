"""
Robot Links

- Kinematic Parameters
- Inertial Parameters

Params
- name
- alpha, a, d, theta
- inertia_matrix, cog, mass

Functions
- transform
- str, repr
"""
import numpy as np

class Link:
    """
    Robot Link Class

    Parameters
    ---
    name : name of the link
    alpha : alpha value of the link
    a : a value of the link
    d : d value of the link
    theta : theta value of the link
    joint_type : joint type of the link
    inertia_matrix : 3x3 inertia matrix
    cog: center of gravity in base frame
    mass : mass of the link

    Methods
    ---
    transform(q) : get transformation of the link
    """
    def __init__(self, name = '', alpha = 0, a = 0, d = 0, theta = 0,
                 joint_type = 'revolute', inertia_matrix = None, cog = None,
                 mass = 0):
        """
        Initialization Function

        ...

        Parameters
        ---
        Refer class definition
        """
        self.name = name
        self.alpha = alpha
        self.a = a
        self.d = d
        self.theta = theta
        self.joint_type = joint_type
        self.inertia_matrix = inertia_matrix
        self.cog = cog
        self.mass = mass

    def transform(self, q):
        """
        Function to generate Transformation matrix

        ...

        Parameters
        ---
        q : Parameter for the link (theta: revolute, d: prismatic)

        Returns
        ---
        transformation: 4x4 transformation matrix
        """
        alpha = self._alpha
        a = self._a

        # Determine parameter for the joint
        if self._joint_type == 'revolute':
            theta = q
            d = self._d
        else:
            theta = self._theta
            d = q

        transformation = np.array([
            [np.cos(theta), - np.sin(theta), 0, a],
            [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), - np.sin(alpha), - np.sin(alpha) * d],
            [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), np.cos(alpha) * d],
            [0, 0, 0, 1]
        ])

        return transformation

    # Getters and Setters
    @property
    def name(self):
        """ Name of the link """
        return self._name

    @name.setter
    def name(self, name):
        """ Name of the link """
        self._name = name

    @property
    def alpha(self):
        """ Alpha value of the link """
        return self._alpha

    @alpha.setter
    def alpha(self, alpha):
        """ Alpha value of the link """
        self._alpha = alpha

    @property
    def a(self):
        """ a value of the link """
        return self._a

    @a.setter
    def a(self, a):
        """ a value of the link """
        self._a = a

    @property
    def d(self):
        """ d value of the link """
        return self._d

    @d.setter
    def d(self, d):
        """ d value of the link """
        self._d = d

    @property
    def theta(self):
        """ Theta value of the link """
        return self._theta

    @theta.setter
    def theta(self, theta):
        """ Theta value of the link """
        self._theta = theta

    @property
    def joint_type(self):
        """ Joint Type of the link """
        return self._joint_type

    @joint_type.setter
    def joint_type(self, joint_type):
        """ Joint Type of the link """
        joint_types = ['revolute', 'prismatic']
        try:
            assert(joint_type in joint_types)
            self._joint_type = joint_type
        except AssertionError:
            raise(f"Joint type {joint_type} not defined!")

    @property
    def inertia_matrix(self):
        """ Inertia Matrix of the link """
        return self._inertia_matrix

    @inertia_matrix.setter
    def inertia_matrix(self, inertia_matrix):
        """ Inertia Matrix value of the link """
        try:
            assert(inertia_matrix.shape == (3, 3))
            self._inertia_matrix = inertia_matrix
        except AssertionError:
            raise("Shape of inertia matrix is incorrect")

    
    @property
    def cog(self):
        """ COG of the link """
        return self._cog

    @cog.setter
    def cog(self, cog):
        """ COG of the link """
        try:
            assert(cog.shape == 3)
            self._cog = cog
        except AssertionError:
            raise("Shape of COG vector is incorrect")

    @property
    def mass(self):
        """ Mass of the link """
        return self._mass

    @mass.setter
    def mass(self, mass):
        """ Mass of the link """
        self._mass = mass