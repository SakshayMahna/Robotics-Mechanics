"""
Unit Testing Optimizers
"""


import unittest
import numpy as np
from tools.optimizers import *

class TestNewton(unittest.TestCase):
    def function_test_1(self, x):
        # Derivate: 2x + 3
        # At x = 1, d = 5
        value = x ** 2 + 3 * x + 2
        return value

    def function_test_2(self, x):
        # Derivate:  exp(x) - 2xexp(x^2)
        # At x = 2, d = -211.003544034
        value = np.exp(x) - np.exp(x ** 2) + 1
        return value

    def function_test_3(self, x):
        # Jacobian: [[df1, 0], [0, df2]]
        # At x = [1, 2], df1 = 5, df2 = -211.003544034
        value = np.array([self.function_test_1(x[0]), self.function_test_2(x[1])])
        return value

    def function_test_4(self, x):
        # Solution: [1, 0]
        value = np.array([
            x[0] ** 3 + x[1] - 1,
            x[1] ** 3 - x[0] + 1,
            x[0] ** 2 - 3 * x[0] + 2,
            x[1] ** 2
        ])
        return value

    def test_derivative(self):
        value = derivative(self.function_test_1, 1)
        self.assertAlmostEqual(value, 5, 4)

        value = derivative(self.function_test_2, 2, 1e-9)
        self.assertAlmostEqual(value, -211.00354, 4)

    def test_jacobian(self):
        value = jacobian(self.function_test_3, np.array([1, 2]), 1e-8)
        np.testing.assert_array_almost_equal(value, [[5, 0], [0, -211.00354]], 4)

    def test_newton(self):
        value = newton_method(self.function_test_1, x_init = -5)
        self.assertAlmostEqual(value, -2, 4)

        value = newton_method(self.function_test_2)
        self.assertAlmostEqual(value, -0.6485, 4)

    def test_newton_vector(self):
        value = newton_method_vector(self.function_test_3, np.array([-5, 0]))
        np.testing.assert_array_almost_equal(value, [-2, -0.6485], 4)

        value = newton_method_vector(self.function_test_4, np.array([-5, 0]))
        np.testing.assert_array_almost_equal(value, [1, 0], 4)