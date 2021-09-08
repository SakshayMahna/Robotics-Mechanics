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

    def test_derivative(self):
        value = derivative(self.function_test_1, 1)
        self.assertAlmostEqual(value, 5, 4)

        value = derivative(self.function_test_2, 2, 1e-9)
        self.assertAlmostEqual(value, -211.00354, 4)

    def test_newton(self):
        value = newton_method(self.function_test_1, x_init = -5)
        self.assertAlmostEqual(value, -2, 4)

        value = newton_method(self.function_test_2)
        self.assertAlmostEqual(value, -0.6485, 4)