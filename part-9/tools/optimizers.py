"""
Optimizer Module
"""

import numpy as np

def derivative(f, x, epsilon = 1e-10):
    """
    Calculate derivative of f at x

    ...

    Parameters
    ---
    f : Function to calculate derivative for
    x : Value at which to calculate derivative
    epsilon(optional): Adjustable precision

    Returns
    ---
    value: Derivative of f at x
    """

    x_ = x + epsilon
    value = (f(x_) - f(x)) / epsilon

    return value

def newton_method(f, x_init = 0, epsilon = 1e-10):
    """
    Newton Raphson Optimizer

    ...

    Parameters
    ---
    f: Function to calculate root for
    x_init(optional) : initial value of x
    epsilon(optional): Adjustable precision

    Returns
    ---
    x: Value of root
    """
    prev_value = x_init + 2 * epsilon
    value = x_init

    iterations = 0
    while abs(prev_value - value) > epsilon:
        prev_value = value

        f_dash = derivative(f, value)
        value = value - f(value) / f_dash

        iterations += 1

    print(f"Newton Method converged in {iterations} iterations")

    return value

    