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

def partial_derivative(f, x, i, epsilon = 1e-10):
    """
    Calculate partial derivative of f at xi

    ...
    
    Parameters
    ---
    f: Vector function
    x: Vector input
    i: index of x
    epsilon(optional): Adjustable precision

    Returns
    ---
    value: Partial Derivative of f at xi
    """
    x_ = np.copy(x).astype(np.float64)
    x_[i] = x_[i] + epsilon
    value = (f(x_) - f(x)) / epsilon

    return value

def jacobian(f, x, epsilon = 1e-10):
    """
    Calculate Jacobian of f wrt x

    ...

    Parameters
    ---
    f: Vector function
    x: Vector input
    epsilon(optional): Adjustable precision

    Returns
    ---
    value: Jacobian of f at x
    """
    f_ = f(x)
    value = np.zeros((len(f_), len(x)))
    
    for i in range(len(x)):
        f_ = partial_derivative(f, x, i, epsilon)
        value[:,i] = f_

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

def newton_method_vector(f, x_init, epsilon = 1e-10, max_iterations = 1000):
    """
    Newton Raphson Optimizer for Vector Functions

    ...

    Parameters
    ---
    f: Function to calculate root for
    x_init : initial value of x
    epsilon(optional): Adjustable precision
    max_iterations(optional): Number of iterations to run

    Returns
    ---
    x: Value of root
    """
    prev_value = x_init + 2 * epsilon
    value = x_init

    iterations = 0
    while np.any(np.abs(prev_value - value) > epsilon):
        prev_value = value

        j = jacobian(f, value)
        value = value - np.dot(np.linalg.pinv(j), f(value))

        iterations += 1

    print(f"Newton Method converged in {iterations} iterations")

    return value