from tools.transformations import *
from tools.visualize import *
import numpy as np

if __name__ == "__main__":
    # Translation Function
    matrix_1 = transl(1, 2, 3)
    print("transl function")
    print(matrix_1)
    print()

    # Rotation about X Axis
    matrix_2 = rotx(np.pi / 2)
    print("rotx function")
    print(matrix_2)
    print()

    # Rotation about Y Axis
    matrix_3 = roty(np.pi / 2)
    print("roty function")
    print(matrix_3)
    print()

    # Rotation about Z Axis
    matrix_4 = rotz(np.pi / 2)
    print("rotz function")
    print(matrix_4)
    print()

    # Convert roll, pitch and yaw angle to transformation matrix
    matrix_5 = rpy2tr(np.pi / 2, np.pi / 2, np.pi / 2)
    print("rpy2tr function")
    print(matrix_5)
    print()

    # Convert transformation matrix to roll, pitch and yaw angle
    r, p, y = tr2rpy(matrix_5)
    print("tr2rpy function")
    print(r, p, y)

    # Visualize a single transformation
    transformation = rpy2tr(np.pi/3, 0, 0)
    transformation = transformation + transl(0.1, 0.1, 0.1)
    plot_transformation(transformation)