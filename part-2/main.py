from transformations import *
import numpy as np

if __name__ == "__main__":
    matrix_1 = transl(1, 2, 3)
    print("transl function")
    print(matrix_1)
    print()

    matrix_2 = rotx(np.pi / 2)
    print("rotx function")
    print(matrix_2)
    print()

    matrix_3 = roty(np.pi / 2)
    print("roty function")
    print(matrix_3)
    print()

    matrix_4 = rotz(np.pi / 2)
    print("rotz function")
    print(matrix_4)
    print()

    matrix_5 = rpy2tr(np.pi / 2, np.pi / 2, np.pi / 2)
    print("rpy2tr function")
    print(matrix_5)
    print()

    r, p, y = tr2rpy(matrix_5)
    print("tr2rpy function")
    print(r, p, y)