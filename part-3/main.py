from visualize import *
from transformations import *
import numpy as np

if __name__ == "__main__":
    transformation = rpy2tr(np.pi/3, 0, 0)
    transformation = transformation + transl(0.1, 0.1, 0.1)
    plot_transformation(transformation)