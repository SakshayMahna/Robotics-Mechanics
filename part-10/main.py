from robot.robot import Robot
import numpy as np

if __name__ == "__main__":
    dh_parameters = np.array([
        [0, 0, 0, 0, "revolute"],
        [0, -np.pi/2, 0, 0, "revolute"],
        [0.1, 0, 0.2, 0, "revolute"],
        [0.1, -np.pi/2, 0.2, 0, "revolute"],
        [0, np.pi/2, 0, 0, "revolute"],
        [0, -np.pi/2, 0, 0, "revolute"]
    ])

    robot = Robot()
    robot.init_dh(dh_parameters)

    q = robot.get_configuration(0, -np.pi/3, np.pi/3, 0, 0, 0)

    robot.plot(q)