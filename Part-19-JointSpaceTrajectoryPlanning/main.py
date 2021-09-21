from robot.robot import Robot
from tools.visualize import plot_joint_trajectory
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

    q1 = robot.get_configuration(0.1, 0, 0, 0, 0, 0)
    q2 = robot.get_configuration(0.2, 0, 0, 0, 0, 0)
    q3 = robot.get_configuration(0.3, 0, 0, 0, 0, 0)
    q4 = robot.get_configuration(0.4, 0, 0, 0, 0, 0)
    q5 = robot.get_configuration(0.5, 0, 0, 0, 0, 0)

    t1 = robot.forward_kinematics(q1)
    t2 = robot.forward_kinematics(q2)
    t3 = robot.forward_kinematics(q3)
    t4 = robot.forward_kinematics(q4)
    t5 = robot.forward_kinematics(q5)

    t1 = t1[:3, 3]
    t2 = t2[:3, 3]
    t3 = t3[:3, 3]
    t4 = t4[:3, 3]
    t5 = t5[:3, 3]

    print(t1)
    print(t2)
    print(t3)
    print(t4)
    print(t5)

    positions = np.array([t1, t2, t3, t4, t5])
    q, qd, qdd = robot.joint_space_trajectory(positions)
    robot.plot_trajectory(q)
