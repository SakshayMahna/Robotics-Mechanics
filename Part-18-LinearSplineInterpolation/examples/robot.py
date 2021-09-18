from robot.robot import Robot
from robot.link import Link
from tools.transformations import tr2rpy
import numpy as np

if __name__ == "__main__":
    # Initializing Links
    links = [
        Link(name = 'link_0', alpha = 1, a = 1, d = 2, theta = 2),
        Link(name = 'link_1', alpha = 2, a = 2, d = 1, theta = 1),
        Link(name = 'link_2', alpha = 1, a = 1, d = 1, theta = 1, joint_type='prismatic')
    ]

    # Initializing a Robot using Links
    robot_link = Robot()
    robot_link.init_link(links)
    print(robot_link)
    print()

    # Initializing a Robot using DH Parameters
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
    print(robot)
    print()

    # Calculating Transformation for an arbitrary link
    q = robot.get_configuration(0.1, 0.1, 0.1, 0.1, 0.1, 0.1)
    transformation = robot.transform("link_1", q)
    print(transformation)
    print()

    # Calculating Forward Kinematics of end effector
    q = robot.get_configuration(0.1, 0.1, 0.1, 0.1, 0.1, 0.1)
    transformation = robot.forward_kinematics(q)
    print(transformation)
    print()

    # Plotting the robot on a given configuration
    q = robot.get_configuration(0, -np.pi/3, np.pi/3, 0, 0, 0)
    robot.plot(q)

    # Calculate Inverse Kinematics for X, Y and Z
    # The X, Y and Z are calculated in turn from forward kinematics
    q = robot.get_configuration(0.1, 0.1, 0.1, 0.1, 0.1, 0.1)
    transformation = robot.forward_kinematics(q)
    pos = transformation[:3, 3]
    value = robot.inverse_kinematics(pos)
    print(value)
    print()

    # Calculate Inverse Kinematics for X, Y, Z, Roll, Pitch and Yaw
    r, p, y = tr2rpy(transformation)
    value = robot.inverse_kinematics(np.concatenate([pos, [r, p, y]]))
    print(value)
    print()

    # Calculate Jacobian of the robot
    q = robot.get_configuration(0.1, 0.1, 0.1, 0.1, 0.1, 0.1)
    jacobian = robot.calculate_jacobian(q)
    print(jacobian)
    print()