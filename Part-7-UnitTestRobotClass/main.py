from link import Link
from robot import Robot

if __name__ == "__main__":
    links = [Link(), Link(), Link()]
    robot = Robot()
    robot.init_link(links)
    print(robot)