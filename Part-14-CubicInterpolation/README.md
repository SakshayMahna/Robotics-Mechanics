# Robot Mechanics
Python Library for Robot Kinematics, Dynamics, Motion Planning and Control. These tools can be used to design, simulate and test different robotics algorithms.

## Prerequisites
The library has been developed on Windows 10 machine and Python 3.8.8. The library can be run on all platforms running Python.

## Installation

Without Virtual Environment

```bash
cd Robotics-Mechanics
pip install -r requirements.txt
```

With Virtual Environment

```bash
cd Robotics-Mechanics
virtualenv mechanics
./mechanics/Scripts/activate
pip install -r requirements.txt
```

## Examples

- Working with Transformation Matrices and their Visualization. [Code](examples\transformation.py)

- Working with Robot - Initialization, Forward Kinematics, Inverse Kinematics, Jacobian of the Robot and Visualization. [Code](examples\robot.py)

## Tests
Unit Tests for all the functionalities are provided.

```bash
cd tests
python test.py
```