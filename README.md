# robotiq2f — Robotiq 2F-85 Gripper Driver for Python

**A pure-Python, ROS-free driver for the Robotiq 2F-85 (2F85) two-finger adaptive gripper.**

`robotiq2f` is a Python package that provides a driver for interacting with the **Robotiq 2F-85** gripper over Modbus RTU. It includes functionalities for finding the gripper's serial port, activating/deactivating the gripper, reading its status, and controlling its movements.

```shell
pip install robotiq2f
```

```python
from robotiq2f import Robotiq2F85

gripper = Robotiq2F85(serial_number="DAK1RLYZ")  # Robotiq 2F-85 over USB/RS-485
gripper.go_to(opening=85, speed=150, force=235)  # fully open the 2F-85
```

This is a fork of [PhilNad/2f85-python-driver](https://github.com/PhilNad/2f85-python-driver).

*Keywords: Robotiq 2F-85, Robotiq 2F85, 2F-85 gripper, Robotiq gripper Python driver, Robotiq 2F, two-finger adaptive gripper, Modbus RTU, robotics.*

## Features
Compared to existing packages, this one is different in the following ways:
- It is independant from ROS, you only need to import the Python module to use the gripper.
- It is a pure Python implementation, no need to compile any C++ code.
- It detects the gripper's serial port based on its serial number.
- It provides a function to account for the motion of the fingertips when planning grasps.

## Installation

Requires Python 3.10 or newer.

### Pip Installation (Recommended)
```shell
pip install robotiq2f
```

### Local Installation
```shell
git clone https://github.com/RobotControlStack/robotiq2f.git
cd robotiq2f
pip install -ve .
```

To allow access to the serial connection, you will also need to add make your user a member of the `dialout` group with
```bash
sudo adduser $USER dialout
```
and logout/login to refresh the membership.

## Usage — Controlling a Robotiq 2F-85 from Python

```python
from robotiq2f import Robotiq2F85

# Initialize the driver with the gripper's serial number
gripper = Robotiq2F85(serial_number="DAK1RLYZ")

# Reset the gripper
gripper.reset()

# Move the gripper to fully open position (opening = 85 mm)
# The motion is done at 150 mm/s with a force of up to 235 Newtons.
gripper.go_to(opening=85, speed=150, force=235)

# Get the current gripper opening
print(gripper.opening)
```

## Feature Desrciption

### Getting Serial Port from Serial Number

In contrast to most existing packages, this driver finds the serial port of a device with a specific serial number so you dont need to worry about the port number changing when you plug the device in. This is done by iterating over all `/dev/ttyUSB*` devices and finding the one with the given serial number.


#### Accounting for the motion of the fingertips
Ever had the gripper collide with the table when closing onto a small object? Not anymore! This driver provides a function that can be used to account for the motion of the fingertips when planning grasps.

Due to how the gripper is designed, the fingertips move away from the gripper's base frame when the gripper closes. This can be inconvenient when the Tool Center Point (TCP) is on the fingertips and needs to be precisely positioned to accurately grasp small objects. The `tcp_Z_offset` function calculates the distance along the gripper Z+ axis that the TCP will move when the gripper goes from its current opening to the desired opening. This function is particularly useful when planning robotic movements for grasping objects. Knowing the offset allows the robot to compensate for the gripper's movement and position the TCP precisely at the desired location before grasping an object of known thickness.

##### Simple Geometrical Model
![image](https://github.com/PhilNad/2f85-python-driver/assets/10478385/144962e3-0422-4eec-9504-95bdf656d31f)

##### Derivation
Let
```math
d = \frac{opening}{2} + pad\_thickness
```
be defined in millimeters, then
```math
\theta =
\begin{cases}
d \lt 12.7 :&   -\arcsin\left(\frac{12.7-d}{57.15}\right)\\
d \geq 12.7 :& \arcsin\left(\frac{d-12.7}{57.15}\right)
\end{cases}
```
and
```math
\begin{align}
TCP_z &= 61.308 + 57.15\cos(\theta) + 7 + 38/2\\
&= 87.308 + 57.15\cos(\theta)
\end{align}
```
is the distance along the Z+ axis of the base frame between the base frame and the TCP frame.

Noting that
```math
\cos(\arcsin(x)) = \cos(-\arcsin(x)) = \sqrt{1-x^2}
```
we get
```math
TCP_Z = 
\begin{cases}
d \lt 12.7 :&  87.308 + 57.15\sqrt{1-\left(\frac{12.7-d}{57.15}\right)^2}\\
d \geq 12.7 :& 87.308 + 57.15\sqrt{1-\left(\frac{d-12.7}{57.15}\right)^2}
\end{cases}
```
that provides an expression for the position of the TCP frame as a function of the opening and pad thickness (both in millimeters).


#### Useful Gripper Properties

- `opening: float`: Current gripper opening in millimeters.

- `current: float`: Current in milliamps.

- `is_activated: bool`: Check if the gripper is activated.

- `is_moving: bool`: Check if the gripper is currently moving.

- `object_detected: bool`: Check if an object is detected by the gripper.

- `in_fault: bool`: Check if the gripper is in a fault state.

## Development

Install the development dependencies and use the `Makefile` targets:

```shell
pip install -ve . && pip install --group dev
make format       # isort + black
make checkformat  # verify formatting without changing files
make lint         # ruff + mypy
make test         # pytest
```

## Limitations
Currently assumes that Linux is used as the operating system, which could be easily extended to support other operating systems.

Currently only the **Robotiq 2F-85** is supported, via the `Robotiq2F85` class. It inherits from an (for now empty) `Robotiq2F` base class that is intended to hold the behaviour shared across the Robotiq 2F family, once the model-specific calibration constants for other members (e.g. the 2F-140) are parameterized.

## License

This project is licensed under the [MIT License](LICENSE).


# TODO
- remove the sleep
- add async thread that constantly polls
- support the 2F-140 by parameterizing the model-specific constants
