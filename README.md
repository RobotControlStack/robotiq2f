# robotiq2f — Robotiq 2F-85 Gripper Driver for Python

A pure-Python, ROS-free driver for the **Robotiq 2F-85** (2F85) two-finger adaptive gripper, talking Modbus RTU over USB. It finds the gripper by serial number, activates it, reads its status, and controls its motion.

Fork of [PhilNad/2f85-python-driver](https://github.com/PhilNad/2f85-python-driver).

- No ROS, no C++ to compile — just `pip install` and import.
- Finds the serial port by the device's serial number, so it survives replugging.
- Optional background status polling, so reading state doesn't block on Modbus.
- Compensates for fingertip motion when planning grasps (see [TCP offset](#tcp-offset)).

## Installation

Requires Python 3.10 or newer.

```shell
pip install robotiq2f
```

Or from a checkout:

```shell
git clone https://github.com/RobotControlStack/robotiq2f.git
cd robotiq2f
pip install -ve .
```

To access the serial port, add yourself to the `dialout` group, then log out and back in:

```shell
sudo adduser $USER dialout
```

As a temporary alternative that is reset when the gripper is replugged:

```shell
sudo chmod 666 /dev/ttyUSB0
```

## Finding your gripper

Lists every `/dev/ttyUSB*` device with its serial number:

```shell
python -m robotiq2f
> /dev/ttyUSB0 DAANTG8W
# the following command also works
udevadm info -a -n /dev/ttyUSB0 | grep serial
```

The serial number is what you pass to `Robotiq2F85(serial_number=...)`.

## Usage

```python
from robotiq2f import Robotiq2F85

# Initialize the driver with the gripper's serial number
gripper = Robotiq2F85(serial_number="DAANTG8W")

# Reset the gripper
gripper.reset()

# Move to fully open (85 mm) at 150 mm/s with up to 235 N
gripper.go_to(opening=85, speed=150, force=235)

# Individual properties
print(gripper.opening)          # current opening in mm
print(gripper.current)          # motor current in mA
print(gripper.is_moving)        # still travelling?
print(gripper.object_detected)  # did it stop on an object?
print(gripper.is_activated, gripper.in_fault)

# Or the whole state in one read
status = gripper.read_status()
print(status.opening, status.goal_opening, status.moving, status.fault.overheating)
```

With `async_control=True` (the default) a background thread polls the gripper, so these reads are
served from cache instead of blocking on Modbus. See `Robotiq2FStatus` and `GripperFault` for the
full set of fields.

## TCP offset

Ever had the gripper collide with the table when closing onto a small object? Because of how the gripper is built, the fingertips move away from the base frame as it closes. If your Tool Center Point sits on the fingertips, that shifts the TCP exactly when you need it placed precisely.

`tcp_Z_offset(desired_opening)` returns how far the TCP travels along the gripper's Z+ axis going from the current opening to the desired one, so the robot can pre-compensate before grasping an object of known thickness.

## Development

```shell
pip install -ve . && pip install --group dev
make format       # isort + black
make checkformat  # verify formatting without changing files
make lint         # ruff + mypy
make test         # pytest
```

## Limitations

Linux only — the serial-port lookup shells out to `udevadm`, which could be extended to other platforms.

Only the Robotiq 2F-85 is supported, via `Robotiq2F85`. It inherits from a (for now empty) `Robotiq2F` base class meant to hold the behaviour shared across the 2F family, once the model-specific calibration constants for other members (e.g. the 2F-140) are parameterized.

## License

[MIT](LICENSE)
