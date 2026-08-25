import math
import struct
import subprocess
import threading
from dataclasses import dataclass, field
from pathlib import Path
from time import perf_counter, sleep

import minimalmodbus as mm


class SimpleFrameRate:
    def __init__(self, frame_rate: float | None):
        self.t: float | None = None
        self.frame_rate = frame_rate

    def reset(self):
        self.t = None

    def limit(self):
        if self.frame_rate is None:
            return
        if self.t is None:
            self.t = perf_counter()
            return
        sleep_time = 1 / self.frame_rate - (perf_counter() - self.t)
        if sleep_time > 0:
            sleep(sleep_time)
        self.t = perf_counter()


class LinuxFindTTYWithSerialNumber:
    def __init__(self):
        pass

    def find(self, serial_number):
        """
        Iterate over all /dev/ttyUSB* devices and try to find the one with the given serial number.

        A list of devices can be obtained with: ls /dev/ttyUSB*
        The serial number of a given device can be obtained with: udevadm info -a -n /dev/ttyUSB0 | grep 'ATTRS{serial}' | head -n1
        producing the output: ATTRS{serial}=="serial_number".

        Parameters:
        -----------
        serial_number : str
            Serial number of the device to find.

        Returns:
        --------
        str
            The path to the device with the given serial number or None if no device matching the specified serial number was found.
        """

        # Get the list of all /dev/ttyUSB* devices.
        tty_devices = Path("/dev").glob("ttyUSB*")

        # Iterate over all /dev/ttyUSB* devices and try to find the one with the given serial number.
        for tty_device in tty_devices:
            # Get the serial number of the current device.
            current_serial_number = self.get_serial_number(tty_device)

            # Check if the serial number of the current device matches the specified serial number.
            if current_serial_number == serial_number:
                # Return the path to the current device.
                return str(tty_device)

        # Return None if no device matching the specified serial number was found.
        return None

    def list_devices(self) -> list[tuple[str, str | None]]:
        """
        List every /dev/ttyUSB* device together with its serial number.

        Returns:
        --------
        list[tuple[str, str | None]]
            (device path, serial number) pairs, sorted by device path. The serial number is
            None for devices that do not expose an ATTRS{serial} attribute.
        """
        return [(str(tty), self.get_serial_number(tty)) for tty in sorted(Path("/dev").glob("ttyUSB*"))]

    def get_serial_number(self, tty_device: Path):
        """
        Get the serial number of a given device.

        Parameters:
        -----------
        tty_device : Path
            Path to the device.

        Returns:
        --------
        str
            The serial number of the device.
        """

        # Get the serial number of the device by running the following command:
        # udevadm info -a -n /dev/ttyUSB0 | grep 'ATTRS{serial}' | head -n1
        # producing the output: ATTRS{serial}=="serial_number".
        output = subprocess.check_output(["udevadm", "info", "-a", "-n", str(tty_device)]).decode("utf-8")
        serial_lines = [line for line in output.split("\n") if "ATTRS{serial}" in line]

        # If no line with 'ATTRS{serial}' was found, return None.
        if len(serial_lines) == 0:
            return None

        try:
            serial_number = serial_lines[0].split('"')[1]
        except (IndexError, ValueError):
            serial_number = None

        # Return the serial number of the device.
        return serial_number


@dataclass
class GripperFault:
    # Reactivation must be performed before any further movement.
    reactivation_required: bool = False
    # Activation bit must be set prior to action.
    activation_required: bool = False
    # Gripper's temperature has risen above 85C and it needs to cool down.
    overheating: bool = False
    # There was no communication within the last second
    communication_timeout: bool = False
    # The voltage supplied to the gripper is below 21.6 Volts
    undervoltage: bool = False
    # Automatic release in progress
    is_auto_releasing: bool = False
    # Automatic release completed
    auto_release_completed: bool = False
    # Internal fault, contact manufacturer.
    internal_fault: bool = False
    # Activation fault
    activation_fault: bool = False
    # A current of more than 1 Amp. was supplied
    overcurrent: bool = False


@dataclass
class Robotiq2FStatus:
    activated: bool
    moving: bool
    # In milliamps
    current: float
    obj_detected: bool
    # In millimeters
    opening: float
    # In millimeters
    goal_opening: float
    is_reset: bool
    is_activating: bool
    is_activated: bool
    fault: GripperFault = field(default_factory=GripperFault)


class GripperNotFoundError(RuntimeError):
    """Raised when no serial device with the requested serial number could be found."""


class Robotiq2F:
    """Common base class for the Robotiq 2F gripper family."""


class Robotiq2F85(Robotiq2F):
    # in hz
    MAX_FREQUENCY = 200
    # in mm, fully open
    MAX_OPENING = 85.0

    def __init__(
        self, serial_number: str, debug: bool = False, async_control: bool = True, read_frequency: float = 200
    ):
        self.debug = debug
        self.device_serial_number = serial_number
        self.tty_device = LinuxFindTTYWithSerialNumber().find(serial_number)
        self.async_control = async_control
        self._last_status_mutex = threading.Lock()
        self._last_status: Robotiq2FStatus | None = None
        self._last_status_time: float | None = None
        self._async_thread: threading.Thread | None = None
        self.read_frequency = read_frequency

        if self.tty_device is None:
            msg = f"No device with serial number {serial_number} found."
            raise GripperNotFoundError(msg)

        self._client_mutex = threading.Lock()
        self.client = mm.Instrument(port=self.tty_device, slaveaddress=9, mode=mm.MODE_RTU, debug=self.debug)
        self.client.serial.baudrate = 115200
        self.client.serial.parity = mm.serial.PARITY_NONE
        self.client.serial.bytesize = 8
        self.client.serial.stopbits = mm.serial.STOPBITS_ONE

    def close(self):
        """Close the serial connection to the gripper."""
        self.client.serial.close()

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()

    @property
    def opening(self):
        """Current opening in millimeters"""
        return self.read_status().opening

    @property
    def goal_opening(self):
        """Goal opening in millimeters"""
        return self.read_status().goal_opening

    @property
    def current(self):
        """Current in milliamps"""
        return self.read_status().current

    @property
    def is_reset(self):
        return self.read_status().is_reset

    @property
    def is_activating(self):
        return self.read_status().is_activating

    @property
    def is_activated(self):
        return self.read_status().is_activated

    @property
    def is_moving(self):
        return self.read_status().moving

    @property
    def object_detected(self):
        return self.read_status().obj_detected

    @property
    def in_fault(self):
        fault = self.read_status().fault
        return (
            fault.reactivation_required
            or fault.activation_required
            or fault.overheating
            or fault.undervoltage
            or fault.internal_fault
            or fault.activation_fault
            or fault.overcurrent
        )

    def count_to_opening(self, count: int):
        """Converts a count to an opening in millimeters"""
        count = min(max(count, 0), 255)
        opening = (230 - count) * 0.39
        return min(max(opening, 0), self.MAX_OPENING)

    def opening_to_count(self, opening: float):
        """Converts an opening in millimeters to a count"""
        opening = min(max(opening, 0), self.MAX_OPENING)
        count = 230 - (opening / 0.39)
        return int(count)

    def count_to_speed(self, count: int):
        """Converts a count to a speed in mm/s
        The speed is between 20-150 mm/s for counts 0-255.
        """
        count = min(max(count, 0), 255)
        return (count / 255) * (150 - 20) + 20

    def speed_to_count(self, speed: float):
        """Converts a speed in mm/s to a count.
        The speed is between 20-150 mm/s for counts 0-255.
        """
        count = (speed - 20) / (150 - 20) * 255
        count = min(max(count, 0), 255)
        return int(count)

    def count_to_force(self, count: int):
        """Converts a count to a force in N
        The force is between 20-235 N for counts 0-255.
        """
        force = (count / 255) * (235 - 20) + 20
        return min(max(force, 0), 235)

    def force_to_count(self, force: float):
        """Converts a force in N to a count
        The force is between 20-235 N for counts 0-255.
        """
        count = (force - 20) / (235 - 20) * 255
        count = min(max(count, 0), 255)
        return int(count)

    def count_to_current(self, count: int):
        """Converts a count to a current in mA"""
        return count * 0.1

    def activate(self, blocking_call: bool = True):
        """
        Activate the gripper.
        """
        action_request_register = 1 << 8
        gripper_options1_register = 0
        with self._client_mutex:
            self.client.write_registers(
                registeraddress=1000, values=[action_request_register + gripper_options1_register]
            )

        if blocking_call:
            # Read the status while the gripper is activating
            fps = SimpleFrameRate(self.MAX_FREQUENCY)
            while self.read_status_sync().is_activating:
                fps.limit()
            # Read the status until the gripper is activated
            while not self.read_status_sync().is_activated:
                fps.limit()

    def deactivate(self, blocking_call: bool = True):
        """
        Deactivate the gripper.
        """
        action_request_register = 0 << 8
        gripper_options1_register = 0
        with self._client_mutex:
            self.client.write_registers(
                registeraddress=1000, values=[action_request_register + gripper_options1_register]
            )

        if blocking_call:
            # Read the status until the gripper is deactivated
            fps = SimpleFrameRate(self.MAX_FREQUENCY)
            while self.read_status_sync().is_activated:
                fps.limit()

    def reset(self, blocking_call: bool = True):
        """
        Reset the gripper.
        """
        self.deactivate(blocking_call)
        self.activate(blocking_call)

    def tcp_Z_from_opening(self, opening: float, pad_thickness: float = 7.8):
        """
        Returns the distance between the gripper base frame and the middle of the fingertips
        when the distance between the fingertips is `opening` and the pad thickness is `pad_thickness`.

        Parameters
        ----------
        opening : float
            Distance between the fingertips. Fully open is `MAX_OPENING` mm, and fully closed is 0mm.
        pad_thickness : float
            Thickness of the pads. Default is 7.8mm (silicone pads).
        """
        # Distance from the Z axis to the farthest side of the fingertip
        d = opening / 2 + pad_thickness
        if d < 12.7:
            tcp_z = 87.308 + 57.15 * math.sqrt(1 - ((12.7 - d) / 57.15) ** 2)
        else:
            tcp_z = 87.308 + 57.15 * math.sqrt(1 - ((d - 12.7) / 57.15) ** 2)
        return tcp_z

    def tcp_Z_offset(self, desired_opening: float, pad_thickness: float = 7.8):
        """
        Returns the distance along the gripper Z+ axis that the TCP (fixed at the middle of the fingertip)
        will move when the gripper goes from its current opening to the desired opening.

        This can be used to compensate for the gripper's movement when the opening is changed such that
        the TCP ends up at the desired position. This requires knowing the thickness of the object to be
        grasped. The robot can be moved to compensate for this offset prior to grasping the object.

        Parameters
        ----------
        desired_opening : float
            Desired opening in millimeters.
        pad_thickness : float
            Thickness of the pads. Default is 7.8mm (silicone pads).
        """
        current_opening = self.opening
        current_tcp_z = self.tcp_Z_from_opening(current_opening, pad_thickness)
        desired_tcp_z = self.tcp_Z_from_opening(desired_opening, pad_thickness)
        return desired_tcp_z - current_tcp_z

    def go_to(self, opening: float, speed: float, force: float):
        """
        Move the gripper to the specified opening, speed and force.

        Parameters:
        -----------
        opening : float
            Opening in millimeters. Must be between 0 and `MAX_OPENING` mm.
        speed : float
            Speed in mm/s. Must be between 20 and 150 mm/s.
        force : float
            Force in N. Must be between 20 and 235 N.
        """
        opening_count = self.opening_to_count(opening)
        speed_count = self.speed_to_count(speed)
        force_count = self.force_to_count(force)

        # Byte 0
        action_request_register = (2**0 + 2**3) << 8
        # Byte 1
        gripper_options1_register = 0
        # Byte 2
        gripper_options2_register = 0
        # Byte 3
        position_request_register = opening_count
        # Byte 4
        speed_register = speed_count << 8
        # Byte 5
        force_register = force_count

        with self._client_mutex:
            self.client.write_registers(
                registeraddress=1000,
                values=[
                    action_request_register + gripper_options1_register,
                    gripper_options2_register + position_request_register,
                    speed_register + force_register,
                ],
            )

        if not self.async_control:
            # Read the status until the gripper is stopped
            fps = SimpleFrameRate(self.MAX_FREQUENCY)
            while self.read_status_sync().moving:
                fps.limit()

    def read_status(self) -> Robotiq2FStatus:
        if not self.async_control:
            return self.read_status_sync()
        if self._async_thread is None:
            self.read_status_sync()
            self._async_thread = threading.Thread(target=self.read_status_thread, daemon=True)
            self._async_thread.start()

        with self._last_status_mutex:
            assert self._last_status is not None
            return self._last_status

    def read_status_thread(self):
        fps = SimpleFrameRate(self.read_frequency)
        while True:
            self.read_status_sync()
            fps.limit()

    def read_status_sync(self) -> Robotiq2FStatus:
        with self._last_status_mutex:
            if (
                self._last_status is not None
                and self._last_status_time is not None
                and self._last_status_time + 1 / self.MAX_FREQUENCY > perf_counter()
            ):
                return self._last_status
            self._last_status_time = perf_counter()

        with self._client_mutex:
            values = self.client.read_registers(registeraddress=2000, number_of_registers=3, functioncode=4)

        # Each register is 16 bits and therefore contains two unsigned char each
        gripper_status_register, _reserved_register = struct.unpack("BB", values[0].to_bytes(2, "big"))
        fault_status_register, position_request_echo_register = struct.unpack("BB", values[1].to_bytes(2, "big"))
        position_register, current_register = struct.unpack("BB", values[2].to_bytes(2, "big"))

        gripper_fault = GripperFault(
            reactivation_required=bool(fault_status_register == 0x05),
            activation_required=bool(fault_status_register == 0x07),
            overheating=bool(fault_status_register == 0x08),
            undervoltage=bool(fault_status_register == 0x0A),
            is_auto_releasing=bool(fault_status_register == 0x0B),
            internal_fault=bool(fault_status_register == 0x0C),
            activation_fault=bool(fault_status_register == 0x0D),
            overcurrent=bool(fault_status_register == 0x0E),
            auto_release_completed=bool(fault_status_register == 0x0F),
        )

        status = Robotiq2FStatus(
            activated=bool(gripper_status_register & 2**0),
            moving=bool(gripper_status_register & 2**3)
            and (not bool(gripper_status_register & 2**6) and not bool(gripper_status_register & 2**7)),
            current=self.count_to_current(current_register),
            obj_detected=(bool(gripper_status_register & 2**6) and not bool(gripper_status_register & 2**7))
            or (not bool(gripper_status_register & 2**6) and bool(gripper_status_register & 2**7)),
            opening=self.count_to_opening(position_register),
            goal_opening=self.count_to_opening(position_request_echo_register),
            is_reset=not bool(gripper_status_register & 2**4) and not bool(gripper_status_register & 2**5),
            is_activating=bool(gripper_status_register & 2**4) and not bool(gripper_status_register & 2**5),
            is_activated=bool(gripper_status_register & 2**4) and bool(gripper_status_register & 2**5),
            fault=gripper_fault,
        )

        # Print the status of the gripper
        if self.debug:
            print("Gripper Status: ")
            print(f"\tActivated: {status.activated}")
            print(f"\tMoving: {status.moving}")
            print(f"\tIs Reset: {status.is_reset}")
            print(f"\tIs Activating: {status.is_activating}")
            print(f"\tIs Activated: {status.is_activated}")
            print(f"\tObj Detected: {status.obj_detected}")
            print("\tFaults: ", end="")
            if gripper_fault.reactivation_required:
                print("Reactivation required, ", end="")
            if gripper_fault.activation_required:
                print("Activation required, ", end="")
            if gripper_fault.overheating:
                print("Overheating, ", end="")
            if gripper_fault.undervoltage:
                print("Undervoltage, ", end="")
            if gripper_fault.is_auto_releasing:
                print("Is auto-releasing, ", end="")
            if gripper_fault.internal_fault:
                print("Internal fault, ", end="")
            if gripper_fault.activation_fault:
                print("Activation fault, ", end="")
            if gripper_fault.overcurrent:
                print("Overcurrent, ", end="")
            if gripper_fault.auto_release_completed:
                print("Auto-release completed", end="")
            print()

        with self._last_status_mutex:
            self._last_status = status
        return status


if __name__ == "__main__":
    gripper = Robotiq2F85(serial_number="DAK1RLYZ")
    gripper.reset()
    gripper.go_to(opening=Robotiq2F85.MAX_OPENING, speed=150, force=235)
    print(gripper.opening)
