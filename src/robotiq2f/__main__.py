"""Command line interface: list the serial numbers of the connected grippers."""

from robotiq2f.robotiq2f import LinuxFindTTYWithSerialNumber


def main() -> None:
    for port, serial_number in LinuxFindTTYWithSerialNumber().list_devices():
        print(port, serial_number)


if __name__ == "__main__":
    main()
