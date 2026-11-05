# VENATOR SOURCE GUIDE: Serial-port selection and read-only MAVLink telemetry helper.
# Comments describe the existing implementation; operational logic remains unchanged.

"""
VENATOR Telemetry Demo
mavlink_connection.py

Handles the direct connection between this computer and the Pixhawk
through the SiK telemetry radios.

This file does NOT require the C# server or localhost website.
"""

from pymavlink import mavutil
import serial.tools.list_ports
import time


# MAVLinkConnection groups related telemetry or navigation operations.
class MAVLinkConnection:
    """Handles the SiK radio / Pixhawk MAVLink connection."""

    # Executes the __init__ operation and retains existing input and error handling.
    def __init__(self):
        self.master = None
        self.port = None
        self.baud = None
        self.last_heartbeat_time = None

    # ---------------------------------------------------------
    # COM PORT DETECTION
    # ---------------------------------------------------------

    @staticmethod
    # Executes the get_serial_ports operation and retains existing input and error handling.
    def get_serial_ports():
        """Return all serial/COM ports currently detected by Windows."""

        ports = []

        for port in serial.tools.list_ports.comports():
            ports.append({
                "device": port.device,
                "description": port.description
            })

        return ports

    # ---------------------------------------------------------
    # CONNECTION
    # ---------------------------------------------------------

    # Executes the connect operation and retains existing input and error handling.
    def connect(self, port, baud=57600, heartbeat_timeout=10):
        """
        Open the selected COM port and wait for a MAVLink heartbeat.

        Opening the COM port proves that Windows can communicate with
        the ground SiK radio.

        Receiving a heartbeat proves that MAVLink data is making the
        complete trip from the Pixhawk through both radios.
        """

        self.port = port
        self.baud = baud

        print(f"\nOpening {port} at {baud} baud...")

        try:
            self.master = mavutil.mavlink_connection(
                port,
                baud=baud
            )

        except Exception as error:
            self.master = None

            return {
                "success": False,
                "radio_connected": False,
                "pixhawk_connected": False,
                "message": f"Could not open {port}: {error}"
            }

        print("Serial port opened.")
        print("Waiting for MAVLink heartbeat...")

        heartbeat = self.master.wait_heartbeat(
            timeout=heartbeat_timeout
        )

        if heartbeat is None:
            # The radio/COM port opened, but we did not receive
            # MAVLink traffic from a flight controller.
            return {
                "success": False,
                "radio_connected": True,
                "pixhawk_connected": False,
                "message": (
                    f"{port} opened, but no MAVLink heartbeat "
                    f"was received within {heartbeat_timeout} seconds."
                )
            }

        self.last_heartbeat_time = time.time()

        return {
            "success": True,
            "radio_connected": True,
            "pixhawk_connected": True,
            "message": "MAVLink heartbeat received.",
            "system_id": self.master.target_system,
            "component_id": self.master.target_component
        }

    # ---------------------------------------------------------
    # STATUS
    # ---------------------------------------------------------

    # Executes the is_radio_connected operation and retains existing input and error handling.
    def is_radio_connected(self):
        """True when a MAVLink serial connection object exists."""

        return self.master is not None

    # Executes the heartbeat_age operation and retains existing input and error handling.
    def heartbeat_age(self):
        """Return the number of seconds since the last known heartbeat."""

        if self.last_heartbeat_time is None:
            return None

        return time.time() - self.last_heartbeat_time

    # ---------------------------------------------------------
    # POSITION
    # ---------------------------------------------------------

    # Executes the get_position operation and retains existing input and error handling.
    def get_position(self, timeout=2):
        """Request the most recent GPS/global position message."""

        if self.master is None:
            return None

        message = self.master.recv_match(
            type="GLOBAL_POSITION_INT",
            blocking=True,
            timeout=timeout
        )

        if message is None:
            return None

        return {
            "lat": message.lat / 1e7,
            "lon": message.lon / 1e7,
            "altitude": message.relative_alt / 1000.0
        }

    # ---------------------------------------------------------
    # DISCONNECT
    # ---------------------------------------------------------

    # Executes the disconnect operation and retains existing input and error handling.
    def disconnect(self):
        """Close the serial connection."""

        if self.master is not None:
            self.master.close()

        self.master = None
        self.port = None
        self.baud = None
        self.last_heartbeat_time = None