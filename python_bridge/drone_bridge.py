"""
drone_bridge.py

Connects the C# backend to the drone using MAVLink.

Communication:

Website
   ↓
Program.cs
   ↓
DroneClient.cs
   ↓ JSON over TCP
drone_bridge.py
   ↓ MAVLink
SiK Radio / SITL
   ↓
Pixhawk / Simulator

The C# side sends JSON commands to this program on TCP port 5555.
Python reads the command, performs the correct MAVLink action,
and sends a JSON response back to C#.
"""

import json
import math
import socket
import sys
import time

from pymavlink import mavutil
from serial.tools import list_ports


# ============================================================
# CONNECTION SETTINGS
# ============================================================

# "sitl"  = ArduPilot simulator
# "radio" = real Pixhawk through the SiK telemetry radio
CONNECTION_MODE = "radio"

# UDP connection used for the simulator
SITL_CONNECTION = "udpin:127.0.0.1:14551"

# Serial settings for the SiK telemetry radio
# The COM port will eventually be selected through the GUI
SERIAL_PORT = None
SERIAL_BAUD = 57600

# C# connects to this Python server using TCP
BRIDGE_HOST = "127.0.0.1"
BRIDGE_PORT = 5555

# Maximum time to wait for a MAVLink heartbeat
HEARTBEAT_TIMEOUT = 10

# Radius of Earth in meters
# Used when converting local x/y coordinates into GPS coordinates
EARTH_RADIUS_M = 6_378_137.0


# ============================================================
# COM PORTS
# ============================================================

def get_serial_ports():
    """
    Gets all COM ports currently detected by Windows.

    When the ground SiK radio is connected by USB,
    it should appear as one of these COM ports.
    """

    ports = []

    # Check every serial port detected by Windows
    for port in list_ports.comports():
        ports.append({
            "device": port.device,
            "description": port.description
        })

    return ports


# ============================================================
# DRONE BRIDGE
# ============================================================

class DroneBridge:
    """
    Handles the MAVLink connection and drone commands.
    """

    def __init__(self, connection_string, baud=None):
        """
        Opens a MAVLink connection and waits for a heartbeat.

        connection_string can be:

        SITL:
            udpin:127.0.0.1:14551

        Real radio:
            COM5
        """

        print(f"[bridge] Opening MAVLink connection: {connection_string}")

        # Serial connections need a baud rate.
        # UDP/SITL connections do not.
        if baud is not None:
            self.master = mavutil.mavlink_connection(
                connection_string,
                baud=baud
            )
        else:
            self.master = mavutil.mavlink_connection(connection_string)

        print(
            f"[bridge] Waiting up to {HEARTBEAT_TIMEOUT} seconds "
            "for MAVLink heartbeat..."
        )

        # Wait for a MAVLink heartbeat from the flight controller.
        # A heartbeat proves that we are communicating with the
        # Pixhawk, not just detecting the SiK radio.
        heartbeat = self.master.wait_heartbeat(
            timeout=HEARTBEAT_TIMEOUT
        )

        if heartbeat is None:
            raise TimeoutError(
                f"No MAVLink heartbeat received from {connection_string}"
            )

        print(
            f"[bridge] MAVLink heartbeat received! "
            f"System: {self.master.target_system}, "
            f"Component: {self.master.target_component}"
        )

        # Home position will be saved later using set_home()
        self.home_lat = None
        self.home_lon = None


    # ========================================================
    # SET HOME
    # ========================================================

    def set_home(self):
        """
        Saves the drone's current GPS position as the local home point.
        """

        # Get the current GPS position
        msg = self._get_position_message()

        if msg is None:
            return {
                "status": "error",
                "message": "Could not read GPS position"
            }

        # MAVLink sends lat/lon multiplied by 10^7
        self.home_lat = msg.lat / 1e7
        self.home_lon = msg.lon / 1e7

        return {
            "status": "ok",
            "message": "Home set",
            "data": {
                "lat": self.home_lat,
                "lon": self.home_lon
            }
        }


    # ========================================================
    # ARM
    # ========================================================

    def arm(self):
        """
        Puts the vehicle in GUIDED mode and sends the arm command.
        """

        # Switch the vehicle into GUIDED mode
        self._set_mode("GUIDED")

        # Send the MAVLink arm command
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # 1 = arm
            0,
            0,
            0,
            0,
            0,
            0
        )

        # Wait for the flight controller to accept the command
        if self._wait_for_ack(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        ):
            return {
                "status": "ok",
                "message": "Armed"
            }

        return {
            "status": "error",
            "message": "Arm command rejected"
        }


    # ========================================================
    # TAKEOFF
    # ========================================================

    def takeoff(self, altitude):
        """
        Sends a takeoff command and waits for the selected altitude.
        """

        # Send the MAVLink takeoff command
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            altitude
        )

        # Check if the flight controller accepted takeoff
        if not self._wait_for_ack(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        ):
            return {
                "status": "error",
                "message": "Takeoff command rejected"
            }

        # Wait until the drone reaches the requested altitude
        if self._wait_for_altitude(
            altitude,
            tolerance=1.0,
            timeout=30
        ):
            return {
                "status": "ok",
                "message": f"Reached altitude {altitude}m"
            }

        return {
            "status": "error",
            "message": "Timeout reaching altitude"
        }


    # ========================================================
    # GO TO GPS POSITION
    # ========================================================

    def goto(self, lat, lon, altitude):
        """
        Sends the drone to a GPS latitude and longitude.
        """

        # Send a GPS position target to the flight controller
        self.master.mav.set_position_target_global_int_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            int(0b110111111000),

            # MAVLink expects GPS coordinates multiplied by 10^7
            int(lat * 1e7),
            int(lon * 1e7),

            altitude,

            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0
        )

        # Wait until the drone gets close to the waypoint
        if self._wait_for_position(
            lat,
            lon,
            tolerance_m=2.0,
            timeout=60
        ):
            return {
                "status": "ok",
                "message": "Reached waypoint"
            }

        return {
            "status": "error",
            "message": "Timeout reaching waypoint"
        }


    # ========================================================
    # GO TO LOCAL X/Y POSITION
    # ========================================================

    def goto_local(self, x, y, altitude):
        """
        Converts a local x/y waypoint into GPS coordinates.

        x = meters east/west from home
        y = meters north/south from home
        """

        # Home must be known before local coordinates can be used
        if self.home_lat is None:
            return {
                "status": "error",
                "message": "Home not set — call set_home first"
            }

        # Convert meters north/south into latitude
        lat = (
            self.home_lat
            + (y / EARTH_RADIUS_M) * (180.0 / math.pi)
        )

        # Convert meters east/west into longitude
        lon = (
            self.home_lon
            + (x / EARTH_RADIUS_M)
            * (180.0 / math.pi)
            / math.cos(self.home_lat * math.pi / 180.0)
        )

        # Use the normal GPS goto function
        return self.goto(lat, lon, altitude)


    # ========================================================
    # LAND
    # ========================================================

    def land(self):
        """
        Changes the vehicle into LAND mode and waits for disarm.
        """

        # Switch into LAND mode
        self._set_mode("LAND")

        # Wait for the vehicle to land and disarm
        if self._wait_for_disarm(timeout=60):
            return {
                "status": "ok",
                "message": "Landed"
            }

        return {
            "status": "error",
            "message": "Timeout waiting for landing"
        }


    # ========================================================
    # GET POSITION
    # ========================================================

    def get_position(self):
        """
        Gets the latest GPS position from MAVLink.
        """

        msg = self._get_position_message()

        if msg is None:
            return {
                "status": "error",
                "message": "No GPS data available"
            }

        return {
            "status": "ok",
            "data": {
                # Convert MAVLink GPS values into normal degrees
                "lat": msg.lat / 1e7,
                "lon": msg.lon / 1e7,

                # relative_alt is in millimeters
                # Divide by 1000 to convert to meters
                "altitude": msg.relative_alt / 1000.0
            }
        }


    # ========================================================
    # INTERNAL HELPER FUNCTIONS
    # ========================================================

    def _set_mode(self, mode_name):
        """
        Changes the flight mode of the vehicle.
        """

        # Get the MAVLink ID for the requested mode
        mode_id = self.master.mode_mapping()[mode_name]

        # Send the mode change
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )


    def _wait_for_ack(self, command_id, timeout=5):
        """
        Waits for the flight controller to confirm that
        a MAVLink command was accepted.
        """

        start = time.time()

        while time.time() - start < timeout:

            msg = self.master.recv_match(
                type="COMMAND_ACK",
                blocking=True,
                timeout=1
            )

            # Make sure this ACK belongs to the command we sent
            if msg and msg.command == command_id:
                return (
                    msg.result
                    == mavutil.mavlink.MAV_RESULT_ACCEPTED
                )

        return False


    def _get_position_message(self, timeout=2):
        """
        Gets a GLOBAL_POSITION_INT message from MAVLink.
        """

        return self.master.recv_match(
            type="GLOBAL_POSITION_INT",
            blocking=True,
            timeout=timeout
        )


    def _wait_for_altitude(
        self,
        target_alt,
        tolerance,
        timeout
    ):
        """
        Waits until the drone reaches the requested altitude.
        """

        start = time.time()

        while time.time() - start < timeout:

            msg = self._get_position_message(timeout=1)

            if msg is None:
                continue

            # MAVLink altitude is in millimeters
            current_alt = msg.relative_alt / 1000.0

            # Check if the drone is close enough to the target altitude
            if abs(current_alt - target_alt) < tolerance:
                return True

        return False


    def _wait_for_position(
        self,
        target_lat,
        target_lon,
        tolerance_m,
        timeout
    ):
        """
        Waits until the drone gets close enough to a waypoint.
        """

        start = time.time()

        while time.time() - start < timeout:

            msg = self._get_position_message(timeout=1)

            if msg is None:
                continue

            # Get the current GPS position
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7

            # Calculate distance from the waypoint
            distance = self._distance_m(
                current_lat,
                current_lon,
                target_lat,
                target_lon
            )

            # Waypoint has been reached
            if distance < tolerance_m:
                return True

        return False


    def _wait_for_disarm(self, timeout):
        """
        Waits until the vehicle becomes disarmed after landing.
        """

        start = time.time()

        while time.time() - start < timeout:

            msg = self.master.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=1
            )

            if msg is None:
                continue

            # Check the armed flag in the heartbeat
            armed = bool(
                msg.base_mode
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )

            # Vehicle is finished landing once it disarms
            if not armed:
                return True

        return False


    @staticmethod
    def _distance_m(lat1, lon1, lat2, lon2):
        """
        Calculates the distance in meters between two GPS points.
        """

        rlat1 = math.radians(lat1)
        rlat2 = math.radians(lat2)

        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)

        # Haversine formula
        a = (
            math.sin(dlat / 2) ** 2
            + math.cos(rlat1)
            * math.cos(rlat2)
            * math.sin(dlon / 2) ** 2
        )

        c = 2 * math.atan2(
            math.sqrt(a),
            math.sqrt(1 - a)
        )

        return EARTH_RADIUS_M * c


# ============================================================
# COMMAND HANDLER
# ============================================================

def handle_command(bridge, command):
    """
    Takes a JSON command from C# and sends it to
    the correct DroneBridge function.
    """

    # Read the command name
    cmd = command.get("cmd")

    try:

        # ---------- Get COM ports ----------

        # This command works without a Pixhawk connection
        if cmd == "get_ports":
            return {
                "status": "ok",
                "message": "Serial ports detected",
                "data": {
                    "ports": get_serial_ports()
                }
            }

        # Every command below this point needs
        # an active MAVLink connection
        if bridge is None:
            return {
                "status": "error",
                "message": "No Pixhawk connected"
            }

        # ---------- Drone commands ----------

        if cmd == "arm":
            return bridge.arm()

        if cmd == "takeoff":
            return bridge.takeoff(
                command["altitude"]
            )

        if cmd == "goto":
            return bridge.goto(
                command["lat"],
                command["lon"],
                command["altitude"]
            )

        if cmd == "goto_local":
            return bridge.goto_local(
                command["x"],
                command["y"],
                command["altitude"]
            )

        if cmd == "land":
            return bridge.land()

        if cmd == "get_position":
            return bridge.get_position()

        if cmd == "set_home":
            return bridge.set_home()

        # Command was not recognized
        return {
            "status": "error",
            "message": f"Unknown command: {cmd}"
        }

    # A required part of the command was missing
    except KeyError as e:
        return {
            "status": "error",
            "message": f"Missing field: {e}"
        }

    # Catch other errors so the server does not crash
    except Exception as e:
        return {
            "status": "error",
            "message": f"Exception: {e}"
        }


# ============================================================
# TCP SERVER
# ============================================================

def serve(bridge):
    """
    Runs the TCP server that DroneClient.cs connects to.
    """

    # Create the TCP server
    server = socket.socket(
        socket.AF_INET,
        socket.SOCK_STREAM
    )

    # Allows the same port to be reused after restarting
    server.setsockopt(
        socket.SOL_SOCKET,
        socket.SO_REUSEADDR,
        1
    )

    # Start the server on 127.0.0.1:5555
    server.bind(
        (BRIDGE_HOST, BRIDGE_PORT)
    )

    # Allow one C# client to connect
    server.listen(1)

    print(
        f"[bridge] Listening on "
        f"{BRIDGE_HOST}:{BRIDGE_PORT}"
    )

    # Keep running until the program is manually stopped
    while True:

        # Wait for DroneClient.cs to connect
        client, addr = server.accept()

        print(
            f"[bridge] Client connected from {addr}"
        )

        try:

            # Stores incoming data until a full command is received
            buffer = ""

            while True:

                # Read data sent by C#
                data = client.recv(4096).decode("utf-8")

                # C# disconnected
                if not data:
                    break

                buffer += data

                # Each JSON command ends with a new line
                while "\n" in buffer:

                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()

                    if not line:
                        continue

                    try:

                        # Convert JSON text into a Python dictionary
                        command = json.loads(line)

                    except json.JSONDecodeError as e:

                        response = {
                            "status": "error",
                            "message": f"Invalid JSON: {e}"
                        }

                    else:

                        print(
                            f"[bridge] <- {command}"
                        )

                        # Run the command
                        response = handle_command(
                            bridge,
                            command
                        )

                        print(
                            f"[bridge] -> {response}"
                        )

                    # Convert the response back into JSON
                    # and send it to C#
                    client.sendall(
                        (
                            json.dumps(response) + "\n"
                        ).encode("utf-8")
                    )

        except ConnectionResetError:

            print(
                "[bridge] Client disconnected abruptly"
            )

        finally:

            client.close()

            print(
                "[bridge] Client disconnected"
            )


# ============================================================
# MAIN PROGRAM
# ============================================================

def main():

    try:

        # ---------- Simulator mode ----------

        if CONNECTION_MODE.lower() == "sitl":

            print("[bridge] Mode: SITL")

            try:

                # Connect directly to the simulator
                bridge = DroneBridge(
                    SITL_CONNECTION
                )

            except TimeoutError as e:

                print(
                    f"[bridge] SITL not connected: {e}"
                )

                bridge = None


        # ---------- Real radio mode ----------

        elif CONNECTION_MODE.lower() == "radio":

            print(
                "[bridge] Mode: SiK telemetry radio"
            )

            # Start without a Pixhawk connection.
            # The GUI will eventually select a COM port
            # and request the connection.
            bridge = None

            print(
                "[bridge] Waiting for telemetry connection request."
            )

            print(
                "[bridge] Available COM ports:"
            )

            # Check which COM ports Windows currently detects
            ports = get_serial_ports()

            if not ports:

                print(
                    "  No COM ports detected."
                )

            else:

                # Print every detected COM port
                for port in ports:

                    print(
                        f"  {port['device']} - "
                        f"{port['description']}"
                    )


        # ---------- Invalid mode ----------

        else:

            print(
                f"[bridge] ERROR: Unknown connection mode: "
                f"{CONNECTION_MODE}"
            )

            return


        # Start the C# <-> Python server.
        #
        # The server still starts without a Pixhawk so
        # the GUI can request the available COM ports.
        serve(bridge)


    # Ctrl+C stops the program
    except KeyboardInterrupt:

        print(
            "\n[bridge] Shutting down"
        )


    # Catch other startup errors
    except Exception as e:

        print(
            f"[bridge] ERROR: {e}"
        )


# Run main() when this file is started directly
if __name__ == "__main__":
    main()