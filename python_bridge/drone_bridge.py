"""
drone_bridge.py — translator between the C# planner and ArduPilot SITL.

Architecture:
    [C# planner] --JSON over TCP--> [this bridge] --MAVLink over UDP--> [SITL]

The bridge listens on TCP port 5555 for line-delimited JSON commands, executes
them against the drone, and writes JSON responses back to the same connection.

Commands (each is a JSON object with a 'cmd' field):
    {"cmd": "arm"}
    {"cmd": "takeoff", "altitude": 20}
    {"cmd": "goto", "lat": -35.36, "lon": 149.16, "altitude": 20}
    {"cmd": "goto_local", "x": 50, "y": 30, "altitude": 20}
    {"cmd": "land"}
    {"cmd": "get_position"}
    {"cmd": "set_home"}

Responses (one JSON object per command):
    {"status": "ok", "message": "...", "data": {...optional...}}
    {"status": "error", "message": "..."}

The 'goto_local' command takes meters offset from the home GPS position, which
is how the C# planner thinks about the world. The bridge does the conversion
to lat/lon using a local equirectangular approximation — accurate enough for
short-range flight (a few km).
"""

import json
import math
import socket
import sys
import time
from pymavlink import mavutil
from serial.tools import list_ports

# Configuration
# ---------- Connection configuration ----------

# "sitl"  = ArduPilot simulator
# "radio" = Real Pixhawk through a SiK telemetry radio
CONNECTION_MODE = "radio"

# Simulator connection
SITL_CONNECTION = "udpin:127.0.0.1:14551"

# Real SiK telemetry radio settings.
# We will change SERIAL_PORT tomorrow after Windows detects the radio.
SERIAL_PORT = None
SERIAL_BAUD = 57600

# C# <-> Python bridge settings
BRIDGE_HOST = "127.0.0.1"
BRIDGE_PORT = 5555

# Don't wait forever if no Pixhawk/SITL heartbeat is received.
HEARTBEAT_TIMEOUT = 10

# Earth's radius in meters — used by local-to-GPS conversion.
EARTH_RADIUS_M = 6_378_137.0

def get_serial_ports():
    """
    Returns all serial/COM ports currently detected by Windows.

    Tomorrow, when the SiK telemetry radio is plugged into USB,
    it should appear in this list as a COM port.
    """
    ports = []

    for port in list_ports.comports():
        ports.append({
            "device": port.device,
            "description": port.description
        })

    return ports


class DroneBridge:
    """Wraps a pymavlink connection with high-level command methods."""

    def __init__(self, connection_string, baud=None):
        """
        Open a MAVLink connection and wait for the flight controller heartbeat.

        connection_string can be either:
        - SITL UDP connection, such as udpin:127.0.0.1:14551
        - Windows serial port, such as COM5
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

        self.home_lat = None
        self.home_lon = None

    def set_home(self):
        msg = self._get_position_message()
        if msg is None:
            return {"status": "error", "message": "Could not read GPS position"}
        self.home_lat = msg.lat / 1e7
        self.home_lon = msg.lon / 1e7
        return {
            "status": "ok",
            "message": "Home set",
            "data": {"lat": self.home_lat, "lon": self.home_lon}
        }

    def arm(self):
        self._set_mode('GUIDED')
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        if self._wait_for_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM):
            return {"status": "ok", "message": "Armed"}
        return {"status": "error", "message": "Arm command rejected"}

    def takeoff(self, altitude):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude
        )
        if not self._wait_for_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF):
            return {"status": "error", "message": "Takeoff command rejected"}
        if self._wait_for_altitude(altitude, tolerance=1.0, timeout=30):
            return {"status": "ok", "message": f"Reached altitude {altitude}m"}
        return {"status": "error", "message": "Timeout reaching altitude"}

    def goto(self, lat, lon, altitude):
        self.master.mav.set_position_target_global_int_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            int(0b110111111000),
            int(lat * 1e7), int(lon * 1e7), altitude,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        if self._wait_for_position(lat, lon, tolerance_m=2.0, timeout=60):
            return {"status": "ok", "message": "Reached waypoint"}
        return {"status": "error", "message": "Timeout reaching waypoint"}

    def goto_local(self, x, y, altitude):
        if self.home_lat is None:
            return {"status": "error", "message": "Home not set — call set_home first"}
        lat = self.home_lat + (y / EARTH_RADIUS_M) * (180.0 / math.pi)
        lon = self.home_lon + (x / EARTH_RADIUS_M) * (180.0 / math.pi) / math.cos(
            self.home_lat * math.pi / 180.0
        )
        return self.goto(lat, lon, altitude)

    def land(self):
        self._set_mode('LAND')
        if self._wait_for_disarm(timeout=60):
            return {"status": "ok", "message": "Landed"}
        return {"status": "error", "message": "Timeout waiting for landing"}

    def get_position(self):
        msg = self._get_position_message()
        if msg is None:
            return {"status": "error", "message": "No GPS data available"}
        return {
            "status": "ok",
            "data": {
                "lat": msg.lat / 1e7,
                "lon": msg.lon / 1e7,
                "altitude": msg.relative_alt / 1000.0
            }
        }

    # ---------- internal helpers ----------

    def _set_mode(self, mode_name):
        mode_id = self.master.mode_mapping()[mode_name]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )

    def _wait_for_ack(self, command_id, timeout=5):
        start = time.time()
        while time.time() - start < timeout:
            msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
            if msg and msg.command == command_id:
                return msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        return False

    def _get_position_message(self, timeout=2):
        return self.master.recv_match(
            type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout
        )

    def _wait_for_altitude(self, target_alt, tolerance, timeout):
        start = time.time()
        while time.time() - start < timeout:
            msg = self._get_position_message(timeout=1)
            if msg is None:
                continue
            current_alt = msg.relative_alt / 1000.0
            if abs(current_alt - target_alt) < tolerance:
                return True
        return False

    def _wait_for_position(self, target_lat, target_lon, tolerance_m, timeout):
        start = time.time()
        while time.time() - start < timeout:
            msg = self._get_position_message(timeout=1)
            if msg is None:
                continue
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            d = self._distance_m(current_lat, current_lon, target_lat, target_lon)
            if d < tolerance_m:
                return True
        return False

    def _wait_for_disarm(self, timeout):
        start = time.time()
        while time.time() - start < timeout:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg is None:
                continue
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed:
                return True
        return False

    @staticmethod
    def _distance_m(lat1, lon1, lat2, lon2):
        rlat1 = math.radians(lat1)
        rlat2 = math.radians(lat2)
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat / 2) ** 2 + math.cos(rlat1) * math.cos(rlat2) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return EARTH_RADIUS_M * c


def handle_command(bridge, command):
    """Dispatch a single JSON command to the appropriate bridge method."""
    cmd = command.get("cmd")
    try:
        if cmd == "get_ports":
            return {
                "status": "ok",
                "message": "Serial ports detected",
                "data": {
                    "ports": get_serial_ports()
                }
            }
                # All commands below this point require an active MAVLink connection.
        if bridge is None:
            return {
                "status": "error",
                "message": "No Pixhawk connected"
            }
        if cmd == "arm":
            return bridge.arm()
        if cmd == "takeoff":
            return bridge.takeoff(command["altitude"])
        if cmd == "goto":
            return bridge.goto(command["lat"], command["lon"], command["altitude"])
        if cmd == "goto_local":
            return bridge.goto_local(command["x"], command["y"], command["altitude"])
        if cmd == "land":
            return bridge.land()
        if cmd == "get_position":
            return bridge.get_position()
        if cmd == "set_home":
            return bridge.set_home()
        return {"status": "error", "message": f"Unknown command: {cmd}"}
    except KeyError as e:
        return {"status": "error", "message": f"Missing field: {e}"}
    except Exception as e:
        return {"status": "error", "message": f"Exception: {e}"}


def serve(bridge):
    """Run the TCP server loop. Accepts one client at a time."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((BRIDGE_HOST, BRIDGE_PORT))
    server.listen(1)
    print(f"[bridge] Listening on {BRIDGE_HOST}:{BRIDGE_PORT}")

    while True:
        client, addr = server.accept()
        print(f"[bridge] Client connected from {addr}")
        try:
            buffer = ""
            while True:
                data = client.recv(4096).decode("utf-8")
                if not data:
                    break
                buffer += data
                # Process every complete line (newline-delimited JSON).
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        command = json.loads(line)
                    except json.JSONDecodeError as e:
                        response = {"status": "error", "message": f"Invalid JSON: {e}"}
                    else:
                        print(f"[bridge] <- {command}")
                        response = handle_command(bridge, command)
                        print(f"[bridge] -> {response}")
                    client.sendall((json.dumps(response) + "\n").encode("utf-8"))
        except ConnectionResetError:
            print("[bridge] Client disconnected abruptly")
        finally:
            client.close()
            print("[bridge] Client disconnected")


def main():
    try:
        # ---------- Simulator mode ----------
        if CONNECTION_MODE.lower() == "sitl":
            print("[bridge] Mode: SITL")

            try:
                bridge = DroneBridge(SITL_CONNECTION)
            except TimeoutError as e:
                print(f"[bridge] SITL not connected: {e}")
                bridge = None

        # ---------- Real radio mode ----------
        elif CONNECTION_MODE.lower() == "radio":
            print("[bridge] Mode: SiK telemetry radio")

            # Start with no Pixhawk connection.
            # The GUI will eventually choose the COM port and request a connection.
            bridge = None

            print("[bridge] Waiting for telemetry connection request.")
            print("[bridge] Available COM ports:")

            ports = get_serial_ports()

            if not ports:
                print("  No COM ports detected.")
            else:
                for port in ports:
                    print(
                        f"  {port['device']} - "
                        f"{port['description']}"
                    )

        else:
            print(
                f"[bridge] ERROR: Unknown connection mode: "
                f"{CONNECTION_MODE}"
            )
            return

        # IMPORTANT:
        # Start the C# <-> Python server even if no Pixhawk is connected.
        serve(bridge)

    except KeyboardInterrupt:
        print("\n[bridge] Shutting down")

    except Exception as e:
        print(f"[bridge] ERROR: {e}")


if __name__ == "__main__":
    main()

