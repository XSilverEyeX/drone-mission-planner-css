
from pymavlink import mavutil
import time

PORT = "COM4"
BAUD = 57600

print(f"Connecting to {PORT} at {BAUD} baud...")

drone = mavutil.mavserial(
    device=PORT,
    baud=BAUD,
    source_system=255
)   

print("Waiting for Pixhawk heartbeat...")

heartbeat = drone.wait_heartbeat(timeout=20)

if heartbeat is None:
    print("No heartbeat received. Check radio power and TELEM2 settings.")
    drone.close()
    raise SystemExit(1)

print("CONNECTED TO PIXHAWK!")
print("System ID:", drone.target_system)
print("Autopilot:", heartbeat.autopilot)

print("\nListening for telemetry for 15 seconds...")

end_time = time.time() + 15

while time.time() < end_time:
    msg = drone.recv_match(
        type=["SYS_STATUS", "ATTITUDE", "GLOBAL_POSITION_INT"],
        blocking=True,
        timeout=1
    )

    if msg is None:
        continue

    if msg.get_type() == "SYS_STATUS":
        print(
            f"Battery: {msg.voltage_battery / 1000:.2f} V, "
            f"{msg.battery_remaining}%"
        )

    elif msg.get_type() == "ATTITUDE":
        print(f"Attitude received: roll={msg.roll:.2f}, pitch={msg.pitch:.2f}")

    elif msg.get_type() == "GLOBAL_POSITION_INT":
        print(f"GPS: {msg.lat / 1e7:.6f}, {msg.lon / 1e7:.6f}")

drone.close()
print("\nTest complete.")
