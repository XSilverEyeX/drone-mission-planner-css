"""
VENATOR SiK Radio-to-Radio Test

Tests bidirectional communication between two SiK telemetry radios
connected to the same computer.

Radio A: COM3
Radio B: COM4
"""

import serial
import time


# -----------------------------
# Radio settings
# -----------------------------

RADIO_A_PORT = "COM3"
RADIO_B_PORT = "COM4"

BAUD_RATE = 57600


# -----------------------------
# Open both radios
# -----------------------------

print("Opening Radio A on", RADIO_A_PORT)
radio_a = serial.Serial(RADIO_A_PORT, BAUD_RATE, timeout=2)

print("Opening Radio B on", RADIO_B_PORT)
radio_b = serial.Serial(RADIO_B_PORT, BAUD_RATE, timeout=2)

# Give the radios a moment to initialize.
time.sleep(2)

print("\nBoth radios opened successfully.\n")


# -----------------------------
# Test Radio A -> Radio B
# -----------------------------

message_a = b"VENATOR TEST: Radio A to Radio B\n"

# Remove any old data waiting in Radio B's receive buffer.
radio_b.reset_input_buffer()

print("Radio A sending:")
print(message_a.decode().strip())

radio_a.write(message_a)
radio_a.flush()

received_b = radio_b.readline()

if received_b:
    print("Radio B received:")
    print(received_b.decode(errors="replace").strip())
else:
    print("FAILED: Radio B received nothing.")


# -----------------------------
# Test Radio B -> Radio A
# -----------------------------

print("\n------------------------------\n")

message_b = b"VENATOR TEST: Radio B to Radio A\n"

# Remove any old data waiting in Radio A's receive buffer.
radio_a.reset_input_buffer()

print("Radio B sending:")
print(message_b.decode().strip())

radio_b.write(message_b)
radio_b.flush()

received_a = radio_a.readline()

if received_a:
    print("Radio A received:")
    print(received_a.decode(errors="replace").strip())
else:
    print("FAILED: Radio A received nothing.")


# -----------------------------
# Close both COM ports
# -----------------------------

radio_a.close()
radio_b.close()

print("\nRadio test complete.")