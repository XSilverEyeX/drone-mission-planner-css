"""
VENATOR SiK Radio-to-Radio Test

Tests communication between two SiK telemetry radios.

Both radios are connected to the same computer using USB.
The program sends a message in both directions to make sure
the radios can communicate with each other.

Radio A: COM3
Radio B: COM4
"""

import serial
import time


# ============================================================
# RADIO SETTINGS
# ============================================================

# COM ports assigned to each SiK radio
RADIO_A_PORT = "COM3"
RADIO_B_PORT = "COM4"

# Serial baud rate used by both radios
BAUD_RATE = 57600


# ============================================================
# OPEN RADIOS
# ============================================================

# Open the serial connection to Radio A
print("Opening Radio A on", RADIO_A_PORT)

radio_a = serial.Serial(
    RADIO_A_PORT,
    BAUD_RATE,
    timeout=2
)


# Open the serial connection to Radio B
print("Opening Radio B on", RADIO_B_PORT)

radio_b = serial.Serial(
    RADIO_B_PORT,
    BAUD_RATE,
    timeout=2
)


# Give both radios time to initialize
time.sleep(2)

print("\nBoth radios opened successfully.\n")


# ============================================================
# TEST RADIO A -> RADIO B
# ============================================================

# Message that Radio A will send
message_a = b"VENATOR TEST: Radio A to Radio B\n"


# Clear any old data waiting in Radio B
radio_b.reset_input_buffer()


print("Radio A sending:")
print(message_a.decode().strip())


# Send the message through Radio A
radio_a.write(message_a)
radio_a.flush()


# Wait for Radio B to receive the message
received_b = radio_b.readline()


# Check if Radio B received anything
if received_b:

    print("Radio B received:")

    print(
        received_b
        .decode(errors="replace")
        .strip()
    )

else:

    print(
        "FAILED: Radio B received nothing."
    )


# ============================================================
# TEST RADIO B -> RADIO A
# ============================================================

print("\n------------------------------\n")


# Message that Radio B will send
message_b = b"VENATOR TEST: Radio B to Radio A\n"


# Clear any old data waiting in Radio A
radio_a.reset_input_buffer()


print("Radio B sending:")
print(message_b.decode().strip())


# Send the message through Radio B
radio_b.write(message_b)
radio_b.flush()


# Wait for Radio A to receive the message
received_a = radio_a.readline()


# Check if Radio A received anything
if received_a:

    print("Radio A received:")

    print(
        received_a
        .decode(errors="replace")
        .strip()
    )

else:

    print(
        "FAILED: Radio A received nothing."
    )


# ============================================================
# CLOSE RADIOS
# ============================================================

# Close both serial connections when the test is finished
radio_a.close()
radio_b.close()


print("\nRadio test complete.")