"""
VENATOR Telemetry Demo
venator_demo.py

Standalone terminal demo for testing communication between:
Computer -> SiK Radio -> SiK Radio -> Pixhawk

This program does NOT require:
- ASP.NET
- C#
- localhost website
- Drone Mission Planner
"""

from mavlink_connection import MAVLinkConnection


def print_header():
    """Display the VENATOR demo title."""

    print("\n" + "=" * 50)
    print("           VENATOR TELEMETRY DEMO")
    print("=" * 50)


def select_port(connection):
    """Find COM ports and allow the user to select one."""

    ports = connection.get_serial_ports()

    print("\nAVAILABLE COM PORTS")
    print("-" * 50)

    if not ports:
        print("No COM ports detected.")
        return None

    # Display every COM port Windows currently detects.
    for index, port in enumerate(ports, start=1):
        print(
            f"[{index}] {port['device']} - "
            f"{port['description']}"
        )

    print("[0] Exit")

    while True:
        choice = input("\nSelect COM port: ").strip()

        if choice == "0":
            return None

        try:
            choice_number = int(choice)

            if 1 <= choice_number <= len(ports):
                return ports[choice_number - 1]["device"]

        except ValueError:
            pass

        print("Invalid selection. Try again.")


def telemetry_menu(connection):
    """Menu shown after a Pixhawk connection is established."""

    while True:

        print("\n" + "-" * 50)
        print("TELEMETRY")
        print("-" * 50)

        print(f"COM Port:     {connection.port}")
        print("Radio:        CONNECTED")
        print("Pixhawk:      MAVLink heartbeat received")

        heartbeat_age = connection.heartbeat_age()

        if heartbeat_age is not None:
            print(f"Heartbeat:    {heartbeat_age:.1f} seconds ago")

        print("\nCOMMANDS")
        print("[1] Get GPS position")
        print("[2] Connection status")
        print("[3] Disconnect")
        print("[0] Exit")

        choice = input("\nCommand: ").strip()

        # -----------------------------------------------------
        # GET GPS
        # -----------------------------------------------------

        if choice == "1":

            print("\nWaiting for GPS position...")

            position = connection.get_position()

            if position is None:
                print("No GLOBAL_POSITION_INT message received.")

            else:
                print("\nGPS POSITION")
                print("-" * 50)

                print(
                    f"Latitude:     "
                    f"{position['lat']:.7f}"
                )

                print(
                    f"Longitude:    "
                    f"{position['lon']:.7f}"
                )

                print(
                    f"Altitude:     "
                    f"{position['altitude']:.2f} m"
                )

        # -----------------------------------------------------
        # CONNECTION STATUS
        # -----------------------------------------------------

        elif choice == "2":

            print("\nCONNECTION STATUS")
            print("-" * 50)

            print(
                f"Radio:        "
                f"{'CONNECTED' if connection.is_radio_connected() else 'DISCONNECTED'}"
            )

            print(
                f"COM Port:     "
                f"{connection.port or 'None'}"
            )

            print(
                f"Baud Rate:    "
                f"{connection.baud or 'None'}"
            )

            heartbeat_age = connection.heartbeat_age()

            if heartbeat_age is None:
                print("Heartbeat:    None")

            else:
                print(
                    f"Heartbeat:    "
                    f"{heartbeat_age:.1f} seconds ago"
                )

        # -----------------------------------------------------
        # DISCONNECT
        # -----------------------------------------------------

        elif choice == "3":

            connection.disconnect()

            print("\nDisconnected from telemetry.")
            return "disconnect"

        # -----------------------------------------------------
        # EXIT
        # -----------------------------------------------------

        elif choice == "0":

            connection.disconnect()

            print("\nTelemetry connection closed.")
            return "exit"

        else:
            print("Invalid command.")


def main():
    """Main VENATOR telemetry demo."""

    connection = MAVLinkConnection()

    while True:

        print_header()

        # Find and select a Windows COM port.
        selected_port = select_port(connection)

        if selected_port is None:
            print("\nVENATOR demo closed.")
            break

        print(f"\nSelected: {selected_port}")

        # Attempt to open the SiK radio and receive
        # a MAVLink heartbeat from the Pixhawk.
        result = connection.connect(
            selected_port,
            baud=57600,
            heartbeat_timeout=10
        )

        print("\nCONNECTION RESULT")
        print("-" * 50)

        if result["radio_connected"]:
            print("Radio:        CONNECTED")
        else:
            print("Radio:        DISCONNECTED")

        if result["pixhawk_connected"]:
            print("Pixhawk:      CONNECTED")
        else:
            print("Pixhawk:      NO HEARTBEAT")

        print(f"Message:      {result['message']}")

        # Only enter the telemetry menu after receiving
        # an actual MAVLink heartbeat.
        if result["success"]:

            print(
                f"System ID:    "
                f"{result['system_id']}"
            )

            print(
                f"Component ID: "
                f"{result['component_id']}"
            )

            action = telemetry_menu(connection)

            if action == "exit":
                break

        else:
            # Close the attempted connection before allowing
            # another COM port to be selected.
            connection.disconnect()

            input(
                "\nPress Enter to return to COM-port selection..."
            )


if __name__ == "__main__":
    main()