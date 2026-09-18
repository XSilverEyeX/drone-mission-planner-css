"""
VENATOR Desktop Launcher

Starts the existing VENATOR ASP.NET web application
and displays it inside a desktop window.

This lets the GCS run like a normal desktop application
without manually opening a web browser.
"""

import subprocess
import sys
import time
import urllib.request
from pathlib import Path

from PyQt6.QtCore import QUrl
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.QtWebEngineWidgets import QWebEngineView


# ============================================================
# SETTINGS
# ============================================================

# Address used by the local VENATOR web server
VENATOR_URL = "http://localhost:5035"

# Maximum amount of time to wait for the server
STARTUP_TIMEOUT = 15


# ============================================================
# FIND PROJECT
# ============================================================

# Location of this Python file
APP_FOLDER = Path(__file__).resolve().parent

# Main repository folder
REPO_FOLDER = APP_FOLDER.parent

# Existing ASP.NET project
WEB_PROJECT = (
    REPO_FOLDER
    / "csharp"
    / "DroneNavigation.Web"
    / "DroneNavigation.Web.csproj"
)


# ============================================================
# VENATOR WINDOW
# ============================================================

class VenatorWindow(QMainWindow):
    """
    Main desktop window for the VENATOR GCS.
    """

    def __init__(self):
        super().__init__()

        # Window title
        self.setWindowTitle(
            "VENATOR Ground Control Station"
        )

        # Starting window size
        self.resize(1400, 850)

        # Create the built-in web view
        self.browser = QWebEngineView()

        # Load the existing VENATOR interface
        self.browser.setUrl(
            QUrl(VENATOR_URL)
        )

        # Fill the window with the GUI
        self.setCentralWidget(
            self.browser
        )


# ============================================================
# START ASP.NET SERVER
# ============================================================

def start_server():
    """
    Starts the existing C# backend.
    """

    print("[VENATOR] Starting backend...")

    # Make sure the C# project exists
    if not WEB_PROJECT.exists():

        raise FileNotFoundError(
            f"Could not find:\n{WEB_PROJECT}"
        )

    # Start the same command we normally run manually
    process = subprocess.Popen(
        [
            "dotnet",
            "run",
            "--project",
            str(WEB_PROJECT),
            "--urls",
            VENATOR_URL
        ],
        cwd=REPO_FOLDER
    )

    return process


# ============================================================
# WAIT FOR SERVER
# ============================================================

def wait_for_server():
    """
    Waits until ASP.NET is ready before opening the GUI.
    """

    print("[VENATOR] Waiting for backend...")

    start_time = time.time()

    while time.time() - start_time < STARTUP_TIMEOUT:

        try:

            # Use the health endpoint already in Program.cs
            with urllib.request.urlopen(
                VENATOR_URL + "/api/health",
                timeout=1
            ) as response:

                if response.status == 200:

                    print(
                        "[VENATOR] Backend ready."
                    )

                    return True

        except Exception:

            # Server is still starting
            time.sleep(0.25)

    return False


# ============================================================
# MAIN PROGRAM
# ============================================================

def main():

    server_process = None

    try:

        # Start the existing C# backend
        server_process = start_server()

        # Wait for ASP.NET to finish starting
        if not wait_for_server():

            print(
                "[VENATOR] ERROR: "
                "Backend did not start."
            )

            return 1

        # Start PyQt
        app = QApplication(sys.argv)

        # Create the VENATOR desktop window
        window = VenatorWindow()

        # Maximize it for the GCS
        window.showMaximized()

        print(
            "[VENATOR] Ground Control Station ready."
        )

        # Run the GUI
        return app.exec()

    except KeyboardInterrupt:

        print(
            "\n[VENATOR] Closing..."
        )

        return 0

    except Exception as e:

        print(
            f"[VENATOR] ERROR: {e}"
        )

        return 1

    finally:

        # Stop ASP.NET when the desktop application closes
        if server_process is not None:

            print(
                "[VENATOR] Stopping backend..."
            )

            server_process.terminate()

            try:

                server_process.wait(
                    timeout=5
                )

            except subprocess.TimeoutExpired:

                server_process.kill()


# Start the program
if __name__ == "__main__":
    sys.exit(main())