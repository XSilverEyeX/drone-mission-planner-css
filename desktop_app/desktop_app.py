"""
VENATOR Ground Control Station
Desktop GUI Launcher

This program opens the VENATOR interface directly inside
a PyQt6 desktop window.
"""

import sys
from pathlib import Path

from PyQt6.QtCore import QUrl
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.QtWebEngineWidgets import QWebEngineView


# ============================================================
# FIND THE VENATOR GUI
# ============================================================

# Folder containing desktop_app.py
APP_FOLDER = Path(__file__).resolve().parent

# Main project folder
REPO_FOLDER = APP_FOLDER.parent

# Location of the VENATOR HTML interface
INDEX_FILE = (
    REPO_FOLDER
    / "csharp"
    / "DroneNavigation.Web"
    / "wwwroot"
    / "index.html"
)


# ============================================================
# VENATOR WINDOW
# ============================================================

class VenatorWindow(QMainWindow):

    def __init__(self):
        super().__init__()

        # Name shown at the top of the window
        self.setWindowTitle(
            "VENATOR Ground Control Station"
        )

        # Starting window size
        self.resize(1400, 850)

        # Create a web view inside the PyQt window
        self.browser = QWebEngineView()

        # Make sure index.html exists
        if not INDEX_FILE.exists():

            raise FileNotFoundError(
                f"Could not find GUI:\n{INDEX_FILE}"
            )

        print(
            "[VENATOR] Loading:",
            INDEX_FILE
        )

        # Load index.html directly into the desktop window
        self.browser.setUrl(
            QUrl.fromLocalFile(
                str(INDEX_FILE)
            )
        )

        # Make the GUI fill the window
        self.setCentralWidget(
            self.browser
        )


# ============================================================
# START APPLICATION
# ============================================================

def main():

    print(
        "[VENATOR] Starting Ground Control Station..."
    )

    # Create the PyQt application
    app = QApplication(sys.argv)

    # Create our VENATOR window
    window = VenatorWindow()

    # Open it maximized
    window.showMaximized()

    print(
        "[VENATOR] GUI ready."
    )

    # Keep the GUI running
    return app.exec()


# ============================================================
# RUN
# ============================================================

if __name__ == "__main__":
    sys.exit(main())