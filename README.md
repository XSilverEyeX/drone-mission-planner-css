# VENATOR Ground Control Station

VENATOR is a ground control interface for the team's PX4-based UAV. The application includes a web dashboard, local telemetry bridge, and path-planning tools for testing and development.

## Project files

| File | Description |
| --- | --- |
| `csharp/DroneNavigation.Web/wwwroot/index.html` | Dashboard, map, telemetry displays, and flight controls |
| `desktop_app/desktop_app.py` | Desktop window for the GUI |
| `csharp/DroneNavigation.Web/Program.cs` | Web API and path-planning requests |
| `csharp/DroneNavigation/Drone/DroneClient.cs` | Communication between the API and Python bridge |
| `python_bridge/drone_bridge.py` | PX4 MAVLink telemetry and command handling |
| `csharp/DroneNavigation/PathPlanning/` | A* path planning, occupancy grid, and route smoothing |
| `csharp/DroneNavigation/QuadTree/` | Obstacle indexing |
| `python_bridge/test_drone_bridge.py` | PX4 bridge tests |

## Running the GUI

The web interface requires the .NET 10 SDK and Python with `pymavlink` and `pyserial`.

Start the Python bridge from the project folder:

```powershell
python -m pip install pymavlink pyserial
python .\python_bridge\drone_bridge.py
```

Open another terminal in the same folder and start the web application:

```powershell
dotnet run --project .\csharp\DroneNavigation.Web
```

Open the local address shown in the terminal. Select the telemetry port in the GUI when connecting to a Pixhawk.

The optional desktop window can be started with `python .\desktop_app\desktop_app.py` after installing `PyQt6` and `PyQt6-WebEngine`.

## Current work

The GUI includes telemetry monitoring, an indoor sensor dashboard, flight mode selection, and an interactive map. The C# API forwards requests to the Python MAVLink bridge. The A* planner can calculate paths around obstacles.

Testing and PX4 setup notes are in [docs/PX4_DEMO.md](docs/PX4_DEMO.md). The interface is still being developed and should not be treated as a validated autonomous flight controller.

## License

MIT — see [LICENSE](LICENSE).
