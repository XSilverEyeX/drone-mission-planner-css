# VENATOR Ground Control Station — Commented Submission

VENATOR is a PX4-oriented drone ground-control interface developed as part of a senior design integration effort. The project combines a web dashboard, an ASP.NET Core API, a local Python MAVLink bridge, and obstacle-aware path-planning components.

This submission branch adds source-code comments and a presentation-oriented README. **It does not claim that flight behavior has been newly validated or that any command safeguards have been changed.**

## Main components

| Component | Path | Purpose |
| --- | --- | --- |
| GUI and dashboard | `csharp/DroneNavigation.Web/wwwroot/index.html` | Leaflet map, controls, telemetry displays, and client-side interactions |
| Desktop window | `desktop_app/desktop_app.py` | Optional PyQt6 wrapper around the HTML interface |
| Web API | `csharp/DroneNavigation.Web/Program.cs` | Local HTTP endpoints for planning and command forwarding |
| API types | `csharp/DroneNavigation.Web/Models/Dtos.cs` | Data contracts for requests and responses |
| Bridge client | `csharp/DroneNavigation/Drone/DroneClient.cs` | JSON-over-TCP bridge client |
| PX4 bridge | `python_bridge/drone_bridge.py` | MAVLink telemetry, status, acknowledgements, and safety-gated requests |
| Planning | `csharp/DroneNavigation/PathPlanning/` | Grid, A* search, route smoothing |
| Spatial indexing | `csharp/DroneNavigation/QuadTree/` | Quadtree obstacle queries |
| Tests | `python_bridge/test_drone_bridge.py`, `csharp/DroneNavigation.Tests/` | Software regression checks |

## Running the local dashboard

Prerequisites: .NET 10 SDK, Python, and Python packages `pymavlink` and `pyserial`. Use a controlled bench setup before any hardware connection. Keep propellers removed during bench checks.

In the repository root, open two terminals:

```powershell
python -m pip install pymavlink pyserial
python .\python_bridge\drone_bridge.py
```

In the second terminal:

```powershell
dotnet run --project .\csharp\DroneNavigation.Web
```

Open the local URL printed by ASP.NET Core. Use the interface to select the correct telemetry port. The code does not assume a specific COM port or automatically arm the aircraft.

For the current PX4 status, safety gates, and demonstration scope, read [PX4_DEMO.md](docs/PX4_DEMO.md) and [VENATOR_SESSION_NOTES.md](docs/VENATOR_SESSION_NOTES.md).

## Showcase flow

1. Present the dashboard and interactive map.
2. Explain the browser-to-C# REST/API request flow.
3. Show the C#-to-Python JSON/TCP interface.
4. Explain how the MAVLink bridge receives PX4 telemetry and handles acknowledgements.
5. Show A* path planning and obstacle detection in code.
6. Distinguish tested simulation and props-off bench behavior from unverified live flight.

## Attribution and license

This source incorporates existing project components. See the retained [MIT LICENSE](LICENSE) for the original copyright and licensing notice. No ownership or exclusive authorship is implied by this submission copy.
