using System.Text.Json;
using DroneNavigation.Drone;
using DroneNavigation.Geometry;
using DroneNavigation.Obstacles;
using DroneNavigation.PathPlanning;
using DroneNavigation.QuadTree;
using DroneNavigation.Web.Models;

var builder = WebApplication.CreateBuilder(args);

// Allow the browser frontend to call us from a different origin during development.
// In production this should be restricted to specific origins.
builder.Services.AddCors(options =>
{
    options.AddDefaultPolicy(policy =>
        policy.AllowAnyOrigin()
              .AllowAnyMethod()
              .AllowAnyHeader());
});

// Register one DroneClient that is shared across API requests.
// This maintains the TCP connection between C# and the Python bridge.
builder.Services.AddSingleton<DroneClient>();

var app = builder.Build();

app.UseCors();
app.UseDefaultFiles();
app.UseStaticFiles();


// ============================================================
// WORLD CONFIGURATION
// ============================================================

// Same coordinate system used by the existing path planner.
const double WorldSize = 100.0;
const double CellSize = 4.0;
const double SafetyBufferMeters = 3.0;


// ============================================================
// PLAN PATH
// ============================================================

// Given obstacles, start, goal, and altitude,
// calculate a safe path for the drone.
app.MapPost("/api/plan", (PlanRequest req) =>
{
    QuadTree tree = new QuadTree(
        new Rectangle(0, 0, WorldSize, WorldSize)
    );

    // Add all obstacles from the website into the QuadTree.
    foreach (ObstacleDto o in req.Obstacles)
    {
        tree.Insert(
            new Obstacle(
                o.X,
                o.Y,
                o.Radius,
                o.Height
            )
        );
    }

    // Build the grid used by the A* path planner.
    Grid grid = new Grid(
        tree,
        new Rectangle(0, 0, WorldSize, WorldSize),
        cellSize: CellSize,
        droneAltitudeMeters: req.AltitudeMeters,
        safetyBufferMeters: SafetyBufferMeters
    );

    Point start = new Point(
        req.Start.X,
        req.Start.Y
    );

    Point goal = new Point(
        req.Goal.X,
        req.Goal.Y
    );

    // Find the raw A* path.
    List<Point> rawPath = AStar.FindPath(
        grid,
        start,
        goal
    );

    // No valid path was found.
    if (rawPath.Count == 0)
    {
        return Results.Ok(new PlanResponse
        {
            Success = false,
            Message = "No path found — obstacles block all routes",
            Waypoints = new List<PointDto>(),
            RawWaypointCount = 0
        });
    }

    // Remove unnecessary waypoints from the raw path.
    List<Point> smoothed = PathSmoother.Smooth(
        grid,
        rawPath
    );

    return Results.Ok(new PlanResponse
    {
        Success = true,
        Message =
            $"Planned {smoothed.Count} waypoints (raw {rawPath.Count})",

        Waypoints = smoothed
            .Select(p => new PointDto
            {
                X = p.X,
                Y = p.Y
            })
            .ToList(),

        RawWaypointCount = rawPath.Count
    });
});


// ============================================================
// TELEMETRY - AVAILABLE COM PORTS
// ============================================================

// Ask the Python bridge which Windows COM ports currently exist.
//
// Browser
//    ↓
// Program.cs
//    ↓
// DroneClient.cs
//    ↓
// Python bridge
//    ↓
// pyserial
//    ↓
// Windows COM ports
app.MapGet("/api/telemetry/ports", (DroneClient client) =>
{
    try
    {
        // This only connects C# to the Python bridge.
        // It does NOT mean that a Pixhawk is connected.
        if (!client.IsConnected)
        {
            client.Connect();
        }

        // Ask Python for the current serial-port list.
        DroneResponse response = client.GetPorts();

        if (!response.IsOk || !response.Data.HasValue)
        {
            return Results.Ok(new
            {
                success = false,
                message = response.Message,
                ports = Array.Empty<object>()
            });
        }

        JsonElement data = response.Data.Value;

        // Make sure Python actually returned a "ports" array.
        if (!data.TryGetProperty(
                "ports",
                out JsonElement portsElement))
        {
            return Results.Ok(new
            {
                success = false,
                message =
                    "Bridge response did not contain a ports list",
                ports = Array.Empty<object>()
            });
        }

        // Convert Python's JSON array into normal C# objects.
        // This makes the response easy for JavaScript to use.
        var ports = portsElement
            .EnumerateArray()
            .Select(port => new
            {
                device =
                    port.GetProperty("device").GetString(),

                description =
                    port.GetProperty("description").GetString()
            })
            .ToList();

        return Results.Ok(new
        {
            success = true,
            ports
        });
    }
    catch (Exception ex)
    {
        return Results.Ok(new
        {
            success = false,
            message = ex.Message,
            ports = Array.Empty<object>()
        });
    }
});


// ============================================================
// FLY MISSION
// ============================================================

// Execute the path created by the planner.
//
// NOTE:
// The Python bridge currently decides how these MAVLink commands
// are sent to the flight controller.
app.MapPost("/api/fly", async (FlyRequest req, DroneClient client) =>
{
    if (req.Waypoints == null || req.Waypoints.Count < 2)
    {
        return Results.BadRequest(new
        {
            error = "Need at least 2 waypoints (start + goal)"
        });
    }

    try
    {
        // Connect C# to Python if necessary.
        if (!client.IsConnected)
        {
            client.Connect();
        }

        // Run the mission in the background so the HTTP request
        // does not remain open for the entire flight.
        _ = Task.Run(() =>
        {
            try
            {
                // Establish the starting location.
                client.SetHome();

                // Arm the vehicle.
                client.Arm();

                // Take off to the selected cruise altitude.
                client.Takeoff(req.AltitudeMeters);

                // The first waypoint is treated as our local origin.
                PointDto startPoint = req.Waypoints[0];

                // Send every remaining waypoint to the Python bridge.
                for (int i = 1; i < req.Waypoints.Count; i++)
                {
                    PointDto p = req.Waypoints[i];

                    double xOffset =
                        p.X - startPoint.X;

                    double yOffset =
                        p.Y - startPoint.Y;

                    client.GotoLocal(
                        xOffset,
                        yOffset,
                        req.AltitudeMeters
                    );
                }

                // Land after completing all waypoints.
                client.Land();
            }
            catch (Exception ex)
            {
                Console.WriteLine(
                    $"[mission] Error: {ex.Message}"
                );
            }
        });

        return Results.Ok(new
        {
            status = "started",
            waypoints = req.Waypoints.Count
        });
    }
    catch (Exception ex)
    {
        return Results.Problem(
            $"Could not start mission: {ex.Message}"
        );
    }
});


// ============================================================
// ABORT / LAND NOW
// ============================================================

// Immediately request LAND from the Python bridge.
app.MapPost("/api/abort", (DroneClient client) =>
{
    try
    {
        if (!client.IsConnected)
        {
            client.Connect();
        }

        DroneResponse response = client.Land();

        return Results.Ok(new
        {
            status = response.Status,
            message = response.Message
        });
    }
    catch (Exception ex)
    {
        return Results.Problem(
            $"Abort failed: {ex.Message}"
        );
    }
});


// ============================================================
// DRONE STATUS / POSITION
// ============================================================

// Current temporary home location used by Sergio's SITL setup.
//
// IMPORTANT:
// These coordinates are still the Canberra SITL location.
// Later we will replace this with the real Pixhawk/home GPS
// instead of keeping it hard-coded.
const double HomeLat = -35.363262;
const double HomeLon = 149.165237;
const double EarthRadiusM = 6_378_137.0;


// The website polls this endpoint to get the drone's position.
app.MapGet("/api/status", (DroneClient client) =>
{
    try
    {
        if (!client.IsConnected)
        {
            client.Connect();
        }

        DroneResponse response =
            client.GetPosition();

        // No Pixhawk / position available yet.
        if (!response.IsOk || !response.Data.HasValue)
        {
            return Results.Ok(new
            {
                connected = false,
                message = response.Message
            });
        }

        JsonElement data =
            response.Data.Value;

        double lat =
            data.GetProperty("lat").GetDouble();

        double lon =
            data.GetProperty("lon").GetDouble();

        double altitude =
            data.GetProperty("altitude").GetDouble();

        // Convert GPS coordinates into the local X/Y coordinate
        // system used by the map and path planner.
        double yMeters =
            (lat - HomeLat)
            * Math.PI
            / 180.0
            * EarthRadiusM;

        double xMeters =
            (lon - HomeLon)
            * Math.PI
            / 180.0
            * EarthRadiusM
            * Math.Cos(
                HomeLat
                * Math.PI
                / 180.0
            );

        return Results.Ok(new
        {
            connected = true,

            lat,
            lon,
            altitude,

            // Local frame:
            // X = meters east of home
            // Y = meters north of home
            xLocal = xMeters,
            yLocal = yMeters
        });
    }
    catch (Exception ex)
    {
        return Results.Ok(new
        {
            connected = false,
            message = ex.Message
        });
    }
});


// ============================================================
// HEALTH CHECK
// ============================================================

// Simple endpoint used to verify that the ASP.NET server itself
// is running without sending anything to the drone.
app.MapGet("/api/health", () =>
    Results.Ok(new
    {
        status = "ok",
        timestamp = DateTime.UtcNow
    })
);


// ============================================================
// START SERVER
// ============================================================

app.Run();