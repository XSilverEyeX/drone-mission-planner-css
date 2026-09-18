using System.Text.Json;
using DroneNavigation.Drone;
using DroneNavigation.Geometry;
using DroneNavigation.Obstacles;
using DroneNavigation.PathPlanning;
using DroneNavigation.QuadTree;
using DroneNavigation.Web.Models;


// Creates the ASP.NET web application
var builder = WebApplication.CreateBuilder(args);


// ---------- CORS ----------

// Allows the website to communicate with the backend
// This is mainly useful while developing and testing
builder.Services.AddCors(options =>
{
    options.AddDefaultPolicy(policy =>
        policy.AllowAnyOrigin()
              .AllowAnyMethod()
              .AllowAnyHeader());
});


// ---------- Drone client ----------

// Creates one shared DroneClient for the whole application
// DroneClient handles communication between C# and Python
builder.Services.AddSingleton<DroneClient>();


var app = builder.Build();


// Enables CORS and allows the website files to be loaded
app.UseCors();
app.UseDefaultFiles();
app.UseStaticFiles();



// ============================================================
// PATH PLANNER SETTINGS
// ============================================================

// Size of the local planning area in meters
const double WorldSize = 100.0;

// Size of each grid cell used by the path planner
const double CellSize = 4.0;

// Extra distance the drone should stay away from obstacles
const double SafetyBufferMeters = 3.0;



// ============================================================
// PLAN PATH
// ============================================================

// Receives the obstacles, start, goal, and altitude from the GUI
// and calculates a safe path using the A* path planner
app.MapPost("/api/plan", (PlanRequest req) =>
{

    // Create the QuadTree that stores the obstacles
    QuadTree tree = new QuadTree(
        new Rectangle(
            0,
            0,
            WorldSize,
            WorldSize
        )
    );


    // Add each obstacle from the GUI into the QuadTree
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


    // Create the grid that A* will search through
    Grid grid = new Grid(
        tree,
        new Rectangle(
            0,
            0,
            WorldSize,
            WorldSize
        ),
        cellSize: CellSize,
        droneAltitudeMeters: req.AltitudeMeters,
        safetyBufferMeters: SafetyBufferMeters
    );


    // Get the start point sent from the GUI
    Point start = new Point(
        req.Start.X,
        req.Start.Y
    );


    // Get the goal point sent from the GUI
    Point goal = new Point(
        req.Goal.X,
        req.Goal.Y
    );


    // Use A* to calculate a path around the obstacles
    List<Point> rawPath = AStar.FindPath(
        grid,
        start,
        goal
    );


    // If A* could not find a safe route
    if (rawPath.Count == 0)
    {
        return Results.Ok(
            new PlanResponse
            {
                Success = false,

                Message =
                    "No path found — obstacles block all routes",

                Waypoints =
                    new List<PointDto>(),

                RawWaypointCount = 0
            }
        );
    }


    // A* can create more points than we actually need
    // This removes unnecessary waypoints and smooths the path
    List<Point> smoothed = PathSmoother.Smooth(
        grid,
        rawPath
    );


    // Send the final path back to the GUI
    return Results.Ok(
        new PlanResponse
        {
            Success = true,

            Message =
                $"Planned {smoothed.Count} waypoints " +
                $"(raw {rawPath.Count})",

            Waypoints = smoothed
                .Select(p =>
                    new PointDto
                    {
                        X = p.X,
                        Y = p.Y
                    }
                )
                .ToList(),

            RawWaypointCount =
                rawPath.Count
        }
    );
});



// ============================================================
// TELEMETRY COM PORTS
// ============================================================

// The GUI calls this endpoint to get the COM ports
// detected by the Python bridge.
//
// Communication:
// GUI -> Program.cs -> DroneClient.cs -> Python -> Windows
app.MapGet(
    "/api/telemetry/ports",
    (DroneClient client) =>
{
    try
    {

        // Connect C# to the Python bridge if needed
        //
        // This does NOT mean the Pixhawk is connected.
        // It only means C# can communicate with Python.
        if (!client.IsConnected)
        {
            client.Connect();
        }


        // Ask Python for the available Windows COM ports
        DroneResponse response =
            client.GetPorts();


        // Make sure Python returned valid data
        if (
            !response.IsOk ||
            !response.Data.HasValue
        )
        {
            return Results.Ok(
                new
                {
                    success = false,

                    message =
                        response.Message,

                    ports =
                        Array.Empty<object>()
                }
            );
        }


        // Get the JSON data returned from Python
        JsonElement data =
            response.Data.Value;


        // Make sure the response actually contains a ports list
        if (
            !data.TryGetProperty(
                "ports",
                out JsonElement portsElement
            )
        )
        {
            return Results.Ok(
                new
                {
                    success = false,

                    message =
                        "Bridge response did not contain a ports list",

                    ports =
                        Array.Empty<object>()
                }
            );
        }


        // Convert the JSON port information into C# objects
        // that can easily be sent back to JavaScript
        var ports =
            portsElement
                .EnumerateArray()
                .Select(port =>
                    new
                    {
                        device =
                            port
                                .GetProperty("device")
                                .GetString(),

                        description =
                            port
                                .GetProperty("description")
                                .GetString()
                    }
                )
                .ToList();


        // Send the COM-port list back to the GUI
        return Results.Ok(
            new
            {
                success = true,
                ports
            }
        );
    }

    catch (Exception ex)
    {

        // If the Python bridge is not running or another error occurs
        return Results.Ok(
            new
            {
                success = false,
                message = ex.Message,
                ports = Array.Empty<object>()
            }
        );
    }
});



// ============================================================
// FLY MISSION
// ============================================================

// Receives the planned waypoints from the GUI
// and sends the mission commands through DroneClient
app.MapPost(
    "/api/fly",
    async (
        FlyRequest req,
        DroneClient client
    ) =>
{

    // A mission needs at least a start and goal point
    if (
        req.Waypoints == null ||
        req.Waypoints.Count < 2
    )
    {
        return Results.BadRequest(
            new
            {
                error =
                    "Need at least 2 waypoints (start + goal)"
            }
        );
    }


    try
    {

        // Connect C# to Python if it is not already connected
        if (!client.IsConnected)
        {
            client.Connect();
        }


        // Run the mission in the background
        // This prevents the website request from staying open
        // during the entire flight
        _ = Task.Run(() =>
        {
            try
            {

                // Tell the Python bridge to save the home position
                client.SetHome();


                // Arm the drone
                client.Arm();


                // Take off to the altitude selected in the GUI
                client.Takeoff(
                    req.AltitudeMeters
                );


                // Treat the first waypoint as the local starting point
                PointDto startPoint =
                    req.Waypoints[0];


                // Send the rest of the waypoints one at a time
                for (
                    int i = 1;
                    i < req.Waypoints.Count;
                    i++
                )
                {
                    PointDto p =
                        req.Waypoints[i];


                    // Find how far east/west the waypoint is
                    // from the starting position
                    double xOffset =
                        p.X -
                        startPoint.X;


                    // Find how far north/south the waypoint is
                    // from the starting position
                    double yOffset =
                        p.Y -
                        startPoint.Y;


                    // Send the local waypoint to Python
                    client.GotoLocal(
                        xOffset,
                        yOffset,
                        req.AltitudeMeters
                    );
                }


                // Land after all waypoints have been completed
                client.Land();
            }

            catch (Exception ex)
            {

                // Print mission errors in the server terminal
                Console.WriteLine(
                    $"[mission] Error: {ex.Message}"
                );
            }
        });


        // Tell the GUI that the mission was started
        return Results.Ok(
            new
            {
                status = "started",
                waypoints =
                    req.Waypoints.Count
            }
        );
    }

    catch (Exception ex)
    {

        return Results.Problem(
            $"Could not start mission: {ex.Message}"
        );
    }
});



// ============================================================
// ABORT / LAND
// ============================================================

// The GUI calls this endpoint when the Abort button is pressed
// It sends a LAND command through the Python bridge
app.MapPost(
    "/api/abort",
    (DroneClient client) =>
{
    try
    {

        // Make sure C# is connected to Python
        if (!client.IsConnected)
        {
            client.Connect();
        }


        // Send the LAND command
        DroneResponse response =
            client.Land();


        // Send the result back to the GUI
        return Results.Ok(
            new
            {
                status =
                    response.Status,

                message =
                    response.Message
            }
        );
    }

    catch (Exception ex)
    {

        return Results.Problem(
            $"Abort failed: {ex.Message}"
        );
    }
});



// ============================================================
// DRONE POSITION
// ============================================================

// Temporary home location from the SITL simulator
//
// These coordinates will eventually be replaced with the
// actual GPS/home position received from the Pixhawk.
const double HomeLat =
    -35.363262;

const double HomeLon =
    149.165237;

const double EarthRadiusM =
    6_378_137.0;


// The GUI calls this endpoint every second
// to get the drone's current position
app.MapGet(
    "/api/status",
    (DroneClient client) =>
{
    try
    {

        // Make sure C# can communicate with Python
        if (!client.IsConnected)
        {
            client.Connect();
        }


        // Ask Python for the latest drone position
        DroneResponse response =
            client.GetPosition();


        // If Python does not have a valid position yet
        if (
            !response.IsOk ||
            !response.Data.HasValue
        )
        {
            return Results.Ok(
                new
                {
                    connected = false,
                    message =
                        response.Message
                }
            );
        }


        // Get the position information returned from Python
        JsonElement data =
            response.Data.Value;


        double lat =
            data
                .GetProperty("lat")
                .GetDouble();


        double lon =
            data
                .GetProperty("lon")
                .GetDouble();


        double altitude =
            data
                .GetProperty("altitude")
                .GetDouble();


        // Convert GPS latitude into meters north/south
        // from the home position
        double yMeters =
            (lat - HomeLat)
            * Math.PI
            / 180.0
            * EarthRadiusM;


        // Convert GPS longitude into meters east/west
        // from the home position
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


        // Send both GPS and local coordinates to the GUI
        return Results.Ok(
            new
            {
                connected = true,

                lat,
                lon,
                altitude,

                // X = meters east/west of home
                // Y = meters north/south of home
                xLocal = xMeters,
                yLocal = yMeters
            }
        );
    }

    catch (Exception ex)
    {

        // If the bridge or drone is unavailable
        return Results.Ok(
            new
            {
                connected = false,
                message = ex.Message
            }
        );
    }
});



// ============================================================
// HEALTH CHECK
// ============================================================

// Simple test to make sure the ASP.NET server is running
// This does not send anything to the drone
app.MapGet(
    "/api/health",
    () =>
        Results.Ok(
            new
            {
                status = "ok",
                timestamp =
                    DateTime.UtcNow
            }
        )
);



// ============================================================
// START SERVER
// ============================================================

// Starts the ASP.NET web server
app.Run();