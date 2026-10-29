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
// ---------- Drone client ----------

// Creates one shared DroneClient for the whole application
// DroneClient handles communication between C# and Python
builder.Services.AddSingleton(new DroneClient(port: builder.Configuration.GetValue<int?>("Bridge:Port") ?? 5555));


var app = builder.Build();


// Enables CORS and allows the website files to be loaded

// Vehicle APIs are for the local operator UI. Block cross-site browser mutations.
app.Use(async (context, next) =>
{
    if (HttpMethods.IsPost(context.Request.Method))
    {
        var origin = context.Request.Headers.Origin.ToString();
        var expected = $"{context.Request.Scheme}://{context.Request.Host}";
        if (context.Request.Headers["Sec-Fetch-Site"] == "cross-site" ||
            (!string.IsNullOrEmpty(origin) && !string.Equals(origin, expected, StringComparison.OrdinalIgnoreCase)))
        { context.Response.StatusCode = 403; return; }
    }
    await next();
});
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
// Vehicle endpoints relay explicit actions only; no background flight sequence.
IResult Relay(DroneClient client, object command)
{
    try
    {
        var reply = client.Send(command);
        return reply.IsOk ? Results.Ok(reply) : Results.BadRequest(reply);
    }
    catch (Exception ex)
    {
        return Results.Json(new { status = "error", message = $"Bridge unavailable or response lost; command outcome may be unknown: {ex.Message}" }, statusCode: 503);
    }
}
app.MapGet("/api/telemetry/ports", (DroneClient client) =>
{
    try
    {
        var reply = client.GetPorts();
        return Results.Ok(new { success = reply.IsOk, ports = reply.Data?.GetProperty("ports"), message = reply.Message });
    }
    catch (Exception ex) { return Results.Ok(new { success = false, ports = Array.Empty<object>(), message = ex.Message }); }
});
app.MapGet("/api/status", (DroneClient client) =>
{
    try { return Results.Ok(client.GetStatus().Data); }
    catch (Exception ex) { return Results.Ok(new { connected = false, radioConnected = false, blockers = new[] { "Python bridge unavailable" }, message = ex.Message }); }
});
app.MapPost("/api/telemetry/connect", (ConnectionRequest req, DroneClient client) => Relay(client, new { cmd = "connect", port = req.Port, baud = req.Baud }));
app.MapPost("/api/telemetry/disconnect", (DroneClient client) => Relay(client, new { cmd = "disconnect" }));
app.MapPost("/api/demo/enable", (DemoEnableRequest req, DroneClient client) => Relay(client, new { cmd = "enable_demo", enabled = req.Enabled, confirmed = req.Confirmed }));
app.MapPost("/api/demo/{action}", (string action, DemoRequest req, DroneClient client) =>
{
    if (!new[] { "arm", "disarm", "takeoff", "hold", "goto", "yaw", "land", "abort", "bench_arm", "offboard_bench", "set_mode" }.Contains(action))
        return Results.BadRequest(new { status = "error", message = "Unknown demo action" });
    return Relay(client, new { cmd = action, mode = req.Mode, altitude = req.Altitude, lat = req.Lat, lon = req.Lon, heading = req.Heading, propsOffConfirmed = req.PropsOffConfirmed });
});
app.MapPost("/api/abort", (DroneClient client) => Relay(client, new { cmd = "abort" }));
app.MapPost("/api/fly", () => Results.BadRequest(new { status = "error", message = "Automatic planner execution is disabled. Review the planned path and use the gated demo controls." }));
app.MapGet("/api/health", () => Results.Ok(new { status = "ok", timestamp = DateTime.UtcNow }));
app.Run();
