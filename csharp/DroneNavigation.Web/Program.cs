using DroneNavigation.Drone;
using DroneNavigation.Geometry;
using DroneNavigation.Obstacles;
using DroneNavigation.PathPlanning;
using DroneNavigation.QuadTree;
using DroneNavigation.Web.Models;

var builder = WebApplication.CreateBuilder(args);

// Allow the browser frontend to call us from a different origin during dev.
// In production you'd lock this down to specific origins.
builder.Services.AddCors(options =>
{
    options.AddDefaultPolicy(policy =>
        policy.AllowAnyOrigin().AllowAnyMethod().AllowAnyHeader());
});

// Register a single DroneClient that the API will share across requests.
// Long-lived TCP connection to the Python bridge.
builder.Services.AddSingleton<DroneClient>();

var app = builder.Build();
app.UseCors();
app.UseDefaultFiles();
app.UseStaticFiles();

// World configuration — same conventions as the rest of the project.
const double WorldSize = 100.0;
const double CellSize = 4.0;
const double SafetyBufferMeters = 3.0;

// ----- Plan endpoint: given obstacles + start + goal + altitude, return a path -----

app.MapPost("/api/plan", (PlanRequest req) =>
{
    QuadTree tree = new QuadTree(new Rectangle(0, 0, WorldSize, WorldSize));
    foreach (ObstacleDto o in req.Obstacles)
        tree.Insert(new Obstacle(o.X, o.Y, o.Radius, o.Height));

    Grid grid = new Grid(tree,
        new Rectangle(0, 0, WorldSize, WorldSize),
        cellSize: CellSize,
        droneAltitudeMeters: req.AltitudeMeters,
        safetyBufferMeters: SafetyBufferMeters);

    Point start = new Point(req.Start.X, req.Start.Y);
    Point goal = new Point(req.Goal.X, req.Goal.Y);

    List<Point> rawPath = AStar.FindPath(grid, start, goal);
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

    List<Point> smoothed = PathSmoother.Smooth(grid, rawPath);

    return Results.Ok(new PlanResponse
    {
        Success = true,
        Message = $"Planned {smoothed.Count} waypoints (raw {rawPath.Count})",
        Waypoints = smoothed.Select(p => new PointDto { X = p.X, Y = p.Y }).ToList(),
        RawWaypointCount = rawPath.Count
    });
});

// ----- Fly endpoint: actually execute a planned path against the bridge -----

app.MapPost("/api/fly", async (FlyRequest req, DroneClient client) =>
{
    if (req.Waypoints == null || req.Waypoints.Count < 2)
        return Results.BadRequest(new { error = "Need at least 2 waypoints (start + goal)" });

    try
    {
        if (!client.IsConnected) client.Connect();

        // Run the flight on a background task so the HTTP request can return immediately.
        // The mission can take ~1 minute and we don't want to block.
        _ = Task.Run(() =>
        {
            try
            {
                client.SetHome();
                client.Arm();
                client.Takeoff(req.AltitudeMeters);

                PointDto startPoint = req.Waypoints[0];
                for (int i = 1; i < req.Waypoints.Count; i++)
                {
                    PointDto p = req.Waypoints[i];
                    double xOffset = p.X - startPoint.X;
                    double yOffset = p.Y - startPoint.Y;
                    client.GotoLocal(xOffset, yOffset, req.AltitudeMeters);
                }

                client.Land();
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[mission] Error: {ex.Message}");
            }
        });

        return Results.Ok(new { status = "started", waypoints = req.Waypoints.Count });
    }
    catch (Exception ex)
    {
        return Results.Problem($"Could not start mission: {ex.Message}");
    }
});

// ----- Abort endpoint: switch to LAND immediately -----

app.MapPost("/api/abort", (DroneClient client) =>
{
    try
    {
        if (!client.IsConnected) client.Connect();
        DroneResponse r = client.Land();
        return Results.Ok(new { status = r.Status, message = r.Message });
    }
    catch (Exception ex)
    {
        return Results.Problem($"Abort failed: {ex.Message}");
    }
});

// ----- Status endpoint: current drone position (used by polling fallback or for testing) -----

// Configuration: the home GPS the bridge uses for local-frame conversions.
// Must match the Canberra airfield where SITL spawns by default.
// Phase E.6 will replace this with a dynamic value.
const double HomeLat = -35.363262;
const double HomeLon = 149.165237;
const double EarthRadiusM = 6_378_137.0;

app.MapGet("/api/status", (DroneClient client) =>
{
    try
    {
        if (!client.IsConnected) client.Connect();
        DroneResponse r = client.GetPosition();

        if (!r.IsOk || !r.Data.HasValue)
            return Results.Ok(new { connected = false, message = r.Message });

        // The bridge returns absolute GPS (lat/lon). For the map, we also
        // want to know the position relative to home in world meters so the
        // frontend can draw the drone icon in the same coordinate space as
        // obstacles.
        var data = r.Data.Value;
        double lat = data.GetProperty("lat").GetDouble();
        double lon = data.GetProperty("lon").GetDouble();
        double altitude = data.GetProperty("altitude").GetDouble();

        // Inverse of the bridge's equirectangular conversion.
        double yMeters = (lat - HomeLat) * Math.PI / 180.0 * EarthRadiusM;
        double xMeters = (lon - HomeLon) * Math.PI / 180.0 * EarthRadiusM
                       * Math.Cos(HomeLat * Math.PI / 180.0);

        return Results.Ok(new
        {
            connected = true,
            lat,
            lon,
            altitude,
            // Local frame: meters east/north of home.
            xLocal = xMeters,
            yLocal = yMeters
        });
    }
    catch (Exception ex)
    {
        return Results.Ok(new { connected = false, message = ex.Message });
    }
});

// ----- Health check, so we can verify the server is alive without touching the drone -----

app.MapGet("/api/health", () => Results.Ok(new { status = "ok", timestamp = DateTime.UtcNow }));

app.Run();