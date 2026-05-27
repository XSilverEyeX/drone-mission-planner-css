using DroneNavigation.Drone;
using DroneNavigation.Geometry;
using DroneNavigation.Obstacles;
using DroneNavigation.PathPlanning;
using DroneNavigation.QuadTree;

// End-to-end demo: build a world, plan a path, fly it.
//
// Prerequisites:
//   1. SITL is running (sim_vehicle.py -v ArduCopter --out 127.0.0.1:14550 --out 127.0.0.1:14551)
//   2. Python bridge is running (python3 drone_bridge.py)
//   3. QGroundControl is open (optional but recommended for visual)

const double WorldSize = 100.0;     // 100m x 100m local-frame world
const double CellSize = 4.0;        // 4m grid cells
const double CruiseAltitude = 20.0; // drone flies at 20m

Console.WriteLine("=== Phase D end-to-end demo ===\n");

// Build the same 'wall' scene we visualized in Phase C.
QuadTree obstacles = new QuadTree(new Rectangle(0, 0, WorldSize, WorldSize));
obstacles.Insert(new Obstacle(30, 50, radius: 8, height: 30));
obstacles.Insert(new Obstacle(50, 30, radius: 10, height: 30));
obstacles.Insert(new Obstacle(50, 70, radius: 10, height: 30));
obstacles.Insert(new Obstacle(70, 50, radius: 8, height: 30));
obstacles.Insert(new Obstacle(20, 20, radius: 5, height: 5)); // short, will fly over

Grid grid = new Grid(obstacles,
    new Rectangle(0, 0, WorldSize, WorldSize),
    cellSize: CellSize,
    droneAltitudeMeters: CruiseAltitude);

// Plan and smooth. Start at (10, 50), goal at (90, 50). Same as wall scene.
Point start = new Point(10, 50);
Point goal = new Point(90, 50);

List<Point> rawPath = AStar.FindPath(grid, start, goal);
List<Point> path = PathSmoother.Smooth(grid, rawPath);

Console.WriteLine($"Planned path: {rawPath.Count} raw -> {path.Count} smoothed waypoints");
foreach (Point p in path)
    Console.WriteLine($"  ({p.X:F1}, {p.Y:F1})");
Console.WriteLine();

// Connect to the bridge.
Console.WriteLine("Connecting to drone bridge...");
using DroneClient client = new DroneClient();
client.Connect();
Console.WriteLine("Connected.\n");

// Helper to print results compactly and bail out on errors.
void Step(string label, Func<DroneResponse> action)
{
    Console.Write($"{label}... ");
    DroneResponse r = action();
    Console.WriteLine(r);
    if (!r.IsOk)
    {
        Console.WriteLine("Aborting due to error.");
        Environment.Exit(1);
    }
}

Step("Set home",             () => client.SetHome());
Step("Arm",                  () => client.Arm());
Step($"Takeoff to {CruiseAltitude}m", () => client.Takeoff(CruiseAltitude));

// Fly each smoothed waypoint. The start point is where we already are, so skip it.
for (int i = 1; i < path.Count; i++)
{
    Point p = path[i];
    // Convert from "world" frame (0..100, 0..100 with start at 10,50) into the
    // local frame the bridge expects (meters east/north of home). We use the
    // start point as the origin of the local frame.
    double xOffset = p.X - start.X;
    double yOffset = p.Y - start.Y;
    Step($"Fly to waypoint {i}/{path.Count - 1} (local x={xOffset:F1} y={yOffset:F1})",
        () => client.GotoLocal(xOffset, yOffset, CruiseAltitude));
}

Step("Land", () => client.Land());

Console.WriteLine("\nMission complete.");