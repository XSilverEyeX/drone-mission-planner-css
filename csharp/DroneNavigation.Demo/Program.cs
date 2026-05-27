using DroneNavigation.Geometry;
using DroneNavigation.Obstacles;
using DroneNavigation.PathPlanning;
using DroneNavigation.QuadTree;

// Demo driver. Run with one of these scene names as a command-line argument:
//   wall      — four-obstacle wall the drone must wiggle past
//   corridor  — narrow gap between two large obstacles
//   maze      — random scattered obstacles forming a small maze
// Default if no argument: wall.

string scene = args.Length > 0 ? args[0] : "wall";
Console.WriteLine($"=== A* Path Planning Demo: scene '{scene}' ===\n");

QuadTree obstacles = new QuadTree(new Rectangle(0, 0, 100, 100));
Point start = new Point(10, 50);
Point goal = new Point(90, 50);

switch (scene)
{
    case "wall":
        // Four obstacles forming a partial wall the drone must route around.
        obstacles.Insert(new Obstacle(30, 50, radius: 8, height: 30));
        obstacles.Insert(new Obstacle(50, 30, radius: 10, height: 30));
        obstacles.Insert(new Obstacle(50, 70, radius: 10, height: 30));
        obstacles.Insert(new Obstacle(70, 50, radius: 8, height: 30));
        // Plus a short obstacle the drone flies over.
        obstacles.Insert(new Obstacle(20, 20, radius: 5, height: 5));
        break;

    case "corridor":
        // Two large obstacles leaving a narrow gap in the middle.
        obstacles.Insert(new Obstacle(50, 25, radius: 18, height: 30));
        obstacles.Insert(new Obstacle(50, 75, radius: 18, height: 30));
        break;

    case "maze":
        // Scattered obstacles forming a more complex avoidance scene.
        obstacles.Insert(new Obstacle(25, 30, radius: 7, height: 30));
        obstacles.Insert(new Obstacle(25, 70, radius: 7, height: 30));
        obstacles.Insert(new Obstacle(50, 50, radius: 8, height: 30));
        obstacles.Insert(new Obstacle(75, 30, radius: 7, height: 30));
        obstacles.Insert(new Obstacle(75, 70, radius: 7, height: 30));
        obstacles.Insert(new Obstacle(40, 15, radius: 5, height: 30));
        obstacles.Insert(new Obstacle(60, 85, radius: 5, height: 30));
        break;

    default:
        Console.WriteLine($"Unknown scene '{scene}'. Use 'wall', 'corridor', or 'maze'.");
        return;
}

Grid grid = new Grid(
    obstacles,
    new Rectangle(0, 0, 100, 100),
    cellSize: 4.0,
    droneAltitudeMeters: 20.0);

List<Point> rawPath = AStar.FindPath(grid, start, goal);
List<Point> smoothedPath = PathSmoother.Smooth(grid, rawPath);

Console.WriteLine($"Grid: {grid.Cols}x{grid.Rows} cells, {grid.CellSize}m each");
Console.WriteLine($"Start: ({start.X}, {start.Y})");
Console.WriteLine($"Goal:  ({goal.X}, {goal.Y})");
Console.WriteLine($"Raw path:      {rawPath.Count} waypoints");
Console.WriteLine($"Smoothed path: {smoothedPath.Count} waypoints");
Console.WriteLine($"Reduction:     {100.0 * (rawPath.Count - smoothedPath.Count) / Math.Max(1, rawPath.Count):F1}%\n");

Console.WriteLine("--- Raw A* path ---");
Console.WriteLine(GridVisualizer.ToAscii(grid, rawPath, start, goal));

Console.WriteLine("--- Smoothed path ---");
Console.WriteLine(GridVisualizer.ToAscii(grid, smoothedPath, start, goal));

// Write two SVGs — one for raw, one for smoothed. Both go to project root.
string rawSvg = GridVisualizer.ToSvg(grid, rawPath, start, goal, pixelsPerCell: 20);
string smoothedSvg = GridVisualizer.ToSvg(grid, smoothedPath, start, goal, pixelsPerCell: 20);

string baseDir = Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..");
string rawPathFile = Path.Combine(baseDir, $"path_{scene}_raw.svg");
string smoothedPathFile = Path.Combine(baseDir, $"path_{scene}_smoothed.svg");
File.WriteAllText(rawPathFile, rawSvg);
File.WriteAllText(smoothedPathFile, smoothedSvg);

Console.WriteLine($"Raw SVG:      {Path.GetFullPath(rawPathFile)}");
Console.WriteLine($"Smoothed SVG: {Path.GetFullPath(smoothedPathFile)}");