namespace DroneNavigation.Tests;

using Xunit;
using DroneNavigation.Geometry;
using DroneNavigation.Obstacles;
using DroneNavigation.PathPlanning;

// Tests for the path smoother. The smoother takes a raw A* output and removes
// redundant intermediate waypoints while preserving the path's safety.
public class PathSmootherTests
{
    private static Grid MakeEmptyGrid()
    {
        QuadTree.QuadTree obstacles = new QuadTree.QuadTree(new Rectangle(0, 0, 100, 100));
        return new Grid(
            obstacles,
            new Rectangle(0, 0, 100, 100),
            cellSize: 5.0,
            droneAltitudeMeters: 20.0);
    }

    [Fact]
    public void EmptyGrid_SmoothsToTwoWaypoints()
    {
        // With no obstacles, A* will produce a long zigzag-ish path. Smoothing
        // should collapse it to just the start and goal — a straight line.
        Grid grid = MakeEmptyGrid();
        List<Point> raw = AStar.FindPath(grid, new Point(10, 10), new Point(90, 90));
        Assert.True(raw.Count > 2, "Test setup: raw path should have many waypoints");

        List<Point> smoothed = PathSmoother.Smooth(grid, raw);

        Assert.Equal(2, smoothed.Count);
    }

    [Fact]
    public void SmoothedPath_StaysCollisionFree()
    {
        // Smoothing must never produce a path segment that crosses an obstacle.
        // Set up a maze, smooth the path, then walk each segment of the smoothed
        // path and verify no sample lands on a blocked cell.
        QuadTree.QuadTree obstacles = new QuadTree.QuadTree(new Rectangle(0, 0, 100, 100));
        obstacles.Insert(new Obstacle(40, 50, radius: 8, height: 30));
        obstacles.Insert(new Obstacle(60, 50, radius: 8, height: 30));
        Grid grid = new Grid(
            obstacles,
            new Rectangle(0, 0, 100, 100),
            cellSize: 4.0,
            droneAltitudeMeters: 20.0);

        List<Point> raw = AStar.FindPath(grid, new Point(10, 50), new Point(90, 50));
        List<Point> smoothed = PathSmoother.Smooth(grid, raw);

        Assert.True(smoothed.Count >= 2);

        // Walk each segment and sample. Use cellSize/4 step to be extra strict.
        for (int i = 0; i < smoothed.Count - 1; i++)
        {
            Point a = smoothed[i];
            Point b = smoothed[i + 1];
            double dx = b.X - a.X;
            double dy = b.Y - a.Y;
            double dist = Math.Sqrt(dx * dx + dy * dy);
            int steps = Math.Max(1, (int)Math.Ceiling(dist / (grid.CellSize / 4.0)));

            for (int s = 0; s <= steps; s++)
            {
                double t = (double)s / steps;
                Point sample = new Point(a.X + dx * t, a.Y + dy * t);
                Assert.True(grid.TryWorldToCell(sample, out int col, out int row));
                Assert.True(grid.IsFree(col, row),
                    $"Smoothed segment {i} -> {i+1} passes through blocked cell ({col}, {row})");
            }
        }
    }

    [Fact]
    public void SmoothedPath_HasFewerOrEqualWaypoints()
    {
        // Smoothing should never INCREASE the number of waypoints. Common sense,
        // but worth asserting — guards against off-by-one bugs in the loop.
        QuadTree.QuadTree obstacles = new QuadTree.QuadTree(new Rectangle(0, 0, 100, 100));
        obstacles.Insert(new Obstacle(50, 50, radius: 10, height: 30));
        Grid grid = new Grid(
            obstacles,
            new Rectangle(0, 0, 100, 100),
            cellSize: 4.0,
            droneAltitudeMeters: 20.0);

        List<Point> raw = AStar.FindPath(grid, new Point(10, 10), new Point(90, 90));
        List<Point> smoothed = PathSmoother.Smooth(grid, raw);

        Assert.True(smoothed.Count <= raw.Count,
            $"Smoothing produced more waypoints ({smoothed.Count}) than raw ({raw.Count})");
    }
}