namespace DroneNavigation.Tests;

using Xunit;
using DroneNavigation.Geometry;
using DroneNavigation.Obstacles;
using DroneNavigation.PathPlanning;

// Tests for the A* path planner. Each test sets up a small world with obstacles,
// runs FindPath, and verifies the returned path obeys the rules: starts at start,
// ends at goal, never crosses a blocked cell, and is empty when no path exists.
public class AStarTests
{
    // A standard test world: 100m x 100m, cells of 5m (so 20x20 grid), drone at 20m altitude.
    // Returns an empty quadtree the caller can populate with obstacles.
    private static QuadTree.QuadTree MakeEmptyWorld()
    {
        return new QuadTree.QuadTree(new Rectangle(0, 0, 100, 100));
    }

    private static Grid MakeGrid(QuadTree.QuadTree obstacles)
    {
        return new Grid(
            obstacles,
            new Rectangle(0, 0, 100, 100),
            cellSize: 5.0,
            droneAltitudeMeters: 20.0);
    }

    [Fact]
    public void EmptyWorld_ReturnsDirectPath()
    {
        Grid grid = MakeGrid(MakeEmptyWorld());

        List<Point> path = AStar.FindPath(grid, new Point(10, 10), new Point(90, 90));

        Assert.NotEmpty(path);

        // First point should be in the start cell, last point in the goal cell.
        Assert.True(grid.TryWorldToCell(path[0], out int startCol, out int startRow));
        Assert.True(grid.TryWorldToCell(path[^1], out int goalCol, out int goalRow));
        Assert.True(grid.TryWorldToCell(new Point(10, 10), out int expectedStartCol, out int expectedStartRow));
        Assert.True(grid.TryWorldToCell(new Point(90, 90), out int expectedGoalCol, out int expectedGoalRow));
        Assert.Equal(expectedStartCol, startCol);
        Assert.Equal(expectedStartRow, startRow);
        Assert.Equal(expectedGoalCol, goalCol);
        Assert.Equal(expectedGoalRow, goalRow);
    }

    [Fact]
    public void StartEqualsGoal_ReturnsSingleCellPath()
    {
        Grid grid = MakeGrid(MakeEmptyWorld());

        List<Point> path = AStar.FindPath(grid, new Point(50, 50), new Point(50, 50));

        Assert.Single(path);
    }

    [Fact]
    public void ObstacleBetweenStartAndGoal_PathRoutesAroundIt()
    {
        // Put a tall obstacle right in the middle. Drone is below its height, so it blocks.
        QuadTree.QuadTree obstacles = MakeEmptyWorld();
        obstacles.Insert(new Obstacle(50, 50, radius: 15, height: 30));
        Grid grid = MakeGrid(obstacles);

        List<Point> path = AStar.FindPath(grid, new Point(10, 50), new Point(90, 50));

        Assert.NotEmpty(path);

        // Every cell along the path must be free — i.e., the planner actually went around.
        foreach (Point p in path)
        {
            Assert.True(grid.TryWorldToCell(p, out int col, out int row));
            Assert.True(grid.IsFree(col, row),
                $"Path crosses a blocked cell at ({col}, {row}) — A* produced an invalid path.");
        }
    }

    [Fact]
    public void ObstacleBelowDroneAltitude_DoesNotBlock()
    {
        // A 5m-tall obstacle and the drone at 20m. With a 3m buffer, threshold is 20m vs 8m —
        // drone clears it. The grid should not mark these cells as blocked.
        QuadTree.QuadTree obstacles = MakeEmptyWorld();
        obstacles.Insert(new Obstacle(50, 50, radius: 15, height: 5));
        Grid grid = MakeGrid(obstacles);

        // The cell containing (50, 50) should be free, because the obstacle is too short.
        Assert.True(grid.TryWorldToCell(new Point(50, 50), out int col, out int row));
        Assert.True(grid.IsFree(col, row));
    }

    [Fact]
    public void CompletelyBlockedWorld_ReturnsEmptyPath()
    {
        // A huge obstacle covering the whole map blocks all routes.
        QuadTree.QuadTree obstacles = MakeEmptyWorld();
        for (double x = 10; x < 100; x += 5)
            for (double y = 10; y < 100; y += 5)
                obstacles.Insert(new Obstacle(x, y, radius: 4, height: 30));
        Grid grid = MakeGrid(obstacles);

        List<Point> path = AStar.FindPath(grid, new Point(2, 2), new Point(98, 98));

        Assert.Empty(path);
    }

    [Fact]
    public void StartInsideObstacle_ReturnsEmptyPath()
    {
        // Start point is in a blocked cell — no valid path can exist.
        QuadTree.QuadTree obstacles = MakeEmptyWorld();
        obstacles.Insert(new Obstacle(50, 50, radius: 20, height: 30));
        Grid grid = MakeGrid(obstacles);

        List<Point> path = AStar.FindPath(grid, new Point(50, 50), new Point(90, 90));

        Assert.Empty(path);
    }

    [Fact]
    public void GoalOutsideGrid_ReturnsEmptyPath()
    {
        Grid grid = MakeGrid(MakeEmptyWorld());

        List<Point> path = AStar.FindPath(grid, new Point(10, 10), new Point(500, 500));

        Assert.Empty(path);
    }

    [Fact]
    public void PathIsContinuous_EachStepIsToANeighborCell()
    {
        // Every consecutive pair of cells in the path must differ by at most one in
        // both col and row. This catches algorithm bugs that "teleport" the drone.
        QuadTree.QuadTree obstacles = MakeEmptyWorld();
        obstacles.Insert(new Obstacle(40, 50, 10, 30));
        obstacles.Insert(new Obstacle(60, 50, 10, 30));
        Grid grid = MakeGrid(obstacles);

        List<Point> path = AStar.FindPath(grid, new Point(10, 10), new Point(90, 90));

        Assert.NotEmpty(path);
        for (int i = 1; i < path.Count; i++)
        {
            Assert.True(grid.TryWorldToCell(path[i - 1], out int prevCol, out int prevRow));
            Assert.True(grid.TryWorldToCell(path[i], out int curCol, out int curRow));
            int dCol = Math.Abs(curCol - prevCol);
            int dRow = Math.Abs(curRow - prevRow);
            Assert.True(dCol <= 1 && dRow <= 1,
                $"Path jumps from ({prevCol}, {prevRow}) to ({curCol}, {curRow}) — not adjacent.");
            Assert.True(dCol + dRow > 0, "Path has a duplicate cell.");
        }
    }
}