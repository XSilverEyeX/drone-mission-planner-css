namespace DroneNavigation.Tests;

using Xunit;
using DroneNavigation.Geometry;
using DroneNavigation.Obstacles;
using DroneNavigation.QuadTree;

public class QuadTreeTests
{
    // Helper to create a standard tree for tests.
    // Bounds: 0-1000 in both axes. Default capacity (4) and max depth (8).
    private QuadTree MakeTree() => new QuadTree(new Rectangle(0, 0, 1000, 1000));

    [Fact]
    public void NewTree_HasNoObstacles()
    {
        QuadTree tree = MakeTree();

        List<Obstacle> results = tree.Query(new Rectangle(0, 0, 1000, 1000));

        Assert.Empty(results);
    }

    [Fact]
    public void Insert_StoresObstacle_AndQueryReturnsIt()
    {
        QuadTree tree = MakeTree();
        Obstacle obstacle = new Obstacle(500, 500, radius: 10, height: 20);

        tree.Insert(obstacle);
        List<Obstacle> results = tree.Query(new Rectangle(0, 0, 1000, 1000));

        Assert.Single(results);
    }

    [Fact]
    public void Query_ExcludesObstacles_OutsideRange()
    {
        QuadTree tree = MakeTree();
        tree.Insert(new Obstacle(100, 100, radius: 5, height: 10));
        tree.Insert(new Obstacle(900, 900, radius: 5, height: 10));

        // Query only the lower-left quadrant.
        List<Obstacle> results = tree.Query(new Rectangle(0, 0, 500, 500));

        Assert.Single(results);
        Assert.Equal(100, results[0].X);
    }

    [Fact]
    public void Query_ReturnsMultipleObstacles_InRange()
    {
        QuadTree tree = MakeTree();
        tree.Insert(new Obstacle(100, 100, 5, 10));
        tree.Insert(new Obstacle(200, 200, 5, 10));
        tree.Insert(new Obstacle(300, 300, 5, 10));

        List<Obstacle> results = tree.Query(new Rectangle(0, 0, 500, 500));

        Assert.Equal(3, results.Count);
    }

    [Fact]
    public void Insert_BeyondCapacity_TriggersSubdivision()
    {
        // Capacity 2, max depth 3. Insert 5 obstacles — should subdivide.
        QuadTree tree = new QuadTree(new Rectangle(0, 0, 1000, 1000),
                                     capacity: 2, maxDepth: 3);

        // All in the same general area but distinguishable points.
        tree.Insert(new Obstacle(100, 100, 5, 10));
        tree.Insert(new Obstacle(200, 200, 5, 10));
        tree.Insert(new Obstacle(300, 300, 5, 10));
        tree.Insert(new Obstacle(400, 400, 5, 10));
        tree.Insert(new Obstacle(450, 450, 5, 10));

        // We can't directly inspect the internal nodes (they are internal-access),
        // but we can verify behavior: all 5 obstacles should still be retrievable.
        List<Obstacle> results = tree.Query(new Rectangle(0, 0, 1000, 1000));
        Assert.Equal(5, results.Count);
    }

    [Fact]
    public void Insert_OutsideBounds_ThrowsArgumentException()
    {
        QuadTree tree = MakeTree();
        Obstacle outOfBounds = new Obstacle(2000, 2000, radius: 10, height: 20);

        Assert.Throws<ArgumentException>(() => tree.Insert(outOfBounds));
    }

    [Fact]
    public void Constructor_InvalidCapacity_ThrowsArgumentException()
    {
        Assert.Throws<ArgumentException>(
            () => new QuadTree(new Rectangle(0, 0, 1000, 1000), capacity: 0));
    }
}


