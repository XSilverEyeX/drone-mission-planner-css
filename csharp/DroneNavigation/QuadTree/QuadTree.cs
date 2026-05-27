namespace DroneNavigation.QuadTree;

using DroneNavigation.Geometry;
using DroneNavigation.Obstacles;

// Public-facing quadtree class. Wraps the recursive QuadTreeNode root and provides
// a simple API: Insert(obstacle) and Query(range). Configuration (capacity, max depth)
// is set at construction time.
//
// Design pattern: Facade. The complexity of recursion, subdivision, and node management
// lives inside QuadTreeNode (which is internal — invisible to external code). Users of
// this library only interact with QuadTree, keeping their code clean.
public class QuadTree
{
    public Rectangle Bounds { get; }
    public int Capacity { get; }
    public int MaxDepth { get; }

    private readonly QuadTreeNode _root;

    public QuadTree(Rectangle bounds, int capacity = 4, int maxDepth = 8)
    {
        if (capacity < 1)
            throw new ArgumentException("Capacity must be at least 1", nameof(capacity));
        if (maxDepth < 1)
            throw new ArgumentException("MaxDepth must be at least 1", nameof(maxDepth));

        Bounds = bounds;
        Capacity = capacity;
        MaxDepth = maxDepth;
        _root = new QuadTreeNode(bounds, depth: 0, capacity, maxDepth);
    }

    // Inserts an obstacle into the tree. Throws if the obstacle's center is outside
    // the tree's bounds — obstacles outside the world don't belong in the data structure.
    public void Insert(Obstacle obstacle)
    {
        if (!Bounds.Contains(obstacle.Center))
            throw new ArgumentException(
                $"Obstacle center {obstacle.Center} is outside tree bounds {Bounds}",
                nameof(obstacle));

        _root.Insert(obstacle);
    }

    // Returns all obstacles whose centers fall within the given rectangle.
    // Allocates a fresh list each call — callers don't need to manage one themselves.
    public List<Obstacle> Query(Rectangle range)
    {
        List<Obstacle> results = new List<Obstacle>();
        _root.Query(range, results);
        return results;
    }
}