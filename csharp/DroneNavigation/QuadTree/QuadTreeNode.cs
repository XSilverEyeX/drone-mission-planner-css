namespace DroneNavigation.QuadTree;

using DroneNavigation.Geometry;
using DroneNavigation.Obstacles;

// Represents a node in the quadtree data structure.
// Each QuadTreeNode represents a rectangular region of space and can contain obstacles.
// If the node has children (NW, NE, SW, SE), then the obstacles are stored in the
// appropriate child node based on their position. If the node is a leaf (no children),
// then it stores obstacles directly. The quadtree subdivides nodes into four children
// when they exceed a certain capacity of obstacles, allowing for efficient spatial queries.
internal class QuadTreeNode
{
    public Rectangle Bounds { get; }
    public int Depth { get; }
    public List<Obstacle> Obstacles { get; }

    // The child quadrants of this node. They are nullable because a node might not have
    // been subdivided yet. When we insert an obstacle and the node exceeds its capacity,
    // we create these child nodes and redistribute the obstacles among them.
    public QuadTreeNode? NW { get; private set; }
    public QuadTreeNode? NE { get; private set; }
    public QuadTreeNode? SW { get; private set; }
    public QuadTreeNode? SE { get; private set; }

    // A node is a leaf if it has not been subdivided yet. We only check NW because
    // subdivision creates all four children at once — they are either all null or all set.
    public bool IsLeaf => NW == null;

    // Private fields hold the configuration values passed in from the parent QuadTree.
    // The underscore prefix is the C# convention for private fields.
    private readonly int _capacity;
    private readonly int _maxDepth;

    public QuadTreeNode(Rectangle bounds, int depth, int capacity, int maxDepth)
    {
        Bounds = bounds;
        Depth = depth;
        Obstacles = new List<Obstacle>();
        _capacity = capacity;
        _maxDepth = maxDepth;
    }

    // Inserts an obstacle into the tree. There are four cases:
    //   Case 1: Leaf has room               → just add it
    //   Case 2: Leaf is full, can subdivide → subdivide, redistribute, then insert
    //   Case 3: Internal node               → delegate to the right child
    //   Case 4: Leaf at max depth           → just add it (safety net)
    public void Insert(Obstacle obstacle)
    {
        // Case 3: This is an internal node — delegate to the right child.
        if (!IsLeaf)
        {
            QuadTreeNode? child = GetQuadrantFor(obstacle.Center);
            child?.Insert(obstacle);
            return;
        }

        // Cases 1 and 4: Leaf has room OR we have hit max depth — just store it.
        if (Obstacles.Count < _capacity || Depth >= _maxDepth)
        {
            Obstacles.Add(obstacle);
            return;
        }

        // Case 2: Leaf is full and we can still subdivide.
        Subdivide();

        // Redistribute existing obstacles to children.
        foreach (Obstacle existing in Obstacles)
        {
            QuadTreeNode? child = GetQuadrantFor(existing.Center);
            child?.Insert(existing);
        }
        Obstacles.Clear();

        // Insert the new obstacle into the appropriate child.
        QuadTreeNode? newChild = GetQuadrantFor(obstacle.Center);
        newChild?.Insert(obstacle);
    }

    // Creates the four child nodes by splitting this node's bounds in half along both axes.
    // Coordinate convention: Y increases UPWARD (math convention, not screen convention).
    // So "North" (NW, NE) is the upper half (higher Y), "South" (SW, SE) is the lower half.
    private void Subdivide()
    {
        double halfWidth = Bounds.Width / 2;
        double halfHeight = Bounds.Height / 2;
        double midX = Bounds.X + halfWidth;
        double midY = Bounds.Y + halfHeight;
        int childDepth = Depth + 1;

        NW = new QuadTreeNode(
            new Rectangle(Bounds.X, midY, halfWidth, halfHeight),
            childDepth, _capacity, _maxDepth);

        NE = new QuadTreeNode(
            new Rectangle(midX, midY, halfWidth, halfHeight),
            childDepth, _capacity, _maxDepth);

        SW = new QuadTreeNode(
            new Rectangle(Bounds.X, Bounds.Y, halfWidth, halfHeight),
            childDepth, _capacity, _maxDepth);

        SE = new QuadTreeNode(
            new Rectangle(midX, Bounds.Y, halfWidth, halfHeight),
            childDepth, _capacity, _maxDepth);
    }

    // Returns the child quadrant that contains the given point, or null if no child does.
    // The short-circuit && ensures we never call .Bounds on a null reference.
    private QuadTreeNode? GetQuadrantFor(Point point)
    {
        if (NW != null && NW.Bounds.Contains(point)) return NW;
        if (NE != null && NE.Bounds.Contains(point)) return NE;
        if (SW != null && SW.Bounds.Contains(point)) return SW;
        if (SE != null && SE.Bounds.Contains(point)) return SE;
        return null;
    }

    // Finds all obstacles whose centers fall within the given query rectangle.
    // Results are accumulated into the provided list rather than returned, so we
    // avoid allocating a new list at every recursion level.
    //
    // The magic of the quadtree is the prune step on the first line: if our
    // bounds do not intersect the query range, we skip this entire subtree.
    public void Query(Rectangle range, List<Obstacle> results)
    {
        if (!Bounds.Intersects(range))
            return;

        if (IsLeaf)
        {
            foreach (Obstacle o in Obstacles)
            {
                if (range.Contains(o.Center))
                    results.Add(o);
            }
            return;
        }

        NW?.Query(range, results);
        NE?.Query(range, results);
        SW?.Query(range, results);
        SE?.Query(range, results);
    }
}