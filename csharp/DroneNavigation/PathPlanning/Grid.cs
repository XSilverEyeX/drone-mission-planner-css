namespace DroneNavigation.PathPlanning;

using DroneNavigation.Geometry;
using DroneNavigation.Obstacles;
using DroneNavigation.QuadTree;

// A 2D grid overlaid on the world. Each cell is either FREE (drone can fly through)
// or BLOCKED (obstacle present). The grid is the search space A* will explore.
//
// Why a grid: A* needs a finite set of nodes to search. The world is continuous, so we
// discretize it into uniform cells. Cell size is a trade-off — smaller cells produce
// finer paths but increase memory and search time quadratically.
//
// Coordinate convention (consistent with the quadtree):
//   - World X, Y are doubles in continuous space.
//   - Grid Col, Row are zero-based integer indices into the cell array.
//   - Y increases UPWARD (math convention). Row 0 is the bottom row of the grid.
//
// Blocked-cell computation is EAGER: at construction time, we query the quadtree
// once per cell and store the result. This makes A* fast at the cost of a one-time
// O(rows * cols) setup. For our scale (thousands of cells) that's negligible.
public class Grid
{
    public Rectangle Bounds { get; }
    public double CellSize { get; }
    public int Cols { get; }
    public int Rows { get; }
    public double DroneAltitudeMeters { get; }
    public double SafetyBufferMeters { get; }

    // _blocked[col, row] is true if that cell intersects any obstacle that
    // blocks the drone at the configured altitude.
    private readonly bool[,] _blocked;

    public Grid(
        global::DroneNavigation.QuadTree.QuadTree obstacles,
        Rectangle bounds,
        double cellSize,
        double droneAltitudeMeters,
        double safetyBufferMeters = 3.0)
    {
        if (cellSize <= 0)
            throw new ArgumentException("Cell size must be positive", nameof(cellSize));

        Bounds = bounds;
        CellSize = cellSize;
        DroneAltitudeMeters = droneAltitudeMeters;
        SafetyBufferMeters = safetyBufferMeters;

        Cols = (int)Math.Ceiling(bounds.Width / cellSize);
        Rows = (int)Math.Ceiling(bounds.Height / cellSize);
        _blocked = new bool[Cols, Rows];

        // Mark blocked cells eagerly. For each cell, ask the quadtree for any obstacles
        // that intersect this cell's bounds. If we find one that blocks at our altitude,
        // the cell is blocked.
        for (int col = 0; col < Cols; col++)
        {
            for (int row = 0; row < Rows; row++)
            {
                Rectangle cellRect = GetCellRect(col, row);
                List<Obstacle> nearby = obstacles.Query(cellRect);
                foreach (Obstacle o in nearby)
                {
                    if (o.BlocksAtAltitude(droneAltitudeMeters, safetyBufferMeters))
                    {
                        _blocked[col, row] = true;
                        break;
                    }
                }
            }
        }
    }

    // Returns the world-space rectangle covered by a given cell.
    public Rectangle GetCellRect(int col, int row)
    {
        double x = Bounds.X + col * CellSize;
        double y = Bounds.Y + row * CellSize;
        return new Rectangle(x, y, CellSize, CellSize);
    }

    // Returns the world-space center point of a cell — used by A* as the "position" of
    // a node and as the waypoint we ultimately send to the drone.
    public Point GetCellCenter(int col, int row)
    {
        Rectangle r = GetCellRect(col, row);
        return new Point(r.CenterX, r.CenterY);
    }

    // Converts a world-space point into the grid cell that contains it.
    // Returns false if the point is outside the grid.
    public bool TryWorldToCell(Point world, out int col, out int row)
    {
        col = (int)Math.Floor((world.X - Bounds.X) / CellSize);
        row = (int)Math.Floor((world.Y - Bounds.Y) / CellSize);
        return col >= 0 && col < Cols && row >= 0 && row < Rows;
    }

    // True if the cell is within bounds AND not blocked by an obstacle.
    public bool IsFree(int col, int row)
    {
        if (col < 0 || col >= Cols || row < 0 || row >= Rows) return false;
        return !_blocked[col, row];
    }

    // True if the cell is blocked. Returns false for out-of-bounds cells
    // (callers should check IsFree which handles both cases).
    public bool IsBlocked(int col, int row)
    {
        if (col < 0 || col >= Cols || row < 0 || row >= Rows) return false;
        return _blocked[col, row];
    }
}