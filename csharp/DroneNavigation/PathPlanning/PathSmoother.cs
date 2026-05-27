namespace DroneNavigation.PathPlanning;

using DroneNavigation.Geometry;

// Smooths a grid-based A* path by removing redundant intermediate waypoints.
// The raw A* output has one waypoint per grid cell — fine for correctness but
// produces a visibly "blocky" path: lots of tiny steps along grid lines.
//
// The smoothing algorithm is called "line-of-sight" or "string-pulling":
//   Starting from waypoint 0, find the FARTHEST waypoint we can reach in a
//   straight line without crossing any blocked cells. Add that as the next
//   smoothed waypoint. Repeat from there. Stop when we have included the goal.
//
// This is greedy but works well in practice. The result is the same path,
// just with the obviously-redundant cells removed. The drone flies clean
// diagonals instead of stair-stepping along grid edges.
//
// Trade-off: O(n^2) in path length in the worst case (we may check every
// pair). For drone-scale paths (dozens to hundreds of cells), this is fine.
public static class PathSmoother
{
    public static List<Point> Smooth(Grid grid, List<Point> rawPath)
    {
        if (rawPath.Count <= 2) return new List<Point>(rawPath);

        List<Point> smoothed = new List<Point> { rawPath[0] };
        int anchor = 0;

        while (anchor < rawPath.Count - 1)
        {
            // Find the farthest waypoint visible in a straight line from anchor.
            int farthest = anchor + 1;
            for (int candidate = anchor + 2; candidate < rawPath.Count; candidate++)
            {
                if (HasLineOfSight(grid, rawPath[anchor], rawPath[candidate]))
                    farthest = candidate;
                else
                    break;
            }

            smoothed.Add(rawPath[farthest]);
            anchor = farthest;
        }

        return smoothed;
    }

    // Returns true if a straight line from "from" to "to" passes only through
    // free grid cells. We sample the line at half-cell intervals — fine enough
    // to catch any blocked cell the line would enter.
    private static bool HasLineOfSight(Grid grid, Point from, Point to)
    {
        double dx = to.X - from.X;
        double dy = to.Y - from.Y;
        double distance = Math.Sqrt(dx * dx + dy * dy);

        // Sample every half-cell along the line.
        double stepSize = grid.CellSize / 2.0;
        int steps = (int)Math.Ceiling(distance / stepSize);
        if (steps == 0) return true;

        for (int i = 0; i <= steps; i++)
        {
            double t = (double)i / steps;
            double sampleX = from.X + dx * t;
            double sampleY = from.Y + dy * t;

            Point sample = new Point(sampleX, sampleY);
            if (!grid.TryWorldToCell(sample, out int col, out int row))
                return false;
            if (!grid.IsFree(col, row))
                return false;
        }

        return true;
    }
}