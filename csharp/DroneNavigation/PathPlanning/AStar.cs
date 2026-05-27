namespace DroneNavigation.PathPlanning;

using DroneNavigation.Geometry;

// A* pathfinding over a Grid.
//
// Algorithm overview:
//   Maintain an "open set" of cells we have reached but not yet finished exploring.
//   Each cell has three scores:
//     g(n) = the actual cost of the cheapest known path from start to n
//     h(n) = a heuristic estimate of the remaining cost from n to goal
//     f(n) = g + h, the estimated total cost of a path through n
//   Repeatedly pop the cell with the lowest f from the open set, examine its
//   neighbors, and update their scores if we have found a cheaper path. Stop when
//   we pop the goal.
//
// Correctness depends on the heuristic being ADMISSIBLE: it must never overestimate
// the true remaining cost. We use OCTILE distance, which is the exact minimum-move
// cost on an 8-connected grid where diagonals cost sqrt(2) and cardinals cost 1.
// Admissible by construction.
//
// Returns a list of cell-center world points from start to goal, INCLUSIVE.
// Returns an empty list if no path exists.
public static class AStar
{
    // Cost of a single cardinal (N/S/E/W) move.
    private const double CardinalCost = 1.0;

    // Cost of a single diagonal (NE/NW/SE/SW) move.
    private static readonly double DiagonalCost = Math.Sqrt(2);

    // The 8 neighbor offsets: (col delta, row delta, move cost).
    // Cardinals first, diagonals second — slightly favors straighter paths
    // when costs tie, by virtue of being examined first.
    private static readonly (int dc, int dr, double cost)[] Neighbors = new[]
    {
        ( 0,  1, CardinalCost),   // N
        ( 0, -1, CardinalCost),   // S
        ( 1,  0, CardinalCost),   // E
        (-1,  0, CardinalCost),   // W
        ( 1,  1, DiagonalCost),   // NE
        (-1,  1, DiagonalCost),   // NW
        ( 1, -1, DiagonalCost),   // SE
        (-1, -1, DiagonalCost),   // SW
    };

    public static List<Point> FindPath(Grid grid, Point startWorld, Point goalWorld)
    {
        // Convert world coordinates to grid cells, bailing out if either is off-grid
        // or sitting on a blocked cell.
        if (!grid.TryWorldToCell(startWorld, out int startCol, out int startRow))
            return new List<Point>();
        if (!grid.TryWorldToCell(goalWorld, out int goalCol, out int goalRow))
            return new List<Point>();
        if (!grid.IsFree(startCol, startRow)) return new List<Point>();
        if (!grid.IsFree(goalCol, goalRow)) return new List<Point>();

        // gScore[cell] = best known cost from start to this cell.
        // cameFrom[cell] = the predecessor on the best known path, used to rebuild
        // the path once we reach the goal.
        Dictionary<(int, int), double> gScore = new Dictionary<(int, int), double>();
        Dictionary<(int, int), (int, int)> cameFrom = new Dictionary<(int, int), (int, int)>();

        // PriorityQueue<T, TPriority> is a min-heap built into .NET — pops the
        // item with the lowest priority first. We use f-score as the priority.
        PriorityQueue<(int col, int row), double> open =
            new PriorityQueue<(int col, int row), double>();

        gScore[(startCol, startRow)] = 0;
        open.Enqueue((startCol, startRow), Heuristic(startCol, startRow, goalCol, goalRow));

        while (open.Count > 0)
        {
            (int col, int row) current = open.Dequeue();

            // Goal reached — rebuild and return the path.
            if (current.col == goalCol && current.row == goalRow)
                return ReconstructPath(grid, cameFrom, current);

            double currentG = gScore[current];

            // Examine all 8 neighbors.
            foreach ((int dc, int dr, double moveCost) in Neighbors)
            {
                int nCol = current.col + dc;
                int nRow = current.row + dr;

                if (!grid.IsFree(nCol, nRow)) continue;

                // Prevent diagonal "corner cutting" through a wall.
                // If we are moving diagonally, both cardinal neighbors that share
                // the corner must also be free. Otherwise we would be slipping
                // through a 1-cell gap between two blocking cells.
                if (dc != 0 && dr != 0)
                {
                    if (!grid.IsFree(current.col + dc, current.row)) continue;
                    if (!grid.IsFree(current.col, current.row + dr)) continue;
                }

                double tentativeG = currentG + moveCost;
                (int, int) neighbor = (nCol, nRow);

                // If we have not seen this neighbor before, or this path to it is
                // cheaper than any previously known path, update and enqueue.
                if (!gScore.TryGetValue(neighbor, out double existingG) || tentativeG < existingG)
                {
                    gScore[neighbor] = tentativeG;
                    cameFrom[neighbor] = current;
                    double f = tentativeG + Heuristic(nCol, nRow, goalCol, goalRow);
                    open.Enqueue(neighbor, f);
                }
            }
        }

        // Open set drained without reaching the goal — no path exists.
        return new List<Point>();
    }

    // Octile distance: the exact minimum cost on an 8-connected grid.
    //   max(|dx|, |dy|) cardinal steps along the longer axis,
    //   plus (sqrt(2) - 1) * min(|dx|, |dy|) extra for the diagonal portion.
    // Equivalent formulation: take min(|dx|,|dy|) diagonals and (max - min) cardinals.
    private static double Heuristic(int col, int row, int goalCol, int goalRow)
    {
        int dx = Math.Abs(col - goalCol);
        int dy = Math.Abs(row - goalRow);
        return CardinalCost * Math.Max(dx, dy)
             + (DiagonalCost - CardinalCost) * Math.Min(dx, dy);
    }

    // Walks the cameFrom chain backward from the goal to rebuild the path,
    // then reverses it so it reads start-to-goal.
    private static List<Point> ReconstructPath(
        Grid grid,
        Dictionary<(int, int), (int, int)> cameFrom,
        (int col, int row) current)
    {
        List<Point> path = new List<Point> { grid.GetCellCenter(current.col, current.row) };
        while (cameFrom.TryGetValue(current, out (int, int) prev))
        {
            current = prev;
            path.Add(grid.GetCellCenter(current.col, current.row));
        }
        path.Reverse();
        return path;
    }
}