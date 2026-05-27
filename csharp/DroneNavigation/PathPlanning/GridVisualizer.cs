namespace DroneNavigation.PathPlanning;

using System.Text;
using DroneNavigation.Geometry;

// Renders a Grid (and optionally an A* path) two ways:
//   1. ToAscii() — multi-line string for printing to console. Great for debugging.
//   2. ToSvg()   — SVG markup you can write to a .svg file and open in a browser.
//                  Suitable for project writeups and presentation slides.
//
// Both renderers use the same conventions:
//   '.' or white  — free cell
//   '#' or grey   — blocked cell
//   'S' or green  — start cell
//   'G' or red    — goal cell
//   '*' or blue   — cell on the planned path (other than start/goal)
public static class GridVisualizer
{
    public static string ToAscii(Grid grid, List<Point>? path = null, Point? start = null, Point? goal = null)
    {
        // Build a set of path cells for O(1) lookup during render.
        HashSet<(int, int)> pathCells = new HashSet<(int, int)>();
        if (path != null)
        {
            foreach (Point p in path)
            {
                if (grid.TryWorldToCell(p, out int c, out int r))
                    pathCells.Add((c, r));
            }
        }

        (int, int)? startCell = null;
        (int, int)? goalCell = null;
        if (start.HasValue && grid.TryWorldToCell(start.Value, out int sc, out int sr))
            startCell = (sc, sr);
        if (goal.HasValue && grid.TryWorldToCell(goal.Value, out int gc, out int gr))
            goalCell = (gc, gr);

        // Print top row first (highest row index), since Y increases upward.
        StringBuilder sb = new StringBuilder();
        for (int row = grid.Rows - 1; row >= 0; row--)
        {
            for (int col = 0; col < grid.Cols; col++)
            {
                char ch;
                if (startCell.HasValue && startCell.Value == (col, row)) ch = 'S';
                else if (goalCell.HasValue && goalCell.Value == (col, row)) ch = 'G';
                else if (pathCells.Contains((col, row))) ch = '*';
                else if (grid.IsBlocked(col, row)) ch = '#';
                else ch = '.';
                sb.Append(ch);
            }
            sb.AppendLine();
        }
        return sb.ToString();
    }

    public static string ToSvg(
        Grid grid,
        List<Point>? path = null,
        Point? start = null,
        Point? goal = null,
        int pixelsPerCell = 20)
    {
        int width = grid.Cols * pixelsPerCell;
        int height = grid.Rows * pixelsPerCell;

        StringBuilder sb = new StringBuilder();
        sb.AppendLine($"<svg xmlns='http://www.w3.org/2000/svg' width='{width}' height='{height}' viewBox='0 0 {width} {height}'>");
        sb.AppendLine($"  <rect width='{width}' height='{height}' fill='white' />");

        // Draw blocked cells.
        for (int col = 0; col < grid.Cols; col++)
        {
            for (int row = 0; row < grid.Rows; row++)
            {
                if (!grid.IsBlocked(col, row)) continue;
                int px = col * pixelsPerCell;
                // Flip Y: row 0 is at the bottom in world coords, but SVG Y increases downward.
                int py = (grid.Rows - 1 - row) * pixelsPerCell;
                sb.AppendLine($"  <rect x='{px}' y='{py}' width='{pixelsPerCell}' height='{pixelsPerCell}' fill='#888' />");
            }
        }

        // Draw light grid lines.
        for (int col = 0; col <= grid.Cols; col++)
        {
            int x = col * pixelsPerCell;
            sb.AppendLine($"  <line x1='{x}' y1='0' x2='{x}' y2='{height}' stroke='#eee' stroke-width='1' />");
        }
        for (int row = 0; row <= grid.Rows; row++)
        {
            int y = row * pixelsPerCell;
            sb.AppendLine($"  <line x1='0' y1='{y}' x2='{width}' y2='{y}' stroke='#eee' stroke-width='1' />");
        }

        // Draw the planned path as a polyline through cell centers.
        if (path != null && path.Count > 1)
        {
            StringBuilder points = new StringBuilder();
            foreach (Point p in path)
            {
                if (!grid.TryWorldToCell(p, out int c, out int r)) continue;
                int cx = c * pixelsPerCell + pixelsPerCell / 2;
                int cy = (grid.Rows - 1 - r) * pixelsPerCell + pixelsPerCell / 2;
                points.Append($"{cx},{cy} ");
            }
            sb.AppendLine($"  <polyline points='{points.ToString().Trim()}' fill='none' stroke='#2a7' stroke-width='3' />");
        }

        // Draw start (green circle) and goal (red circle).
        if (start.HasValue && grid.TryWorldToCell(start.Value, out int startC, out int startR))
        {
            int cx = startC * pixelsPerCell + pixelsPerCell / 2;
            int cy = (grid.Rows - 1 - startR) * pixelsPerCell + pixelsPerCell / 2;
            sb.AppendLine($"  <circle cx='{cx}' cy='{cy}' r='{pixelsPerCell / 2 - 2}' fill='#0a0' />");
        }
        if (goal.HasValue && grid.TryWorldToCell(goal.Value, out int goalC, out int goalR))
        {
            int cx = goalC * pixelsPerCell + pixelsPerCell / 2;
            int cy = (grid.Rows - 1 - goalR) * pixelsPerCell + pixelsPerCell / 2;
            sb.AppendLine($"  <circle cx='{cx}' cy='{cy}' r='{pixelsPerCell / 2 - 2}' fill='#a00' />");
        }

        sb.AppendLine("</svg>");
        return sb.ToString();
    }
}