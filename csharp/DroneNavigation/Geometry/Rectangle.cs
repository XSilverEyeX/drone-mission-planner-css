namespace DroneNavigation.Geometry;

public readonly struct Rectangle
{
    public double X { get; }
    public double Y { get; }
    public double Width { get; }
    public double Height { get; }

    public Rectangle(double x, double y, double width, double height)
    {//This is fail-fast design. A rectangle with negative width is meaningless and likely indicates a bug. By throwing an exception immediately, we catch the error at its source instead of letting it propagate and cause more confusing issues later on.
        if (width < 0) throw new ArgumentException("Width cannot be negative", nameof(width));
        if (height < 0) throw new ArgumentException("Height cannot be negative", nameof(height));

        X = x;
        Y = y;
        Width = width;
        Height = height;
    }

    public double MaxX => X + Width;
    public double MaxY => Y + Height;
    public double CenterX => X + Width / 2;
    public double CenterY => Y + Height / 2;

    public bool Contains(Point point)
    {// For a point to be inside the rectangle, all four conditions must hold. This is how the quadtree will decide which child quadrant an obstacle belongs in — "does this child's rectangle contain this point?"
        return point.X >= X
            && point.X <= MaxX
            && point.Y >= Y
            && point.Y <= MaxY;
    }

    public bool Intersects(Rectangle other) // Two rectangles do not intersect if one is completely to the left, right, above, or below the other. If none of those conditions are true, then they must intersect. CRAZY, confusing. This is how the quadtree will check if an obstacle's rectangle overlaps with a quadrant's rectangle — "does this rectangle intersect with this other rectangle?"
    {
        return !(other.X > MaxX // If the other rectangle is completely to the right of this one, they don't intersect
              || other.MaxX < X // If the other rectangle is completely to the left of this one, they don't intersect
              || other.Y > MaxY // If the other rectangle is completely above this one, they don't intersect
              || other.MaxY < Y); // If the other rectangle is completely below this one, they don't intersect
    }

    public override string ToString() => $"Rect({X}, {Y}, {Width}x{Height})";
}