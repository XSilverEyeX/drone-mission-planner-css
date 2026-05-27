namespace DroneNavigation.Geometry;

public readonly struct Point // Points are value types thats why we use struct instead of class
//readonly — once a Point is created, its X and Y can never change. Aka: immutability
{
    public double X { get; }
    public double Y { get; }

    public Point(double x, double y)
    {
        X = x;
        Y = y;
    }

    public double DistanceTo(Point other)
    {
        double dx = X - other.X;
        double dy = Y - other.Y;
        return Math.Sqrt(dx * dx + dy * dy);
    }
//Things that I didnt know; Structs in C# automatically get a default equality check that compares all fields, so point1.Equals(point2) works out of the box. We can override it later if we want custom behavior, but the default is fine.
    public override string ToString() => $"({X}, {Y})";
}
//GPS coordinates and physics simulations need precision. A float has ~7 digits of precision; a double has ~15. GPS latitude/longitude routinely needs 6+ digits past the decimal — float would round them and you'd see your drone jump around. double is the safe default for spatial math.