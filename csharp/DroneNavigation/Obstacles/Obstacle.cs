namespace DroneNavigation.Obstacles;

using DroneNavigation.Geometry;

public readonly struct Obstacle
{
    public double X { get; }
    public double Y { get; }
    public double Radius { get; }
    public double Height { get; }

    public Obstacle(double x, double y, double radius, double height) // This is fail-fast design. An obstacle with a non-positive radius doesn't make sense and likely indicates a bug. By throwing an exception immediately, we catch the error at its source instead of letting it propagate and cause more confusing issues later on.
    {
        if (radius <= 0) throw new ArgumentException("Radius must be positive", nameof(radius));
        if (height < 0) throw new ArgumentException("Height cannot be negative", nameof(height));

        X = x;
        Y = y;
        Radius = radius;
        Height = height;
    }

    public Point Center => new Point(X, Y);

    public bool BlocksAtAltitude(double altitudeMeters, double safetyBufferMeters) // If the obstacle's height plus the safety buffer is greater than the drone's altitude, then the obstacle blocks the drone at that altitude. This is how we determine if an obstacle is a threat to the drone at its current altitude.
    {
        return altitudeMeters < Height + safetyBufferMeters; // I added the safety buffer to the obstacle's height instead of subtracting it from the drone's altitude because it's more intuitive to think of the obstacle as being "taller" due to the safety buffer, rather than thinking of the drone as being "shorter". It's just a mental model thing. The math works out the same either way.
    }

    public override string ToString() => $"Obstacle@({X}, {Y}) r={Radius} h={Height}"; //I had to ask the AI to make this string representation more concise because the default one was too verbose and made the logs hard to read. This way, we can quickly see the obstacle's position, size, and height without wading through a lot of extra text.
}