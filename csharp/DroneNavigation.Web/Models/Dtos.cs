namespace DroneNavigation.Web.Models;

// Lightweight data shapes used over the wire between the browser and the API.
// These mirror the shapes the frontend will send/receive as JSON.
//
// Why separate from the domain types (Point, Obstacle) in the library?
//   - Domain types are immutable structs. JSON deserialization wants mutable
//     classes with parameterless constructors.
//   - DTOs can evolve independently of the algorithm-side types.
//   - Clear boundary between "wire format" and "internal representation".

public class PointDto
{
    public double X { get; set; }
    public double Y { get; set; }
}

public class ObstacleDto
{
    public double X { get; set; }
    public double Y { get; set; }
    public double Radius { get; set; }
    public double Height { get; set; }
}

public class PlanRequest
{
    public List<ObstacleDto> Obstacles { get; set; } = new();
    public PointDto Start { get; set; } = new();
    public PointDto Goal { get; set; } = new();
    public double AltitudeMeters { get; set; } = 20.0;
}

public class PlanResponse
{
    public bool Success { get; set; }
    public string Message { get; set; } = "";
    public List<PointDto> Waypoints { get; set; } = new();
    public int RawWaypointCount { get; set; }
}

public class FlyRequest
{
    public List<PointDto> Waypoints { get; set; } = new();
    public double AltitudeMeters { get; set; } = 20.0;
}