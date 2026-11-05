// VENATOR SOURCE GUIDE: Defines the strongly typed JSON request and response shapes exchanged by the browser and web API.
// Documentation-only update: existing executable statements and safety checks are unchanged.

namespace DroneNavigation.Web.Models;

// Lightweight data shapes used over the wire between the browser and the API.
// These mirror the shapes the frontend will send/receive as JSON.
//
// Why separate from the domain types (Point, Obstacle) in the library?
//   - Domain types are immutable structs. JSON deserialization wants mutable
//     classes with parameterless constructors.
//   - DTOs can evolve independently of the algorithm-side types.
//   - Clear boundary between "wire format" and "internal representation".

// PointDto: Defines the strongly typed JSON request and response shapes exchanged by the browser and web API.
public class PointDto
{
    public double X { get; set; }
    public double Y { get; set; }
}

// ObstacleDto: Defines the strongly typed JSON request and response shapes exchanged by the browser and web API.
public class ObstacleDto
{
    public double X { get; set; }
    public double Y { get; set; }
    public double Radius { get; set; }
    public double Height { get; set; }
}

// PlanRequest: Defines the strongly typed JSON request and response shapes exchanged by the browser and web API.
public class PlanRequest
{
    public List<ObstacleDto> Obstacles { get; set; } = new();
    public PointDto Start { get; set; } = new();
    public PointDto Goal { get; set; } = new();
    public double AltitudeMeters { get; set; } = 20.0;
}

// PlanResponse: Defines the strongly typed JSON request and response shapes exchanged by the browser and web API.
public class PlanResponse
{
    public bool Success { get; set; }
    public string Message { get; set; } = "";
    public List<PointDto> Waypoints { get; set; } = new();
    public int RawWaypointCount { get; set; }
}

// FlyRequest: Defines the strongly typed JSON request and response shapes exchanged by the browser and web API.
public class FlyRequest
{
    public List<PointDto> Waypoints { get; set; } = new();
    public double AltitudeMeters { get; set; } = 20.0;
}
// ConnectionRequest: Defines the strongly typed JSON request and response shapes exchanged by the browser and web API.
public class ConnectionRequest
{
    public string Port { get; set; } = "";
    public int Baud { get; set; } = 57600;
}
// DemoEnableRequest: Defines the strongly typed JSON request and response shapes exchanged by the browser and web API.
public class DemoEnableRequest
{
    public bool Enabled { get; set; }
    public bool Confirmed { get; set; }
}
// DemoRequest: Defines the strongly typed JSON request and response shapes exchanged by the browser and web API.
public class DemoRequest
{
    public string? Mode { get; set; }
    public double? Altitude { get; set; }
    public double? Lat { get; set; }
    public double? Lon { get; set; }
    public double? Heading { get; set; }
    public bool PropsOffConfirmed { get; set; }
}
