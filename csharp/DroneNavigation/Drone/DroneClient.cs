// VENATOR SOURCE GUIDE: Opens short-lived local TCP exchanges between the C# API and the Python MAVLink bridge.
// Documentation-only update: existing executable statements and safety checks are unchanged.

namespace DroneNavigation.Drone;

using System.Net.Sockets;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;

// Each request owns its TCP exchange: polling can continue during command ACK waits.
// Mutating requests are serialized/rejected by the bridge, which owns vehicle state.
// DroneClient: Opens short-lived local TCP exchanges between the C# API and the Python MAVLink bridge.
public class DroneClient : IDisposable
{
    private readonly string _host;
    private readonly int _port;
    public bool IsConnected { get; private set; }
    public DroneClient(string host = "127.0.0.1", int port = 5555) { _host = host; _port = port; }
    // Records a connection-ready client state.
    public void Connect() { IsConnected = true; }
    // Serializes a command and reads its bridge response.
    public DroneResponse Send(object command)
    {
        using var socket = new TcpClient();
        try
        {
            socket.ConnectAsync(_host, _port).WaitAsync(TimeSpan.FromSeconds(2)).GetAwaiter().GetResult();
            socket.ReceiveTimeout = 30000;
            socket.SendTimeout = 2000;
            using var stream = socket.GetStream();
            using var reader = new StreamReader(stream, Encoding.UTF8);
            using var writer = new StreamWriter(stream, new UTF8Encoding(false)) { AutoFlush = true, NewLine = "\n" };
            writer.WriteLine(JsonSerializer.Serialize(command));
            var line = reader.ReadLine() ?? throw new IOException("Bridge closed the connection");
            IsConnected = true;
            return JsonSerializer.Deserialize<DroneResponse>(line) ?? throw new IOException("Invalid bridge response");
        }
        catch
        {
            IsConnected = false;
            // No retries: a lost response can mean a command was executed.
            throw;
        }
    }
    // Retrieves a fresh telemetry status snapshot.
    public DroneResponse GetStatus() => Send(new { cmd = "get_status" });
    // Implements GetPosition for this component; inputs and return values follow its declared signature.
    public DroneResponse GetPosition() => GetStatus();
    // Lists serial ports exposed by the bridge.
    public DroneResponse GetPorts() => Send(new { cmd = "get_ports" });
    // Implements SetHome for this component; inputs and return values follow its declared signature.
    public DroneResponse SetHome() => Send(new { cmd = "set_home" });
    // Implements Arm for this component; inputs and return values follow its declared signature.
    public DroneResponse Arm() => Send(new { cmd = "arm" });
    // Implements Takeoff for this component; inputs and return values follow its declared signature.
    public DroneResponse Takeoff(double altitudeMeters) => Send(new { cmd = "takeoff", altitude = altitudeMeters });
    // Implements Goto for this component; inputs and return values follow its declared signature.
    public DroneResponse Goto(double lat, double lon, double altitudeMeters) => Send(new { cmd = "goto", lat, lon, altitude = altitudeMeters });
    // Implements GotoLocal for this component; inputs and return values follow its declared signature.
    public DroneResponse GotoLocal(double x, double y, double altitudeMeters) => Send(new { cmd = "goto_local", x, y, altitude = altitudeMeters });
    // Implements Land for this component; inputs and return values follow its declared signature.
    public DroneResponse Land() => Send(new { cmd = "land" });
    // Implements Dispose for this component; inputs and return values follow its declared signature.
    public void Dispose() { IsConnected = false; GC.SuppressFinalize(this); }
}

// DroneResponse: Opens short-lived local TCP exchanges between the C# API and the Python MAVLink bridge.
public class DroneResponse
{
    [JsonPropertyName("status")] public string Status { get; set; } = "";
    [JsonPropertyName("message")] public string? Message { get; set; }
    [JsonPropertyName("data")] public JsonElement? Data { get; set; }
    public bool IsOk => Status == "ok";
    // Implements ToString for this component; inputs and return values follow its declared signature.
    public override string ToString() => $"[{Status}] {Message}";
}
