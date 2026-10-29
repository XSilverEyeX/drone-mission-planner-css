namespace DroneNavigation.Drone;

using System.Net.Sockets;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;

// Each request owns its TCP exchange: polling can continue during command ACK waits.
// Mutating requests are serialized/rejected by the bridge, which owns vehicle state.
public class DroneClient : IDisposable
{
    private readonly string _host;
    private readonly int _port;
    public bool IsConnected { get; private set; }
    public DroneClient(string host = "127.0.0.1", int port = 5555) { _host = host; _port = port; }
    public void Connect() { IsConnected = true; }
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
    public DroneResponse GetStatus() => Send(new { cmd = "get_status" });
    public DroneResponse GetPosition() => GetStatus();
    public DroneResponse GetPorts() => Send(new { cmd = "get_ports" });
    public DroneResponse SetHome() => Send(new { cmd = "set_home" });
    public DroneResponse Arm() => Send(new { cmd = "arm" });
    public DroneResponse Takeoff(double altitudeMeters) => Send(new { cmd = "takeoff", altitude = altitudeMeters });
    public DroneResponse Goto(double lat, double lon, double altitudeMeters) => Send(new { cmd = "goto", lat, lon, altitude = altitudeMeters });
    public DroneResponse GotoLocal(double x, double y, double altitudeMeters) => Send(new { cmd = "goto_local", x, y, altitude = altitudeMeters });
    public DroneResponse Land() => Send(new { cmd = "land" });
    public void Dispose() { IsConnected = false; GC.SuppressFinalize(this); }
}

public class DroneResponse
{
    [JsonPropertyName("status")] public string Status { get; set; } = "";
    [JsonPropertyName("message")] public string? Message { get; set; }
    [JsonPropertyName("data")] public JsonElement? Data { get; set; }
    public bool IsOk => Status == "ok";
    public override string ToString() => $"[{Status}] {Message}";
}
