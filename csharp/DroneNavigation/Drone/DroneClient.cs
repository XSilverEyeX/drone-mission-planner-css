namespace DroneNavigation.Drone;

using System.Net.Sockets;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;

// C# client for the Python drone bridge.
//
// The bridge listens on TCP 127.0.0.1:5555 and accepts newline-delimited JSON
// commands. Each command gets exactly one response. This class wraps the wire
// protocol with strongly-typed methods so callers do not have to think about
// sockets or JSON.
//
// Usage:
//   using DroneClient client = new DroneClient();
//   client.Connect();
//   client.SetHome();
//   client.Arm();
//   client.Takeoff(20);
//   client.GotoLocal(50, 30, 20);
//   client.Land();
//
// Each method blocks until the bridge responds. Errors come back as a
// DroneResponse with Status == "error" — callers should check before continuing.
public class DroneClient : IDisposable
{
    private const string DefaultHost = "127.0.0.1";
    private const int DefaultPort = 5555;

    private readonly string _host;
    private readonly int _port;
    private TcpClient? _socket;
    private StreamReader? _reader;
    private StreamWriter? _writer;

    // Prevent multiple API requests from using the TCP stream at the same time.
    // This keeps each command paired with the correct Python response.
    private readonly object _sendLock = new();

    public DroneClient(string host = DefaultHost, int port = DefaultPort)
    {
        _host = host;
        _port = port;
    }

    public bool IsConnected => _socket?.Connected ?? false;

    public void Connect()
    {
        _socket = new TcpClient();
        _socket.Connect(_host, _port);
        NetworkStream stream = _socket.GetStream();
        _reader = new StreamReader(stream, Encoding.UTF8);
        _writer = new StreamWriter(stream, new UTF8Encoding(false))
        {
            AutoFlush = true,
            NewLine = "\n"
        };
    }

    // High-level commands — each sends a single JSON object and reads the response.

    public DroneResponse SetHome() => Send(new { cmd = "set_home" });

    public DroneResponse Arm() => Send(new { cmd = "arm" });

    public DroneResponse Takeoff(double altitudeMeters) =>
        Send(new { cmd = "takeoff", altitude = altitudeMeters });

    public DroneResponse Goto(double lat, double lon, double altitudeMeters) =>
        Send(new { cmd = "goto", lat, lon, altitude = altitudeMeters });

    // Flies to a point expressed as meters offset from home: x is east, y is north.
    // This is the method the path planner will use — it speaks in local meters.
    public DroneResponse GotoLocal(double x, double y, double altitudeMeters) =>
        Send(new { cmd = "goto_local", x, y, altitude = altitudeMeters });

    public DroneResponse Land() => Send(new { cmd = "land" });

    public DroneResponse GetPosition() => Send(new { cmd = "get_position" });

    // Requests the list of serial/COM ports detected by the Python bridge.
    // Used by the GUI to populate the telemetry COM-port dropdown.
    public DroneResponse GetPorts() => Send(new { cmd = "get_ports" });

    // Sends an arbitrary command object as JSON, reads one line of response,
    // and parses it back into a DroneResponse.
    private DroneResponse Send(object command)
    {
        // Only allow one command at a time to use the TCP connection.
        // This prevents /api/status and other API calls from mixing responses.
        lock (_sendLock)
        {
            if (_writer == null || _reader == null)
            {
                throw new InvalidOperationException(
                    "Not connected. Call Connect() first."
                );
            }

            // Convert the C# command into JSON.
            string json = JsonSerializer.Serialize(command);

            // Send one command to the Python bridge.
            _writer.WriteLine(json);

            // Read the response belonging to this command.
            string? line = _reader.ReadLine();

            if (line == null)
            {
                throw new IOException(
                    "Bridge closed the connection without responding."
                );
            }

            // Convert the Python response back into a DroneResponse.
            DroneResponse? response =
                JsonSerializer.Deserialize<DroneResponse>(line);

            return response ?? new DroneResponse
            {
                Status = "error",
                Message = "Could not parse bridge response"
            };
        }
    }

    public void Dispose()
    {
        _reader?.Dispose();
        _writer?.Dispose();
        _socket?.Dispose();
        GC.SuppressFinalize(this);
    }
}

// Strongly typed representation of a bridge response. Matches the JSON shape:
//   {"status": "ok" | "error", "message": "...", "data": {...optional...}}
public class DroneResponse
{
    [JsonPropertyName("status")]
    public string Status { get; set; } = "";

    [JsonPropertyName("message")]
    public string? Message { get; set; }

    [JsonPropertyName("data")]
    public JsonElement? Data { get; set; }

    public bool IsOk => Status == "ok";

    public override string ToString() =>
        $"[{Status}] {Message ?? ""}" + (Data.HasValue ? $" data={Data}" : "");
}