namespace DroneNavigation.Drone;

using System.Net.Sockets;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;


// ============================================================
// DRONE CLIENT
// ============================================================

// This class handles communication between C# and the Python bridge.
//
// Communication:
// Program.cs -> DroneClient.cs -> Python bridge
//
// The connection uses TCP on:
// 127.0.0.1:5555
//
// Commands are sent to Python as JSON.
// Python sends a JSON response back.
public class DroneClient : IDisposable
{

    // ---------- Connection settings ----------

    // Python bridge is running on the same computer
    private const string DefaultHost = "127.0.0.1";

    // TCP port used by the Python bridge
    private const int DefaultPort = 5555;


    // Stores the host and port being used
    private readonly string _host;
    private readonly int _port;


    // TCP connection to the Python bridge
    private TcpClient? _socket;


    // Used to receive messages from Python
    private StreamReader? _reader;


    // Used to send messages to Python
    private StreamWriter? _writer;


    // Only allows one command to use the connection at a time
    // This keeps commands and responses from getting mixed together
    private readonly object _sendLock = new();



    // ============================================================
    // SETUP
    // ============================================================

    // Creates the DroneClient using the default IP and port
    public DroneClient(
        string host = DefaultHost,
        int port = DefaultPort
    )
    {
        _host = host;
        _port = port;
    }


    // Returns true if C# currently has a TCP connection to Python
    public bool IsConnected =>
        _socket?.Connected ?? false;



    // ============================================================
    // CONNECT TO PYTHON
    // ============================================================

    // Opens the TCP connection between C# and the Python bridge
    public void Connect()
    {

        // Create a new TCP socket
        _socket = new TcpClient();


        // Connect to Python at 127.0.0.1:5555
        _socket.Connect(
            _host,
            _port
        );


        // Get the network stream used to send and receive data
        NetworkStream stream =
            _socket.GetStream();


        // Reader receives responses from Python
        _reader =
            new StreamReader(
                stream,
                Encoding.UTF8
            );


        // Writer sends commands to Python
        _writer =
            new StreamWriter(
                stream,
                new UTF8Encoding(false)
            )
            {
                // Immediately sends data instead of waiting
                AutoFlush = true,

                // Python expects each JSON command on its own line
                NewLine = "\n"
            };
    }



    // ============================================================
    // DRONE COMMANDS
    // ============================================================

    // These functions make it easier for Program.cs to send
    // commands without manually creating JSON every time.


    // Tell Python to save the current drone location as home
    public DroneResponse SetHome() =>
        Send(
            new
            {
                cmd = "set_home"
            }
        );


    // Send an ARM command
    public DroneResponse Arm() =>
        Send(
            new
            {
                cmd = "arm"
            }
        );


    // Tell the drone to take off to a selected altitude
    public DroneResponse Takeoff(
        double altitudeMeters
    ) =>
        Send(
            new
            {
                cmd = "takeoff",
                altitude = altitudeMeters
            }
        );


    // Fly to a GPS latitude and longitude
    public DroneResponse Goto(
        double lat,
        double lon,
        double altitudeMeters
    ) =>
        Send(
            new
            {
                cmd = "goto",
                lat,
                lon,
                altitude = altitudeMeters
            }
        );


    // Fly to a point using local x/y coordinates
    //
    // x = meters east/west
    // y = meters north/south
    //
    // This is useful because the path planner works in meters
    // instead of directly using GPS coordinates.
    public DroneResponse GotoLocal(
        double x,
        double y,
        double altitudeMeters
    ) =>
        Send(
            new
            {
                cmd = "goto_local",
                x,
                y,
                altitude = altitudeMeters
            }
        );


    // Send a LAND command
    public DroneResponse Land() =>
        Send(
            new
            {
                cmd = "land"
            }
        );


    // Ask Python for the drone's current GPS position
    public DroneResponse GetPosition() =>
        Send(
            new
            {
                cmd = "get_position"
            }
        );


    // Ask Python which COM ports Windows currently detects
    //
    // The GUI uses this to find the SiK telemetry radio
    // and display the available ports in the dropdown.
    public DroneResponse GetPorts() =>
        Send(
            new
            {
                cmd = "get_ports"
            }
        );



    // ============================================================
    // SEND COMMAND TO PYTHON
    // ============================================================

    // All of the commands above eventually come through this function.
    //
    // Example:
    //
    // C#:
    // Arm()
    //
    // becomes:
    //
    // {"cmd":"arm"}
    //
    // That JSON is sent to the Python bridge.
    private DroneResponse Send(
        object command
    )
    {

        // Only let one command use the TCP connection at a time
        //
        // This is important because /api/status may be checking
        // the drone while another command is also being sent.
        lock (_sendLock)
        {

            // Make sure Connect() was called first
            if (
                _writer == null ||
                _reader == null
            )
            {
                throw new InvalidOperationException(
                    "Not connected. Call Connect() first."
                );
            }


            // Convert the C# command into JSON
            string json =
                JsonSerializer.Serialize(
                    command
                );


            // Send the JSON command to Python
            _writer.WriteLine(
                json
            );


            // Wait for Python to send one response back
            string? line =
                _reader.ReadLine();


            // If Python closed the connection without responding
            if (line == null)
            {
                throw new IOException(
                    "Bridge closed the connection without responding."
                );
            }


            // Convert Python's JSON response back into C#
            DroneResponse? response =
                JsonSerializer.Deserialize<DroneResponse>(
                    line
                );


            // Return the response to Program.cs
            //
            // If the JSON could not be read, return an error instead
            return response ??
                new DroneResponse
                {
                    Status = "error",

                    Message =
                        "Could not parse bridge response"
                };
        }
    }



    // ============================================================
    // CLOSE CONNECTION
    // ============================================================

    // Closes the reader, writer, and TCP connection
    // when DroneClient is no longer being used
    public void Dispose()
    {
        _reader?.Dispose();
        _writer?.Dispose();
        _socket?.Dispose();

        GC.SuppressFinalize(this);
    }
}



// ============================================================
// PYTHON RESPONSE
// ============================================================

// Represents the JSON response sent back by Python.
//
// Example:
//
// {
//     "status": "ok",
//     "message": "Command completed",
//     "data": { ... }
// }
//
// This lets the rest of the C# program work with the Python
// response like a normal C# object.
public class DroneResponse
{

    // "ok" or "error"
    [JsonPropertyName("status")]
    public string Status
    {
        get;
        set;
    } = "";


    // Message explaining what happened
    [JsonPropertyName("message")]
    public string? Message
    {
        get;
        set;
    }


    // Optional information returned by Python
    // Example: GPS position or COM-port list
    [JsonPropertyName("data")]
    public JsonElement? Data
    {
        get;
        set;
    }


    // Quick way to check if Python said the command worked
    public bool IsOk =>
        Status == "ok";


    // Makes the response easier to print in the terminal
    public override string ToString() =>
        $"[{Status}] {Message ?? ""}" +
        (
            Data.HasValue
                ? $" data={Data}"
                : ""
        );
}