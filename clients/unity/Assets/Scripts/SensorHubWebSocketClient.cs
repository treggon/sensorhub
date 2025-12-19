using System;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Net.WebSockets;
using System.Collections.Concurrent;
using System.Linq;
using UnityEngine;

// Install via Package Manager: com.unity.nuget.newtonsoft-json
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

public class SensorHubWebSocketClient : MonoBehaviour
{
    [Header("WebSocket server (FastAPI)")]
    public string serverUrl = "ws://127.0.0.1:8082/ws";

    [Header("Sensors to subscribe")]
    public string[] sensorIds = new string[] { "sim1", "gps1", "rplidar_s2" };

    [Header("Sensor to visualize")]
    public string lidarSensorId = "rplidar_s2";

    [Header("Visualizer")]
    public RPLidarVisualizer lidarVisualizer;

    [Header("Polling / heartbeat")]
    [Tooltip("Milliseconds between poll messages")]
    public int pollIntervalMs = 100;
    [Tooltip("Send ping every N milliseconds (0 disables heartbeat)")]
    public int pingIntervalMs = 5000;
    [Tooltip("If no data or pong is seen for this many ms, force reconnect")]
    public int connectionTimeoutMs = 15000;

    [Header("Reconnect backoff")]
    [Tooltip("Initial backoff in ms")]
    public int reconnectBackoffMs = 1000;
    [Tooltip("Max backoff in ms")]
    public int reconnectBackoffMaxMs = 15000;

    [Header("Logging")]
    public bool verboseLogs = true;

    private ClientWebSocket ws;
    private CancellationTokenSource cts;
    private Task receiveTask;
    private Task sendTask;
    private Task pingTask;

    private DateTime lastRxUtc = DateTime.MinValue;
    private bool shuttingDown = false;
    private System.Random rng = new System.Random();

    // Queue to move data safely from background receive thread to Unity main thread
    private readonly ConcurrentQueue<(float[] anglesDeg, float[] distances)> lidarQueue =
        new ConcurrentQueue<(float[] anglesDeg, float[] distances)>();

    private bool firstLidarFrameLogged = false;

    #region Unity lifecycle
    async void Start()
    {
        // Auto-link visualizer if not wired in the Inspector
        if (lidarVisualizer == null)
        {
            lidarVisualizer = FindFirstObjectByType<RPLidarVisualizer>();
            if (lidarVisualizer != null && verboseLogs)
                Debug.Log($"WS: Auto-linked visualizer: {lidarVisualizer.gameObject.name}");
            else
                Debug.LogWarning("WS: No RPLidarVisualizer found; points won’t render.");
        }

        await EnsureConnectedAsync();
    }

    private void Update()
    {
        // Drain lidar queue on main thread and feed the visualizer
        while (lidarQueue.TryDequeue(out var item))
        {
            lidarVisualizer?.ShowScan(item.anglesDeg, item.distances);
        }

        // Heartbeat timeout check
        if (ws != null && ws.State == WebSocketState.Open && connectionTimeoutMs > 0)
        {
            var sinceRx = (DateTime.UtcNow - lastRxUtc).TotalMilliseconds;
            if (lastRxUtc != DateTime.MinValue && sinceRx > connectionTimeoutMs)
            {
                Debug.LogWarning($"WS: Connection stale ({sinceRx:F0} ms). Reconnecting...");
                _ = ForceReconnectAsync();
            }
        }
    }

    private async void OnApplicationPause(bool pause)
    {
        if (pause)
        {
            await SafeCloseAsync("pause");
        }
        else
        {
            await EnsureConnectedAsync();
        }
    }

    private async void OnDestroy()
    {
        shuttingDown = true;
        await SafeCloseAsync("destroy");
    }
    #endregion

    #region Connection management
    private async Task EnsureConnectedAsync()
    {
        if (ws != null && ws.State == WebSocketState.Open)
            return;

        int delay = reconnectBackoffMs;

        while (!shuttingDown)
        {
            try
            {
                cts?.Cancel();
                cts?.Dispose();
                cts = new CancellationTokenSource();

                ws?.Dispose();
                ws = new ClientWebSocket();

                if (verboseLogs) Debug.Log($"WS: Connecting to {serverUrl}...");
                await ws.ConnectAsync(new Uri(serverUrl), cts.Token);
                lastRxUtc = DateTime.UtcNow;
                if (verboseLogs) Debug.Log("WS: Connected.");

                // (Re)subscribe
                await ResubscribeAsync();

                // Start tasks
                receiveTask = Task.Run(() => ReceiveLoopAsync(cts.Token), cts.Token);
                sendTask = Task.Run(() => PollLoopAsync(cts.Token), cts.Token);
                pingTask = Task.Run(() => PingLoopAsync(cts.Token), cts.Token);

                // Reset backoff once we are connected
                delay = reconnectBackoffMs;
                return; // connected
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"WS: Connect failed: {ex.Message}");
                if (shuttingDown) return;

                // Exponential backoff + jitter
                int jitter = rng.Next(0, 250);
                int wait = Math.Min(delay + jitter, reconnectBackoffMaxMs);
                if (verboseLogs) Debug.Log($"WS: Retry in {wait} ms...");
                await Task.Delay(wait);
                delay = Math.Min(delay * 2, reconnectBackoffMaxMs);
            }
        }
    }

    private async Task ForceReconnectAsync()
    {
        try
        {
            cts?.Cancel();
            await SafeCloseAsync("reconnect");
        }
        catch { }
        finally
        {
            await EnsureConnectedAsync();
        }
    }

    private async Task SafeCloseAsync(string reason)
    {
        try
        {
            if (ws != null && ws.State == WebSocketState.Open)
            {
                await ws.CloseAsync(WebSocketCloseStatus.NormalClosure, reason, CancellationToken.None);
            }
        }
        catch (Exception ex)
        {
            if (verboseLogs) Debug.LogWarning($"WS: Close error ({reason}): {ex.Message}");
        }
        finally
        {
            try { cts?.Cancel(); } catch { }
            try { receiveTask?.Wait(50); } catch { }
            try { sendTask?.Wait(50); } catch { }
            try { pingTask?.Wait(50); } catch { }
            receiveTask = null; sendTask = null; pingTask = null;

            ws?.Dispose();
            ws = null;

            cts?.Dispose();
            cts = null;
        }
    }

    private async Task ResubscribeAsync()
    {
        foreach (var sid in sensorIds)
        {
            var subMsg = new { action = "subscribe", sensor_id = sid };
            var payload = JsonConvert.SerializeObject(subMsg);
            await ws.SendAsync(Encoding.UTF8.GetBytes(payload),
                               WebSocketMessageType.Text, true, CancellationToken.None);
            if (verboseLogs) Debug.Log($"WS: Sent subscribe for {sid}");
        }
    }
    #endregion

    #region Loops
    private async Task ReceiveLoopAsync(CancellationToken token)
    {
        var buffer = new byte[128 * 1024]; // allow larger frames
        try
        {
            while (!token.IsCancellationRequested && ws != null && ws.State == WebSocketState.Open)
            {
                var ms = new System.IO.MemoryStream();
                WebSocketReceiveResult result;
                do
                {
                    result = await ws.ReceiveAsync(new ArraySegment<byte>(buffer), token);
                    if (result.MessageType == WebSocketMessageType.Close)
                    {
                        if (verboseLogs) Debug.Log("WS: Server requested close.");
                        await ForceReconnectAsync();
                        return;
                    }
                    ms.Write(buffer, 0, result.Count);
                }
                while (!result.EndOfMessage);

                lastRxUtc = DateTime.UtcNow;
                var msg = Encoding.UTF8.GetString(ms.ToArray());
                HandleMessage(msg);
            }
        }
        catch (OperationCanceledException) { /* normal during shutdown */ }
        catch (Exception ex)
        {
            Debug.LogWarning($"WS: Receive loop error: {ex.Message}");
            await ForceReconnectAsync();
        }
    }

    private async Task PollLoopAsync(CancellationToken token)
    {
        try
        {
            while (!token.IsCancellationRequested && ws != null)
            {
                if (ws.State == WebSocketState.Open)
                {
                    var pollMsg = new { action = "poll" };
                    var payload = JsonConvert.SerializeObject(pollMsg);
                    await ws.SendAsync(Encoding.UTF8.GetBytes(payload),
                                       WebSocketMessageType.Text, true, token);
                }
                await Task.Delay(pollIntervalMs, token);
            }
        }
        catch (OperationCanceledException) { }
        catch (Exception ex)
        {
            Debug.LogWarning($"WS: Poll loop error: {ex.Message}");
            await ForceReconnectAsync();
        }
    }

    private async Task PingLoopAsync(CancellationToken token)
    {
        if (pingIntervalMs <= 0) return;
        try
        {
            while (!token.IsCancellationRequested)
            {
                if (ws != null && ws.State == WebSocketState.Open)
                {
                    var pingMsg = new { action = "ping" };
                    var payload = JsonConvert.SerializeObject(pingMsg);
                    await ws.SendAsync(Encoding.UTF8.GetBytes(payload),
                                       WebSocketMessageType.Text, true, token);
                }
                await Task.Delay(pingIntervalMs, token);
            }
        }
        catch (OperationCanceledException) { }
        catch (Exception ex)
        {
            Debug.LogWarning($"WS: Ping loop error: {ex.Message}");
            await ForceReconnectAsync();
        }
    }
    #endregion

    #region Message handling
    private void HandleMessage(string msg)
    {
        try
        {
            var root = JObject.Parse(msg);
            var type = root.Value<string>("type");
            if (string.IsNullOrEmpty(type))
                return;

            switch (type)
            {
                case "subscribed":
                    {
                        string sensorId = root.Value<string>("sensor_id") ?? "(unknown)";
                        if (verboseLogs) Debug.Log($"WS: Subscribed to {sensorId}");
                        break;
                    }

                case "pong":
                    {
                        // Heartbeat OK
                        if (verboseLogs) Debug.Log("WS: Pong");
                        break;
                    }

                case "poll-result":
                    {
                        var data = root["data"] as JObject;
                        if (data == null) return;

                        // Flexible payload resolution
                        JToken payloadCandidate = data[lidarSensorId] ?? data;
                        JToken payload = payloadCandidate["data"] ?? payloadCandidate;

                        float[] angles = TryGetFloatArray(payload, "angles", "theta", "angle");
                        float[] distances = TryGetFloatArray(payload, "distances", "ranges", "distance_mm", "range_mm", "range", "r", "d");

                        if ((angles == null || distances == null) && payload["points"] is JArray points)
                        {
                            var aList = new System.Collections.Generic.List<float>(points.Count);
                            var dList = new System.Collections.Generic.List<float>(points.Count);
                            foreach (var p in points)
                            {
                                float a = TryGetFloat(p, "angle", "theta");
                                float d = TryGetFloat(p, "distance", "range", "distance_mm", "range_mm", "r", "d");
                                aList.Add(a);
                                dList.Add(d);
                            }
                            angles = aList.ToArray();
                            distances = dList.ToArray();
                        }

                        if (angles == null || distances == null || angles.Length != distances.Length)
                        {
                            if (!firstLidarFrameLogged)
                            {
                                firstLidarFrameLogged = true;
                                var keys = (payload as JObject)?.Properties().Select(k => k.Name);
                                Debug.LogWarning($"WS: Lidar payload shape not matched. Keys = [{string.Join(",", keys ?? Array.Empty<string>())}]");
                            }
                            return;
                        }

                        // Units: radians → degrees
                        bool isRadians = angles.Max() > 3.5f;
                        float[] anglesDeg = isRadians ? angles.Select(a => a * Mathf.Rad2Deg).ToArray() : angles;

                        // Heuristic mm vs meters
                        float med = Median(distances);
                        if (lidarVisualizer != null)
                        {
                            lidarVisualizer.distanceScale = (med > 10f) ? 0.001f : 1f;
                            if (!firstLidarFrameLogged || verboseLogs)
                                Debug.Log($"WS: Lidar arrays ok. angles={angles.Length}, distances={distances.Length}, anglesUnit={(isRadians ? "radians" : "degrees")}, distanceMedian={med:F1}, distanceScale={lidarVisualizer.distanceScale:F3}");
                        }
                        else
                        {
                            Debug.LogWarning("WS: Lidar data received but visualizer is null.");
                        }

                        lidarQueue.Enqueue((anglesDeg, distances));
                        firstLidarFrameLogged = true;
                        break;
                    }

                case "error":
                    {
                        string err = root.Value<string>("error") ?? "(unknown)";
                        Debug.LogWarning($"WS: Error: {err}");
                        break;
                    }
            }
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"WS: Failed to parse JSON: {ex.Message}\nRaw: {msg}");
        }
    }

    private static float[] TryGetFloatArray(JToken obj, params string[] keys)
    {
        foreach (var k in keys)
        {
            var t = obj?[k];
            if (t != null && t.Type == JTokenType.Array)
                return t.ToObject<float[]>();
        }
        return null;
    }

    private static float TryGetFloat(JToken obj, params string[] keys)
    {
        foreach (var k in keys)
        {
            var t = obj?[k];
            if (t != null && (t.Type == JTokenType.Float || t.Type == JTokenType.Integer))
                return t.Value<float>();
        }
        return 0f;
    }

    private static float Median(float[] arr)
    {
        if (arr == null || arr.Length == 0) return 0f;
        var copy = (float[])arr.Clone();
        Array.Sort(copy);
        int mid = copy.Length / 2;
        if (copy.Length % 2 == 0) return 0.5f * (copy[mid - 1] + copy[mid]);
        return copy[mid];
    }
}
#endregion