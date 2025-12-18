
using System;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Net.WebSockets;
using UnityEngine;
// Install via Package Manager: com.unity.nuget.newtonsoft-json
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

public class SensorHubWebSocketClient : MonoBehaviour
{
    [Header("WebSocket server (FastAPI)")]
    // Change this if your FastAPI runs on 8000 (uvicorn default)
    public string serverUrl = "ws://localhost:8080/ws";

    [Header("Sensors to subscribe")]
    // Must exist in manager.adapters on the server
    public string[] sensorIds = new string[] { "sim1", "gps1" };

    private ClientWebSocket ws;
    private CancellationTokenSource cts;

    async void Start()
    {
        cts = new CancellationTokenSource();
        ws = new ClientWebSocket();

        try
        {
            await ws.ConnectAsync(new Uri(serverUrl), cts.Token);
            Debug.Log($"WS: Connected to {serverUrl}");

            // Send subscribe messages
            foreach (var sid in sensorIds)
            {
                var subMsg = new { action = "subscribe", sensor_id = sid };
                var payload = JsonConvert.SerializeObject(subMsg);
                await ws.SendAsync(Encoding.UTF8.GetBytes(payload),
                                   WebSocketMessageType.Text,
                                   endOfMessage: true,
                                   cancellationToken: cts.Token);
                Debug.Log($"WS: Sent subscribe for {sid}");
            }

            // Start polling loop (sender)
            _ = Task.Run(async () =>
            {
                while (ws.State == WebSocketState.Open && !cts.IsCancellationRequested)
                {
                    var pollMsg = new { action = "poll" };
                    var payload = JsonConvert.SerializeObject(pollMsg);
                    await ws.SendAsync(Encoding.UTF8.GetBytes(payload),
                                       WebSocketMessageType.Text,
                                       endOfMessage: true,
                                       cancellationToken: cts.Token);

                    await Task.Delay(100, cts.Token); // adjust polling interval
                }
            }, cts.Token);

            // Start receiver loop
            _ = Task.Run(async () =>
            {
                var buffer = new byte[16 * 1024];

                while (ws.State == WebSocketState.Open && !cts.IsCancellationRequested)
                {
                    var ms = new System.IO.MemoryStream();

                    WebSocketReceiveResult result;
                    do
                    {
                        result = await ws.ReceiveAsync(new ArraySegment<byte>(buffer), cts.Token);

                        if (result.MessageType == WebSocketMessageType.Close)
                        {
                            Debug.Log("WS: Server requested close");
                            await ws.CloseAsync(WebSocketCloseStatus.NormalClosure, "closing", cts.Token);
                            return;
                        }

                        ms.Write(buffer, 0, result.Count);
                    } while (!result.EndOfMessage);

                    var msg = Encoding.UTF8.GetString(ms.ToArray());

                    // Parse and route by 'type'
                    try
                    {
                        var root = JObject.Parse(msg);
                        var type = root.Value<string>("type");

                        if (!string.IsNullOrEmpty(type))
                        {
                            switch (type)
                            {
                                case "subscribed":
                                    {
                                        string sensorId = root.Value<string>("sensor_id");
                                        if (string.IsNullOrEmpty(sensorId)) sensorId = "(unknown)";
                                        Debug.Log($"WS: Subscribed to {sensorId}");
                                        break;
                                    }

                                case "poll-result":
                                    {
                                        Debug.Log($"WS: Poll result: {msg}");

                                        var data = root["data"] as JObject;
                                        if (data != null)
                                        {
                                            foreach (var kv in data)
                                            {
                                                string sid = kv.Key;
                                                JToken payloadJson = kv.Value;
                                                // TODO: Update your UI/state with sid + payloadJson
                                                // Example:
                                                // Debug.Log($"WS: {sid} => {payloadJson.ToString(Formatting.None)}");
                                            }
                                        }
                                        break;
                                    }

                                case "error":
                                    {
                                        string err = root.Value<string>("error");
                                        if (string.IsNullOrEmpty(err)) err = "(unknown error)";
                                        Debug.LogWarning($"WS: Error: {err}");
                                        break;
                                    }

                                default:
                                    Debug.Log($"WS: Message: {msg}");
                                    break;
                            }
                        }
                        else
                        {
                            Debug.Log($"WS: Message: {msg}");
                        }
                    }
                    catch (Exception ex)
                    {
                        Debug.LogWarning($"WS: Failed to parse JSON: {ex.Message}\nRaw: {msg}");
                    }
                }
            }, cts.Token);
        }
        catch (Exception e)
        {
            Debug.LogError($"WS: Connect/send error: {e.Message}");
        }
    }

    private async void OnDestroy()
    {
        try
        {
            cts?.Cancel();
            if (ws != null && ws.State == WebSocketState.Open)
            {
                await ws.CloseAsync(WebSocketCloseStatus.NormalClosure,
                                    "component destroyed",
                                    CancellationToken.None);
            }
        }
        catch (Exception e)
        {
            Debug.LogWarning($"WS: Close error: {e.Message}");
        }
        finally
        {
            ws?.Dispose();
            cts?.Dispose();
        }
    }
}
