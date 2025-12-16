
using System;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Net.WebSockets;
using UnityEngine;

public class SensorHubWebSocketClient : MonoBehaviour
{
    public string serverUrl = "ws://localhost:8080/ws";
    public string[] sensorIds = new string[] {"sim1", "gps1"};
    private ClientWebSocket ws;

    async void Start()
    {
        ws = new ClientWebSocket();
        await ws.ConnectAsync(new Uri(serverUrl), CancellationToken.None);
        foreach (var sid in sensorIds)
        {
            var sub = Encoding.UTF8.GetBytes($"{{"action":"subscribe","sensor_id":"{sid}"}}");
            await ws.SendAsync(new ArraySegment<byte>(sub), WebSocketMessageType.Text, true, CancellationToken.None);
        }
        // Start polling loop
        _ = Task.Run(async () =>
        {
            while (ws.State == WebSocketState.Open)
            {
                var poll = Encoding.UTF8.GetBytes("{"action":"poll"}");
                await ws.SendAsync(new ArraySegment<byte>(poll), WebSocketMessageType.Text, true, CancellationToken.None);
                var buf = new byte[16384];
                var res = await ws.ReceiveAsync(new ArraySegment<byte>(buf), CancellationToken.None);
                var msg = Encoding.UTF8.GetString(buf, 0, res.Count);
                Debug.Log($"WS: {msg}");
                await Task.Delay(100);
            }
        });
    }
}
