using System;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

public class PiTCPClient : MonoBehaviour
{
    [Header("Network Settings")]
    public string piIP = "10.159.67.47"; // Replace with your Pi's IP
    public int port = 65432;

    private TcpClient client;
    private NetworkStream stream;

    // 1. Define the data structure
    // Unity requires the [Serializable] tag to convert this class into JSON text.
    [Serializable]
    public class CarCommand
    {
        public string cmd;
        public int speed;
        public int steer_angle;
    }

    void Start()
    {
        ConnectToPi();
    }

    void ConnectToPi()
    {
        try
        {
            // Initialize the socket and connect
            client = new TcpClient(piIP, port);
            stream = client.GetStream();
            Debug.Log("Connected to Raspberry Pi successfully!");
        }
        catch (Exception e)
        {
            Debug.LogError("Socket connection error: " + e.Message);
        }
    }

    // 2. The method to trigger from your game inputs
    public void SendMovementCommand(string commandType, int speedValue, int steeringAngle)
    {
        if (client == null || !client.Connected)
        {
            Debug.LogWarning("Cannot send data. Not connected to the Pi.");
            return;
        }

        try
        {
            // Populate the data object
            CarCommand data = new CarCommand();
            data.cmd = commandType;
            data.speed = speedValue;
            data.steer_angle = steeringAngle;

            // Convert the object into a JSON string
            string jsonString = JsonUtility.ToJson(data) + "\n";

            // Encode the string into bytes (UTF-8) and send it over the stream
            byte[] bytesToSend = Encoding.UTF8.GetBytes(jsonString);
            stream.Write(bytesToSend, 0, bytesToSend.Length);

            // Force the stream to send the data immediately
            stream.Flush();
        }
        catch (Exception e)
        {
            Debug.LogError("Error sending data to Pi: " + e.Message);
        }
    }

    // Example of calling the method during gameplay
    void Update()
    {
    }

    // 3. Clean up the connection when the game closes
    void OnApplicationQuit()
    {
        if (stream != null) stream.Close();
        if (client != null) client.Close();
    }
}