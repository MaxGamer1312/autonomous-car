using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

public class PiTCPManager : MonoBehaviour
{
    [Serializable]
    public class CarConnection
    {
        public string carName = "Car 1";
        public string IP = "10.159.67.47";
        public int port = 65432;
        public GameObject carObject; // We use this to identify which car is sending the data

        [HideInInspector] public TcpClient client;
        [HideInInspector] public NetworkStream stream;
    }

    [Header("Fleet Network Settings")]
    public List<CarConnection> cars = new List<CarConnection>();

    [Serializable]
    public class CarCommand
    {
        public string cmd;
        public int speed;
        public int steer_angle;
    }

    void Start()
    {
        ConnectToAllCars();
    }

    void ConnectToAllCars()
    {
        foreach (var car in cars)
        {
            try
            {
                car.client = new TcpClient(car.IP, car.port);
                car.stream = car.client.GetStream();
                Debug.Log($"Connected to {car.carName} ({car.IP}) successfully!");
            }
            catch (Exception e)
            {
                Debug.LogError($"Socket connection error for {car.carName} ({car.IP}): " + e.Message);
            }
        }
    }

    // Now it takes the GameObject as a parameter to find the right connection
    public void SendMovementCommand(GameObject requestingCar, string commandType, int speedValue, int steeringAngle)
    {
        // Find the specific car connection that matches the GameObject asking to send data
        CarConnection car = cars.Find(c => c.carObject == requestingCar);

        if (car == null || car.client == null || !car.client.Connected)
        {
            return; // Ignore if not connected or if the car isn't in the list
        }

        try
        {
            CarCommand data = new CarCommand();
            data.cmd = commandType;
            data.speed = speedValue;
            data.steer_angle = steeringAngle;

            string jsonString = JsonUtility.ToJson(data) + "\n";
            byte[] bytesToSend = Encoding.UTF8.GetBytes(jsonString);

            car.stream.Write(bytesToSend, 0, bytesToSend.Length);
            car.stream.Flush();
        }
        catch (Exception e)
        {
            Debug.LogError($"Error sending data to {car.carName}: " + e.Message);
        }
    }

    void OnApplicationQuit()
    {
        foreach (var car in cars)
        {
            if (car.stream != null) car.stream.Close();
            if (car.client != null) car.client.Close();
        }
    }
}