using System;
using UnityEngine;

public class CarController : Drivable
{
    private WheelController[] wheels;
    private Rigidbody rb;
    
    [Header("Car Properties")]
    public float motorTorque = 2000f;
    public float brakeTorque = 2000f;
    public float maxSpeed = 20f;
    public float steeringRange = 30f;
    public float steeringRangeAtMaxSpeed = 10f;

    public void Awake()
    {
        rb = GetComponent<Rigidbody>();
        wheels = GetComponentsInChildren<WheelController>();
    }

    public float forwardSpeed = 0;
    public override void Drive(float inputDrive, float inputTurn)
    {
        forwardSpeed = Vector3.Dot(transform.forward, rb.linearVelocity);
        float speedFactor = Mathf.InverseLerp(0, maxSpeed, Mathf.Abs(forwardSpeed)); // Normalized speed factor

        // Reduce motor torque and steering at high speeds for better handling
        float currentMotorTorque = Mathf.Lerp(motorTorque, 0, speedFactor);
        float currentSteerRange = Mathf.Lerp(steeringRange, steeringRangeAtMaxSpeed, speedFactor);

        // Determine if the player is accelerating or trying to reverse
        bool isAccelerating = Mathf.Sign(inputDrive) == Mathf.Sign(forwardSpeed);
        foreach (var wheel in wheels)
        {
            // Apply steering to wheels that support steering
            if (wheel.steerable)
            {
                wheel.wheelCollider.steerAngle = inputTurn * currentSteerRange;
            }

            if (isAccelerating)
            {
                // Apply torque to motorized wheels
                if (wheel.motorized)
                {
                    wheel.wheelCollider.motorTorque = inputDrive * currentMotorTorque;
                }

                // Release brakes when accelerating
                wheel.wheelCollider.brakeTorque = 0f;
            }
            else
            {
                // Apply brakes when reversing direction
                wheel.wheelCollider.motorTorque = 0f;
                wheel.wheelCollider.brakeTorque = Mathf.Abs(inputTurn) * brakeTorque;
            }

            wheel.wheelCollider.GetWorldPose(out Vector3 position, out Quaternion rotation);
            wheel.wheelModel.position = position;
            wheel.wheelModel.rotation = rotation;
        }
    }

    public override void Restart()
    {
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        foreach (WheelController w in wheels)
        {
            w.wheelCollider.motorTorque = 0;
            w.wheelCollider.steerAngle = 0;
        }
    }
    
}
