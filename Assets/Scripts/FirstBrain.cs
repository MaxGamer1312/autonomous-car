using System;
using System.Timers;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class FirstBrain : Agent
{
    public Transform target;
    public float moveSpeed = 1;
    public float turnSpeed = 1;

    private Rigidbody rb;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
    }
    
    public override void OnEpisodeBegin()
    {
        transform.localPosition = new Vector3(0, 1, 0);
        transform.localRotation = Quaternion.identity;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(this.transform.position);
        sensor.AddObservation(target.transform.position);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        float inputPower = actionBuffers.ContinuousActions[0] * moveSpeed;
        float inputSteeringAngle = actionBuffers.ContinuousActions[1] * turnSpeed;
        rb.AddForce(transform.forward * inputPower, ForceMode.Acceleration);
        rb.AddTorque(Vector3.up * inputSteeringAngle, ForceMode.Acceleration);

    }

    

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Vertical");
        continuousActionsOut[1] = Input.GetAxis("Horizontal");
    }

    public void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Goal"))
        {
            AddReward(1f);
            EndEpisode();
        } 
        else if (other.CompareTag("Death"))
        {
            AddReward(-1f);
            EndEpisode();
        }
    }
}
