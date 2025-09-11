using System;
using System.Timers;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class SimpleBrain : Agent
{
    public Transform target;

    [SerializeField]
    private Drivable car;
    public override void Initialize()
    {
        
    }
    
    public override void OnEpisodeBegin()
    {
        transform.localPosition = new Vector3(0, 1.5f, 0);
        transform.localRotation = Quaternion.identity;
        car.Restart();
        
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(this.transform.position);
        sensor.AddObservation(target.transform.position);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        float inputPower = actionBuffers.ContinuousActions[0];
        float inputSteeringAngle = actionBuffers.ContinuousActions[1];
        car.Drive(inputPower, inputSteeringAngle);
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