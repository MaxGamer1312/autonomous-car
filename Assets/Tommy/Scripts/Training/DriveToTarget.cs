using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class DriveToTarget : Agent
{
    public Transform target;

    [SerializeField]
    private CarController car;


    // giving score based on distance to goal
    
    public override void OnEpisodeBegin()
    {
        timer = 0;
        float x = UnityEngine.Random.Range(-22f, 22f);
        float z = UnityEngine.Random.Range(-22f, 22f);
        transform.localPosition = new Vector3(x, 1.5f, z);
        originalDistance = (target.transform.position - transform.position).magnitude;
        x = UnityEngine.Random.Range(-22f, 22f);
        z = UnityEngine.Random.Range(-22f, 22f);
        target.transform.localPosition = new Vector3(x, 1, z);
        //transform.localRotation = Quaternion.identity;
        transform.localRotation = Quaternion.Euler(0, Random.Range(-180f, 180f), 0);
        car.Restart();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(new Vector2(transform.localPosition.x, transform.localPosition.z));
        sensor.AddObservation(new Vector2(target.localPosition.x, target.localPosition.y));

        Vector3 dirTo = target.position - transform.position;
        dirTo.y = 0;
        Vector2 dirTo2D = new Vector2(dirTo.x, dirTo.z).normalized;
        
        Vector3 forward = transform.forward;
        Vector2 forward2D = new Vector2(forward.x, forward.z).normalized;
        

        float signedAngle = Vector2.SignedAngle(forward2D, dirTo2D);
        sensor.AddObservation(signedAngle);

        sensor.AddObservation(car.forwardSpeed);
    }

    private float timer;
    private float originalDistance;
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // timer += Time.deltaTime;
        // if (timer >= 30)
        // {
        //     float distanceToTarget = (target.transform.position - transform.position).magnitude;
        //     
        //     AddReward(Mathf.Lerp(0, 3f, distanceToTarget/ originalDistance));
        //     EndEpisode();
        // }
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
            AddReward(6f);
            EndEpisode();
        } 
        else if (other.CompareTag("Death"))
        {
            AddReward(-1f);
            EndEpisode();
        }
    }
    
}