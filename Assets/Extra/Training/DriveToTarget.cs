using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class DriveToTarget : Agent
{
    [Header("Environment References")]
    public Transform roadsParent;   // Assign the "Roads" parent (with Road children) for this environment.
    public Transform target;        // Assign the target GameObject for this environment.

    [SerializeField]
    private CarController car;

    // Episode timers and distance tracking.
    private float timer;
    private float originalDistance;
    private float prevDistance; // For incremental progress reward.

    // For detecting movement
    private Vector3 lastPosition;    // We'll track the agent's position each step.

    // Tuning parameters.
    public float progressRewardMultiplier = 2f; // Multiplies the incremental progress reward.
    public float timeoutRewardMultiplier = 3f;  // Base timeout reward (if episode times out).
    public float goalBaseReward = 10f;           // Base reward when reaching the goal.
    public float goalBonusMax = 10f;             // Maximum bonus for reaching the goal quickly.

    // Negative reward if the agent moves less than this distance in one step
    public float movementThreshold = 0.5f;
    public float movementPenalty = -1f;  // How much to penalize each step of insufficient movement

    public override void OnEpisodeBegin()
    {
        timer = 0f;

        // Get the number of available roads.
        int childCount = roadsParent.childCount;
        if (childCount == 0)
        {
            Debug.LogError("No road objects found in roadsParent!");
            return;
        }

        // --- Spawn the Car on a Road ---
        int carRoadIndex = Random.Range(0, childCount);
        Transform carRoadPicked = roadsParent.GetChild(carRoadIndex);

        // Place the car at the chosen road's position with a Y offset.
        transform.position = new Vector3(
            carRoadPicked.position.x,
            carRoadPicked.position.y + 1.5f,
            carRoadPicked.position.z
        );

        // Set a random rotation for the car.
        transform.rotation = Quaternion.Euler(0, Random.Range(-180f, 180f), 0);

        // --- Spawn the Target on a Road ---
        if (childCount > 1)
        {
            // If multiple roads exist, choose a different one for the target
            int targetRoadIndex = carRoadIndex;
            while (targetRoadIndex == carRoadIndex)
            {
                targetRoadIndex = Random.Range(0, childCount);
            }
            Transform targetRoadPicked = roadsParent.GetChild(targetRoadIndex);

            // Place the target
            target.position = new Vector3(
                targetRoadPicked.position.x,
                targetRoadPicked.position.y + 1.5f,
                targetRoadPicked.position.z
            );
        }
        else
        {
            // Only one road is available, place target near the agent with an offset
            Vector3 basePos = carRoadPicked.position;
            float offsetX = Random.Range(3f, 10f) * (Random.value > 0.5f ? 1 : -1);
            float offsetZ = Random.Range(3f, 10f) * (Random.value > 0.5f ? 1 : -1);
            target.position = new Vector3(
                basePos.x + offsetX,
                basePos.y + 1.5f,
                basePos.z + offsetZ
            );
        }

        // Distance calculations
        originalDistance = Vector3.Distance(target.position, transform.position);
        prevDistance = originalDistance;

        // Reset or restart the car controller
        car.Restart();

        // Initialize lastPosition for movement tracking
        lastPosition = transform.position;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Observations: positions, angle to target, and car speed
        sensor.AddObservation(new Vector2(transform.localPosition.x, transform.localPosition.z));
        sensor.AddObservation(new Vector2(target.localPosition.x, target.localPosition.z));

        Vector3 dirTo = target.position - transform.position;
        dirTo.y = 0;
        Vector2 dirTo2D = new Vector2(dirTo.x, dirTo.z).normalized;

        Vector3 forward = transform.forward;
        Vector2 forward2D = new Vector2(forward.x, forward.z).normalized;

        float signedAngle = Vector2.SignedAngle(forward2D, dirTo2D);
        sensor.AddObservation(signedAngle);

        sensor.AddObservation(car.forwardSpeed);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        timer += Time.deltaTime;

        // Calculate the current distance to the target
        float currentDistance = Vector3.Distance(target.position, transform.position);

        // 1. Incremental progress reward
        float progress = prevDistance - currentDistance;
        AddReward(progress * progressRewardMultiplier);
        prevDistance = currentDistance;

        // 2. Check movement
        float distanceMoved = Vector3.Distance(transform.position, lastPosition);
        if (distanceMoved < movementThreshold)
        {
            // Penalize if the agent hasn't moved much
            AddReward(movementPenalty);
        }
        lastPosition = transform.position;

        // 3. Timeout after 30 seconds
        if (timer >= 30f)
        {
            float timeoutReward = timeoutRewardMultiplier * (1 - (currentDistance / originalDistance));
            AddReward(timeoutReward);
            EndEpisode();
        }

        // Drive the car
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

    private void OnTriggerEnter(Collider other)
    {
        // Goal
        if (other.CompareTag("Goal"))
        {
            float timeBonus = Mathf.Clamp((30f - timer) / 30f, 0f, 1f) * goalBonusMax;
            AddReward(goalBaseReward + timeBonus);
            EndEpisode();
        }
        // Death
        else if (other.CompareTag("Death"))
        {
            AddReward(-10f);
            EndEpisode();
        }
    }
}
