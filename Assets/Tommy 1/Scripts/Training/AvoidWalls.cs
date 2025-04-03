using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class AvoidWalls : Agent
{
    [SerializeField]
    private CarController car;

    // Maximum distance the rays will check for obstacles.
    public float rayDistance = 20f;

    // Toggle to draw rays in the Scene view for debugging.
    public bool debugRays = true;

    // Variables to store ray observations for ML (each ray provides a normalized distance and a tag code).
    private float frontDistance, frontTag;
    private float leftDistance, leftTag;
    private float rightDistance, rightTag;

    public override void OnEpisodeBegin()
    {
        // Restart the car controller.
        car.Restart();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        GetRaycastObservation(transform.forward, out frontDistance, out frontTag);
        GetRaycastObservation(-transform.right, out leftDistance, out leftTag);
        GetRaycastObservation(transform.right, out rightDistance, out rightTag);

        sensor.AddObservation(frontDistance / rayDistance);
        sensor.AddObservation(frontTag);
        sensor.AddObservation(leftDistance / rayDistance);
        sensor.AddObservation(leftTag);
        sensor.AddObservation(rightDistance / rayDistance);
        sensor.AddObservation(rightTag);
    }

    /// <summary>
    /// Casts a ray from the agent in the given direction and returns the hit distance and tag code.
    /// If an object with the tag "Car" is hit, it is ignored.
    /// </summary>
    /// <param name="direction">Direction in which to cast the ray.</param>
    /// <param name="distance">Distance to the first valid hit (or rayDistance if no hit).</param>
    /// <param name="tagCode">
    /// Encoded tag:
    /// 0: no hit,
    /// 1: hit "Wall",
    /// 2: hit "Death",
    /// 3: hit any other object.
    /// </param>
    private void GetRaycastObservation(Vector3 direction, out float distance, out float tagCode)
    {
        Ray ray = new Ray(transform.position, direction);
        RaycastHit[] hits = Physics.RaycastAll(ray, rayDistance);

        float minDistance = rayDistance;
        RaycastHit validHit = new RaycastHit();
        bool foundValidHit = false;

        foreach (RaycastHit hit in hits)
        {
            // Ignore our own car.
            if (hit.collider.CompareTag("Car"))
                continue;

            if (hit.distance < minDistance)
            {
                minDistance = hit.distance;
                validHit = hit;
                foundValidHit = true;
            }
        }

        if (foundValidHit)
        {
            distance = validHit.distance;

            if (validHit.collider.CompareTag("Wall"))
                tagCode = 1;
            else if (validHit.collider.CompareTag("Death"))
                tagCode = 2;
            else
                tagCode = 3;

            // Choose debug color: if the hit is tagged "Goal", draw green; otherwise red.
            if (debugRays)
            {
                Color debugColor = validHit.collider.CompareTag("Goal") ? Color.green : Color.red;
                Debug.DrawRay(transform.position, direction * distance, debugColor, 0.1f);
            }
        }
        else
        {
            distance = rayDistance;
            tagCode = 0;
            if (debugRays)
                Debug.DrawRay(transform.position, direction * rayDistance, Color.green, 0.1f);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        float inputPower = actionBuffers.ContinuousActions[0];
        float inputSteering = actionBuffers.ContinuousActions[1];
        car.Drive(inputPower, inputSteering);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Vertical");
        continuousActionsOut[1] = Input.GetAxis("Horizontal");
    }

    // Continuously redraw the debug rays so you can see their current length and color.
    private void Update()
    {
        if (!debugRays)
            return;

        DrawDebugRay(transform.forward);
        DrawDebugRay(-transform.right);
        DrawDebugRay(transform.right);
    }

    /// <summary>
    /// Helper method to draw the debug ray in the Scene view.
    /// It casts a ray and draws it up to the first valid hit.
    /// If the hit object is tagged "Goal", the ray is drawn green.
    /// </summary>
    private void DrawDebugRay(Vector3 direction)
    {
        Ray ray = new Ray(transform.position, direction);
        RaycastHit[] hits = Physics.RaycastAll(ray, rayDistance);
        float drawDistance = rayDistance;
        string hitTag = null;

        foreach (RaycastHit hit in hits)
        {
            if (hit.collider.CompareTag("Car"))
                continue;
            if (hit.distance < drawDistance)
            {
                drawDistance = hit.distance;
                hitTag = hit.collider.tag;
            }
        }

        Color color;
        if (drawDistance < rayDistance)
        {
            // If the closest hit is tagged "Goal", use green; otherwise, use red.
            color = (hitTag == "Goal") ? Color.green : Color.red;
        }
        else
        {
            color = Color.green;
        }

        Debug.DrawRay(transform.position, direction * drawDistance, color, 0.1f);
    }
}
