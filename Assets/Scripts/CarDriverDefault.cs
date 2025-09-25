#define VISUALIZE

using System;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;

namespace Tommy.Scripts.Training
{
    public class CarDriverDefault : Agent
    {
        private Vector3 target;

        [SerializeField] private CarController car;

        [Header("Rewards")]
        [SerializeField] private float goalReward = 1f;
        [SerializeField] private float deathPenalty = -1f;

        private float stepPenalty;

        private Color drawingColor;
        private LineRenderer line;

        // If you're using your own spawner, keep this; otherwise delete.
        private RoadSpawner roadSpawner;

        [Header("Goal & Roads")]
        public Transform goal;
        [SerializeField] private Transform parentRoads;

        [SerializeField] private bool debug;

        public override void Initialize()
        {
            // Line between agent and target (debug)
            line = GetComponent<LineRenderer>();
            if (line != null) line.positionCount = 2;

            // Per-step penalty scaled to episode length
            if (MaxStep > 0)
                stepPenalty = deathPenalty / MaxStep;
            else
                stepPenalty = 0f;

            // Optional: use your existing RoadSpawner to place car/goal
            if (parentRoads != null) roadSpawner = new RoadSpawner(parentRoads);
        }

        public override void OnEpisodeBegin()
        {
            // Random spawn (optional, depending on your scene setup)
            if (parentRoads != null && roadSpawner != null)
            {
                roadSpawner.RandomLocation(transform);
                if (goal != null) roadSpawner.RandomLocation(goal);
            }

            // Ensure target is defined
            target = goal != null ? goal.position : transform.position;

            // Reset physics
            var rb = GetComponent<Rigidbody>();
            if (rb != null)
            {
#if UNITY_6000_0_OR_NEWER
                rb.linearVelocity = Vector3.zero;
#else
                rb.velocity = Vector3.zero;
#endif
                rb.angularVelocity = Vector3.zero;
            }
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            // Observation 1–2: target direction in agent local frame (x,z)
            Vector3 aToB = target - transform.position;
            aToB.y = 0f;

            Vector3 forward = transform.forward; forward.y = 0f; forward.Normalize();
            Vector3 right   = transform.right;   right.y   = 0f; right.Normalize();

            float localX = Vector3.Dot(aToB, right);
            float localZ = Vector3.Dot(aToB, forward);

            sensor.AddObservation(new Vector2(localX, localZ));

            // Observation 3: normalized forward speed (safe-guarded)
            float normSpeed = (car != null && car.maxSpeed > 1e-6f)
                ? (car.forwardSpeed / car.maxSpeed)
                : 0f;
            sensor.AddObservation(normSpeed);
        }

        public override void OnActionReceived(ActionBuffers actionBuffers)
        {
            // Keep target synced to goal (if assigned)
            if (goal != null) target = goal.position;

            // Debug line (optional)
            if (line != null)
            {
                if (debug)
                {
                    line.enabled = true;
                    line.positionCount = 2;
                    line.SetPosition(0, transform.position);
                    line.SetPosition(1, target);
                }
                else
                {
                    line.enabled = false;
                }
            }

            // Per-step penalty (only shaping retained)
            if (stepPenalty != 0f)
                AddReward(stepPenalty);

            // Actions
            float inputPower = actionBuffers.ContinuousActions[0];
            float inputSteeringAngle = actionBuffers.ContinuousActions[1];
            car.Drive(inputPower, inputSteeringAngle);
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            var cont = actionsOut.ContinuousActions;
            cont[0] = Input.GetAxis("Vertical");
            cont[1] = Input.GetAxis("Horizontal");
        }

        private void OnTriggerEnter(Collider other)
        {
            if (other.CompareTag("Death"))
            {
                SetReward(deathPenalty);
                EndEpisode();
            }
            else if (other.CompareTag("Goal"))
            {
                AddReward(goalReward);
                EndEpisode();
            }
        }

#if (UNITY_EDITOR && VISUALIZE)
        private void OnDrawGizmos()
        {
            if (!debug) return;
            Gizmos.color = drawingColor;
            Gizmos.DrawSphere(target, 1f);
            Gizmos.DrawLine(transform.position, target);
        }
#endif
    }
}