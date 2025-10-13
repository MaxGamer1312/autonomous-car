#define VISUALIZE

using System;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;

namespace Tommy.Scripts.Training
{
    public class CarDriverNew : Agent
    {
        private Vector3 target;

        [SerializeField] private CarController car;

        [Header("Rewards")]
        [SerializeField] private float goalReward = 1f;
        [SerializeField] private float deathPenalty = -1f;

        [Header("Anti-Stall")]
        [SerializeField] private float stillPenalty = -0.001f;
        [SerializeField] private float stallSpeedThreshold = 0.1f;
        [SerializeField] private int stallTimeoutSteps = 100;
        private int _stallCounter = 0;

        private Vector3 _prevPos;
        private float stepPenalty;

        private Color drawingColor;
        public LayerMask ground;
        private LineRenderer line;

        private ParkingSpawner parkingSpawner;

        [Header("Goal & Parking")]
        public Transform goal;
        private Transform parentParking; 
        [SerializeField] private bool debug;

        public override void Initialize()
        {
            line = GetComponent<LineRenderer>();
            if (line != null) line.positionCount = 2;

            stepPenalty = (MaxStep > 0) ? deathPenalty / MaxStep : 0f;

            // Set up the parking spawner (parent optional)
            parkingSpawner = new ParkingSpawner();
        }

        public override void OnEpisodeBegin()
        {
            if (parkingSpawner == null) return;

            // Re-scan parking spots (works even without a parent)
            parkingSpawner.Refresh();

            // Spawn agent and goal on random parking spots
            parkingSpawner.RandomLocation(transform, 0.5f, true);
            if (goal != null)
                parkingSpawner.RandomLocation(goal, 0.5f, true);

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

            _prevPos = transform.position;
            _stallCounter = 0;
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            // Direction to target in the agent's local frame (flattened)
            Vector3 aToB = (goal != null ? goal.position : target) - transform.position;
            aToB.y = 0f;

            Vector3 fwd = transform.forward; fwd.y = 0f; if (fwd.sqrMagnitude > 1e-6f) fwd.Normalize();
            Vector3 right = transform.right;  right.y = 0f; if (right.sqrMagnitude > 1e-6f) right.Normalize();

            float localX = Vector3.Dot(aToB, right);
            float localY = Vector3.Dot(aToB, fwd);
            sensor.AddObservation(new Vector2(localX, localY));

            // Normalized forward speed
            float normSpeed = (car != null && car.maxSpeed > 1e-6f) ? (car.forwardSpeed / car.maxSpeed) : 0f;
            sensor.AddObservation(normSpeed);
        }

        public override void OnActionReceived(ActionBuffers actionBuffers)
        {
            // Keep target synced with goal
            if (goal != null) target = goal.position;

            // Simple debug line to goal
            if (line != null)
            {
                if (debug)
                {
                    line.enabled = true;
                    line.positionCount = 2;
                    line.SetPosition(0, SurfaceSnap(transform.position));
                    line.SetPosition(1, SurfaceSnap(target));
                }
                else
                {
                    line.enabled = false;
                }
            }

            // Anti-stall shaping
            float speed;
            var rb = GetComponent<Rigidbody>();
            if (rb != null)
            {
#if UNITY_6000_0_OR_NEWER
                speed = rb.linearVelocity.magnitude;
#else
                speed = rb.velocity.magnitude;
#endif
            }
            else
            {
                speed = (transform.position - _prevPos).magnitude / Time.fixedDeltaTime;
            }

            if (speed < stallSpeedThreshold)
            {
                _stallCounter++;
                if (stillPenalty != 0f) AddReward(stillPenalty);
            }
            else
            {
                _stallCounter = 0;
            }

            if (_stallCounter >= stallTimeoutSteps)
            {
                AddReward(deathPenalty * 0.25f);
                EndEpisode();
                return;
            }

            _prevPos = transform.position;

            // Small step penalty
            if (stepPenalty != 0f) AddReward(stepPenalty);

            // Apply actions
            float inputPower = actionBuffers.ContinuousActions[0];
            float inputSteeringAngle = actionBuffers.ContinuousActions[1];
            car.Drive(inputPower, inputSteeringAngle);
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            var a = actionsOut.ContinuousActions;
            a[0] = Input.GetAxis("Vertical");
            a[1] = Input.GetAxis("Horizontal");
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

        private Vector3 SurfaceSnap(Vector3 p, float upOffset = 0.02f)
        {
            Vector3 origin = p + Vector3.up * 5f;
            if (Physics.Raycast(origin, Vector3.down, out var hit, 20f, ground, QueryTriggerInteraction.Ignore))
                return hit.point + Vector3.up * upOffset;
            return new Vector3(p.x, p.y + upOffset, p.z);
        }

#if (UNITY_EDITOR && VISUALIZE)
        private void OnDrawGizmos()
        {
            if (!debug) return;
            Gizmos.color = drawingColor;
            Gizmos.DrawSphere(target, 1);
            Gizmos.DrawLine(transform.position, target);
        }
#endif
    }
}