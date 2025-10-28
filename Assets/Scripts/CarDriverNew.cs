#define VISUALIZE

using System;
using System.Collections;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

namespace Tommy.Scripts.Training
{
    public class CarDriverNew : Agent
    {
        private Vector3 target;
        private PathFinding pathFinder;

        [SerializeField] private CarController car;

        [Header("Observations")]
        [Tooltip("Number of upcoming nodes to include in observations as (angle, distance) pairs.")]
        [SerializeField] private int lookaheadNodes = 3;

        [Header("Rewards")]
        [SerializeField] private float goalReward = 1f;
        [SerializeField] private float deathPenalty = -1f;
        [SerializeField] private float immediateValue = 0.1f;   // reward when hitting the correct next node
        [SerializeField] private float progressNode = 0.3f;     // total shaping reward per segment

        [Header("Anti-Stall")]
        [SerializeField] private float stillPenalty = -0.001f;
        [SerializeField] private float stallSpeedThreshold = 0.1f;
        [SerializeField] private int stallTimeoutSteps = 100;
        private int _stallCounter = 0;

        private Vector3 _prevPos;
        private float stepPenalty;

        public LayerMask ground;
        private LineRenderer line;
        private ParkingSpawner parkingSpawner;

        [Header("Goal & Parking")]
        private Transform goal; // not moved; goal node is chosen by tag
        [SerializeField] private Color goalNodeColor = Color.green;
        private Transform _goalNode;
        private Renderer _goalNodeRenderer;
        private Color _goalNodeOriginalColor;

        [SerializeField] private bool debug;
        [SerializeField] private bool debugRewards;

        // Progressive shaping state (0..1). Reset each time a segment completes.
        private float _lastProgressFrac = 0f;

        public override void Initialize()
        {
            line = GetComponent<LineRenderer>();
            if (line != null) line.positionCount = 2;

            stepPenalty = (MaxStep > 0) ? deathPenalty / MaxStep : 0f;
            parkingSpawner = new ParkingSpawner();

            pathFinder = GetComponent<PathFinding>();
            if (pathFinder == null)
                pathFinder = gameObject.AddComponent<PathFinding>();
        }

        public override void OnEpisodeBegin()
        {
            if (parkingSpawner == null) return;

            pathFinder?.ResetPathState();

            parkingSpawner.Refresh();
            parkingSpawner.RandomLocation(transform, 0.5f, true);

            SelectRandomGoalNode(transform);
            target = _goalNode != null ? _goalNode.position : transform.position;

            if (pathFinder != null && _goalNode != null)
                pathFinder.ForceRecomputeNow(transform, _goalNode);

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
            _lastProgressFrac = 0f;
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            // (angle, distance) for the next 'lookaheadNodes' path nodes
            var nodes = pathFinder?.GetNextKNodes(Mathf.Max(0, lookaheadNodes));
            int emitted = 0;

            if (nodes != null)
            {
                // flatten car forward
                Vector3 fwd = transform.forward; fwd.y = 0f;
                if (fwd.sqrMagnitude > 1e-6f) fwd.Normalize();

                foreach (var node in nodes)
                {
                    if (node == null) continue;
                    Vector3 to = node.position - transform.position;
                    to.y = 0f; // only going on the horizontal plane
                    float dist = to.magnitude;

                    float ang = 0f;
                    if (dist > 1e-6f && fwd.sqrMagnitude > 1e-6f)
                    {
                        Vector3 dir = to / dist;
                        float dot = Mathf.Clamp(Vector3.Dot(fwd, dir), -1f, 1f);
                        float crossY = Vector3.Cross(fwd, dir).y;
                        ang = Mathf.Atan2(crossY, dot); // signed angle in radians (-pi..pi), left=+, right=-
                    }

                    sensor.AddObservation(ang);
                    sensor.AddObservation(dist);
                    // dist and angle are based on the car's position relative to the node

                    if (node.CompareTag("Stop Node"))
                    {
                        float speedThreshold = 0.5f;

                        // Small penalty per frame if going too fast
                        if (car.forwardSpeed > speedThreshold)
                        {
                            AddReward(-0.01f); // stepwise penalty
                            stoppedTime = 0f;  // reset stop timer
                        }
                        else
                        {
                            // Reward for staying stopped at the node
                            stoppedTime += Time.fixedDeltaTime;
                            AddReward(0.01f * Time.fixedDeltaTime); 
                        }

                        // Bonus for fully stopping for N seconds
                        if (stoppedTime >= 2f) // 2 seconds
                        {
                            AddReward(0.2f); // give a decent reward for stopping correctly
                            stoppedTime = 0f; // reset for next stop node
                        }
                    }
                    emitted++;
                    if (emitted >= lookaheadNodes) break;
                }
            }

            // pad observations with zeros if fewer nodes
            for (; emitted < lookaheadNodes; emitted++)
            {
                sensor.AddObservation(0f); // angle
                sensor.AddObservation(0f); // distance
            }

            // Keep a simple motion cue (optional, not about the goal)
            float normSpeed = (car != null && car.maxSpeed > 1e-6f)
                ? (car.forwardSpeed / car.maxSpeed)
                : 0f;
            sensor.AddObservation(normSpeed);
        }
    
        public override void OnActionReceived(ActionBuffers actionBuffers)
        {
            if (_goalNode != null) target = _goalNode.position;
            pathFinder?.UpdatePath(transform, _goalNode);

            // Progressive shaping toward the next node (linear, only when making positive progress)
            var nextNode = pathFinder?.GetNextNode();
            var anchor = pathFinder?.GetAnchorNode();
            if (nextNode != null && anchor != null)
            {
                float D0 = Vector3.Distance(anchor.position, nextNode.position);
                if (D0 > 1e-4f)
                {
                    float dNow = Vector3.Distance(transform.position, nextNode.position);
                    float frac = Mathf.Clamp01(1f - (dNow / D0)); // 0 at anchor, 1 at next node
                    float delta = Mathf.Max(0f, frac - _lastProgressFrac);

                    if (delta > 0f && Math.Abs(progressNode) > 1e-8f)
                    {
                        float r = progressNode * delta;
                        AddReward(r);
                        if (debugRewards) Debug.Log($"Reward: {r:F4} - reason: progress ({frac:P0})");
                        _lastProgressFrac = frac;
                    }
                }
            }

            // Stall shaping
            float speed;
            var rb = GetComponent<Rigidbody>();
#if UNITY_6000_0_OR_NEWER
            speed = rb != null ? rb.linearVelocity.magnitude : (transform.position - _prevPos).magnitude / Time.fixedDeltaTime;
#else
            speed = rb != null ? rb.velocity.magnitude : (transform.position - _prevPos).magnitude / Time.fixedDeltaTime;
#endif

            if (speed < stallSpeedThreshold)
            {
                _stallCounter++;
                if (stillPenalty != 0f)
                {
                    AddReward(stillPenalty);
                    if (debugRewards) Debug.Log($"Reward: {stillPenalty} - reason: stall");
                }
            }
            else _stallCounter = 0;

            if (_stallCounter >= stallTimeoutSteps)
            {
                AddReward(deathPenalty * 0.25f);
                if (debugRewards) Debug.Log($"Reward: {deathPenalty * 0.25f} - reason: stall timeout");
                pathFinder?.ResetPathState();
                EndEpisode();
                return;
            }

            _prevPos = transform.position;

            if (stepPenalty != 0f)
            {
                AddReward(stepPenalty);
                if (debugRewards) Debug.Log($"Reward: {stepPenalty} - reason: step");
            }

            // Apply actions
            float inputPower = actionBuffers.ContinuousActions[0];
            float inputSteering = actionBuffers.ContinuousActions[1];
            car.Drive(inputPower, inputSteering);
        }

        private void OnTriggerEnter(Collider other)
        {
            if (other.CompareTag("Death"))
            {
                AddReward(deathPenalty);
                if (debugRewards) Debug.Log($"Reward: {deathPenalty} - reason: death");
                pathFinder?.ResetPathState();
                EndEpisode();
                return;
            }

            int laneLayer = LayerMask.NameToLayer("LaneNode");
            if (other.gameObject.layer == laneLayer)
            {
                if (_goalNode != null && SameNode(other.transform, _goalNode))
                {
                    // pay remaining shaping on the last leg, then goal bonus
                    var next = pathFinder?.GetNextNode();
                    var anchor = pathFinder?.GetAnchorNode();
                    if (next != null && anchor != null)
                    {
                        float remaining = Mathf.Clamp01(1f - _lastProgressFrac);
                        float r = progressNode * remaining;
                        if (r > 0f) { AddReward(r); if (debugRewards) Debug.Log($"Reward: {r:F4} - reason: progress remainder at goal"); }
                    }

                    AddReward(goalReward);
                    if (debugRewards) Debug.Log($"Reward: {goalReward} - reason: goal node");
                    pathFinder?.ResetPathState();
                    EndEpisode();
                    return;
                }

                if (pathFinder != null)
                {
                    // Ignore only the episode's spawn node (neutral)
                    if (pathFinder.IsEpisodeSpawnNode(other.transform))
                        return;

                    var expectedNext = pathFinder.GetNextNode();
                    if (expectedNext != null && SameNode(other.transform, expectedNext))
                    {
                        // complete shaping for this leg
                        float remain = Mathf.Clamp01(1f - _lastProgressFrac);
                        float rProg = progressNode * remain;
                        if (rProg > 0f) { AddReward(rProg); if (debugRewards) Debug.Log($"Reward: {rProg:F4} - reason: progress complete"); }

                        // immediate hit
                        AddReward(immediateValue);
                        if (debugRewards) Debug.Log($"Reward: {immediateValue} - reason: correct node");

                        // advance path & reset shaping for next segment
                        pathFinder.AdvanceIfNodeReached(other.transform);
                        _lastProgressFrac = 0f;
                    }
                    else
                    {
                        AddReward(-immediateValue);
                        if (debugRewards) Debug.Log($"Reward: {-immediateValue} - reason: wrong node");
                    }
                }
            }
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            var ca = actionsOut.ContinuousActions;

            float accelKey =
                (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow)) ?  1f :
                (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow)) ? -1f : 0f;

            float steerKey =
                (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow))  ? -1f :
                (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow)) ?  1f : 0f;

            float accel = (Mathf.Abs(accelKey) > 0f) ? accelKey : Input.GetAxis("Vertical");
            float steer = (Mathf.Abs(steerKey) > 0f) ? steerKey : Input.GetAxis("Horizontal");

            const float dead = 0.05f;
            if (Mathf.Abs(accel) < dead) accel = 0f;
            if (Mathf.Abs(steer) < dead) steer = 0f;

            ca[0] = Mathf.Clamp(accel, -1f, 1f);
            ca[1] = Mathf.Clamp(steer, -1f, 1f);
        }

        private void SelectRandomGoalNode(Transform carTransform)
        {
            if (_goalNodeRenderer != null)
            {
                _goalNodeRenderer.material.color = _goalNodeOriginalColor;
                _goalNodeRenderer = null;
            }

            var nodes = GameObject.FindGameObjectsWithTag("Parking Node");
            if (nodes == null || nodes.Length == 0) return;

            Transform chosenNode = null;
            float dist;
            int guard = 0;

            do
            {
                chosenNode = nodes[UnityEngine.Random.Range(0, nodes.Length)].transform;
                dist = Vector3.Distance(carTransform.position, chosenNode.position);
                guard++;
            }
            while (dist < 2f && nodes.Length > 1 && guard < 50);

            _goalNode = chosenNode;
            if (_goalNode != null)
            {
                _goalNodeRenderer = _goalNode.GetComponentInChildren<Renderer>();
                if (_goalNodeRenderer != null)
                {
                    _goalNodeOriginalColor = _goalNodeRenderer.material.color;
                    _goalNodeRenderer.material.color = goalNodeColor;
                }
            }
        }

        private static bool SameNode(Transform a, Transform b)
        {
            if (a == null || b == null) return false;
            return a == b || a.IsChildOf(b) || b.IsChildOf(a);
        }
    }
}