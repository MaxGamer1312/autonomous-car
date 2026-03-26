#define VISUALIZE

using System;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;


namespace Tommy.Scripts.Training
{
    // Manages the observation, reward, and actions the Agent makes.
    // The Agent's goal is to follow the shortest path indicated by green nodes
    public class test : Agent
    {
        [SerializeField] private CarController car;
        [SerializeField] private Transform roadParent;

        [Header("Observations")]
        [Tooltip("Number of upcoming nodes to include in observations as (angle, distance) pairs.")]
        // Amount of green nodes the model can see in front of it
        [SerializeField] private int lookaheadNodes = 3;

        [Header("Rewards")]
        [SerializeField] private float goalReward = 1f;
        [SerializeField] private float deathPenalty = -1f;
        // reward when hitting the correct next node
        [SerializeField] private float immediateValue = 0.1f;
        // total shaping reward per segment
        [SerializeField] private float progressNode = 0.3f;     
        [SerializeField] private float stillPenalty = -0.001f;
        [SerializeField] private float stallSpeedThreshold = 0.1f;
        [SerializeField] private int stallTimeoutSteps = 100;

        [Header("Path Following")]
        [SerializeField] private bool bidirectional = false;


        [Header("Debug")]
        [SerializeField] private bool debug;

        

        private PathFinding _pathManager;
        private int _stallCounter = 0;
        private Vector3 _prevPos;
        // Small penalty to give the Agent at each step
        // to encourage Agent to finish as fast as possible
        private float _stepPenalty;

        private ParkingSpawner _parkingSpawner;
        
        
        private Transform _goalNode;

        // Progressive shaping state (0..1). Reset each time a segment completes.
        private float _lastProgressFrac = 0f;

        public override void Initialize()
        {
            _stepPenalty = (MaxStep > 0) ? deathPenalty / MaxStep : 0f;

            _parkingSpawner = new ParkingSpawner(roadParent);

            _pathManager = GetComponent<PathFinding>();
            if (_pathManager == null){
                _pathManager = gameObject.AddComponent<PathFinding>();
            }
        }

        public override void OnEpisodeBegin()
        {
            _pathManager?.ResetPathState();

            _parkingSpawner.Refresh();
            _parkingSpawner.RandomLocation(transform, 0.5f, true);

            _goalNode = _parkingSpawner.GetRandomGoal(transform, 2f);

            if (_pathManager != null && _goalNode != null)
                _pathManager.ForceRecomputeNow(transform, _goalNode);

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

        // Called when Agent needs to make a decision (Decision Period)
        public override void CollectObservations(VectorSensor sensor)
        {
            // (angle, distance) for the next 'lookaheadNodes' path nodes
            var nodes = _pathManager?.GetNextKNodes(Mathf.Max(0, lookaheadNodes));

            int validObservations = 0;

            // flatten car forward
            Vector3 fwd = transform.forward;
            fwd.y = 0f;
            fwd.Normalize();

            // Calculates the angle and distance from the next n green nodes
            // where n is min(lookaheadNodes, total green nodes)
            if (nodes != null)
            {
                int count = Mathf.Min(nodes.Count, lookaheadNodes);

                for(int i = 0; i < count; i++)
                {
                    var currNode = nodes[i];
                    if (currNode == null) {
                        continue;
                    }
                    Vector3 to = currNode.position - transform.position;
                    to.y = 0f;

                    float dist = to.magnitude;
                    to.Normalize();

                    float dot = Mathf.Clamp(Vector3.Dot(fwd, to), -1f, 1f);
                    float crossY = Vector3.Cross(fwd, to).y;
                    float ang = Mathf.Atan2(crossY, dot); // signed angle in radians (-pi..pi), left=+, right=-
             

                    sensor.AddObservation(ang);
                    sensor.AddObservation(dist);
                    validObservations++;
                }
            }

            // pad with zeros if fewer nodes
            for (int i = validObservations; i < lookaheadNodes; i++)
            {
                sensor.AddObservation(0f); // angle
                sensor.AddObservation(0f); // distance
            }

            // Keep a simple motion cue (optional, not about the goal)
            float normSpeed = (car != null && car.maxSpeed > 1e-6f) ? (car.forwardSpeed / car.maxSpeed) : 0f;
            sensor.AddObservation(normSpeed);
        }

        // Called when Agent needs to make an output (Decision Period)
        public override void OnActionReceived(ActionBuffers actionBuffers)
        {
            _pathManager?.UpdatePath(transform, _goalNode);

            // Progressive shaping toward the next node (linear, only when making positive progress)
            var nextNode = _pathManager?.GetNextNode();
            var anchor = _pathManager?.GetAnchorNode();
            if (nextNode != null && anchor != null)
            {
                float D0 = Vector3.Distance(anchor.position, nextNode.position);

                float dNow = Vector3.Distance(transform.position, nextNode.position);
                float frac = Mathf.Clamp01(1f - (dNow / D0)); // 0 at anchor, 1 at next node
                float delta = Mathf.Max(0f, frac - _lastProgressFrac);

                if (delta > 0f && Math.Abs(progressNode) > 1e-8f)
                {
                    float r = progressNode * delta;
                    AddReward(r);
                    if (debug){
                        Debug.Log($"Reward: {r:F4} - reason: progress ({frac:P0})");
                    }
                    _lastProgressFrac = frac;
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
            // Agent gets penalty for not meeting speed threshold
            if (speed < stallSpeedThreshold)
            {
                _stallCounter++;

                AddReward(stillPenalty);
                if (debug) {
                    Debug.Log($"Reward: {stillPenalty} - reason: stall");
                }
            }
            else{
                _stallCounter = 0;
            }

            // Agent stalling for too long
            // which ends episode and gains massive penalty
            if (_stallCounter >= stallTimeoutSteps)
            {
                AddReward(deathPenalty * 0.25f);
                if (debug) {
                    Debug.Log($"Reward: {deathPenalty * 0.25f} - reason: stall timeout");
                }
                _pathManager?.ResetPathState();
                EndEpisode();
                return;
            }

            _prevPos = transform.position;

            // Penalty for not finishing
            if (_stepPenalty != 0f)
            {
                AddReward(_stepPenalty);
                if (debug) {
                    Debug.Log($"Reward: {_stepPenalty} - reason: step");
                }
            }

            // Apply actions
            float inputPower = actionBuffers.ContinuousActions[0];
            float inputSteering = actionBuffers.ContinuousActions[1];
            car.Drive(inputPower, inputSteering);
        }

        // Called when the Agent touches an object
        private void OnTriggerEnter(Collider other)
        {
            // Gets a penalty if it touches an object with a tag named "Death"
            if (other.CompareTag("Death") || other.CompareTag("Car"))
            {
                AddReward(deathPenalty);
                if (debug) {
                    Debug.Log($"Reward: {deathPenalty} - reason: death");
                }
                _pathManager?.ResetPathState();
                EndEpisode();
                return;
            }

            int laneLayer = LayerMask.NameToLayer("LaneNode");
            if (other.gameObject.layer == laneLayer)
            {
                // If the Agent hit the goal node
                if (_goalNode != null && SameNode(other.transform, _goalNode))
                {

                    // pay remaining shaping on the last node, then goal bonus


             
                    var next = _pathManager?.GetNextNode();
                    var anchor = _pathManager?.GetAnchorNode();
                    if (next != null && anchor != null)
                    {
                        float remaining = Mathf.Clamp01(1f - _lastProgressFrac);
                        float r = progressNode * remaining;
                        if (r > 0f) {
                            AddReward(r);
                            if (debug) {
                                Debug.Log($"Reward: {r:F4} - reason: progress remainder at goal");
                            }
                        }
                    }

                    AddReward(goalReward);
                    if (debug) {
                        Debug.Log($"Reward: {goalReward} - reason: goal node");
                    }
                    _pathManager?.ResetPathState();
                    EndEpisode();
                    return;
                }

                if (_pathManager != null)
                {
                    // Ignore the episode's spawn node (neutral)
                    // and anchor node
                    if (_pathManager.IsEpisodeSpawnNode(other.transform) || SameNode(other.transform, _pathManager.GetAnchorNode()))
                    {
                        return;
                    }

                    // If the Agent hit the next green node
                    var expectedNext = _pathManager.GetNextNode();
                    if (expectedNext != null && SameNode(other.transform, expectedNext))
                    {
                        // complete shaping for this leg
                        float remain = Mathf.Clamp01(1f - _lastProgressFrac);
                        float rProg = progressNode * remain;
                        if (rProg > 0f) { 
                            AddReward(rProg);
                            if (debug) {
                                Debug.Log($"Reward: {rProg:F4} - reason: progress complete");
                            }
                        }

                        // immediate hit
                        AddReward(immediateValue);
                        if (debug) {
                            Debug.Log($"Reward: {immediateValue} - reason: correct node");
                        }

                        // advance path & reset shaping for next segment
                        _pathManager.AdvanceIfNodeReached(other.transform);
                        _lastProgressFrac = 0f;
                    }
                    else
                    {
                        // Penalty for hitting the wrong node
                        AddReward(-immediateValue);
                        if (debug) {
                            Debug.Log($"Reward: {-immediateValue} - reason: wrong node");
                        }
                    }
                }
            }
        }

        // For manual control
        public override void Heuristic(in ActionBuffers actionsOut)
        {
            var ca = actionsOut.ContinuousActions;

            float accelKey =
                (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow)) ? 1f :
                (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow)) ? -1f : 0f;

            float steerKey =
                (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow)) ? -1f :
                (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow)) ? 1f : 0f;

            float accel = (Mathf.Abs(accelKey) > 0f) ? accelKey : Input.GetAxis("Vertical");
            float steer = (Mathf.Abs(steerKey) > 0f) ? steerKey : Input.GetAxis("Horizontal");

            const float dead = 0.05f;
            if (Mathf.Abs(accel) < dead) {
                accel = 0f;
            }
            if (Mathf.Abs(steer) < dead) {
                steer = 0f;
            }
            ca[0] = Mathf.Clamp(accel, -1f, 1f);
            ca[1] = Mathf.Clamp(steer, -1f, 1f);
        }
    

        private static bool SameNode(Transform a, Transform b)
        {
            if (a == null || b == null) return false;
            return a == b || a.IsChildOf(b) || b.IsChildOf(a);
        }
    }
}