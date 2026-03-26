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
    public class CarDriverManager : Agent
    {
        [SerializeField] private CarController car;
        [SerializeField] private Transform roadParent;

        [Header("Observations")]
        [Tooltip("Number of upcoming nodes to include in observations as (angle, distance) pairs.")]
        [SerializeField] private int lookaheadNodes = 3;
        [Tooltip("Number of closest cars to observe.")]
        [SerializeField] private int maxObservedCars = 3;
        [Tooltip("Max world-unit distance for path node observations. Distances divided by this → 0-1.")]
        [SerializeField] private float nodeSenseRange = 30f;
        [Tooltip("Max world-unit distance at which other cars are sensed. Distances divided by this → 0-1.")]
        [SerializeField] private float carSenseRange = 20f;

        [Header("Rewards")]
        [SerializeField] private float goalReward = 1f;
        [SerializeField] private float deathPenalty = -1f;
        [SerializeField] private float immediateValue = 0.1f;
        [SerializeField] private float progressNode = 0.3f;
        [SerializeField] private float stillPenalty = -0.001f;
        [SerializeField] private float stallSpeedThreshold = 0.1f;
        [SerializeField] private int stallTimeoutSteps = 100;

        [Header("Path Following")]
        [SerializeField] private float checkingRate = 2f;

        [Header("Spawning")]
        [Tooltip("If true, cars can spawn on and target any lane node, not just designated parking nodes.")]
        [SerializeField] private bool useAnyNode = false;
        [Tooltip("Minimum distance from any other car required to claim a spawn node. " +
                 "0 = only block nodes a car is sitting on. Capped at carSenseRange.")]
        [SerializeField] private float spawnClearance = 0f;

        [Header("Hardware Connection")]
        [SerializeField] private PiTCPManager piManager;

        [Header("Debug")]
        [SerializeField] private bool debug;

        // --- PERMANENT HOME BASE TRACKING (useAnyNode = false) ---
        // Pool of unclaimed home nodes. We REMOVE from this list when a car claims a node
        // and ADD back when a car is destroyed. Avoids HashSet equality issues with destroyed Transforms.
        private static List<Transform> _homeNodePool = new List<Transform>();
        private static bool _homePoolReady = false;
        private Transform _myHomeNode;

        // --- PER-EPISODE SPAWN TRACKING (useAnyNode = true) ---
        // Maps envRoot → (spawnNode → carTransform that owns it this episode).
        // Using Transform→Transform avoids HashSet hash-collision issues.
        private static Dictionary<Transform, Dictionary<Transform, Transform>> _claimedSpawnNodes
            = new Dictionary<Transform, Dictionary<Transform, Transform>>();
        private Transform _mySpawnNode; // which node this car claimed this episode

        // Runs before every Play session, even when "Disable Domain Reload" is enabled in
        // Editor → Project Settings → Editor → Enter Play Mode Options. Guarantees static
        // state is always clean at startup regardless of domain reload configuration.
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void ResetStaticState()
        {
            _homeNodePool = new List<Transform>();
            _homePoolReady = false;
            _claimedSpawnNodes = new Dictionary<Transform, Dictionary<Transform, Transform>>();
        }

        // --- CONTINUITY FLAG ---
        private bool _preservePositionForNextEpisode = false;

        private Rigidbody _rb;  // cached — never call GetComponent every step
        private PathFinding _pathManager;
        private int _stallCounter = 0;
        private Vector3 _prevPos;
        private float _stepPenalty;

        private ParkingSpawner _parkingSpawner;
        private Transform _goalNode;

        private float _lastProgressFrac = 0f;
        private float _checkingTimer = 0f;
        private bool _isPathMissing = false;
        private DecisionRequester _decisionRequester;

        public override void Initialize()
        {
            _stepPenalty = (MaxStep > 0) ? deathPenalty / MaxStep : 0f;
            _rb = GetComponent<Rigidbody>();

            _parkingSpawner = new ParkingSpawner(roadParent);
            _decisionRequester = GetComponent<DecisionRequester>();
            _pathManager = GetComponent<PathFinding>();
            if (_pathManager == null)
            {
                _pathManager = gameObject.AddComponent<PathFinding>();
            }
            _pathManager?.ResetPathState();

            // Initial placement — respects the spawn mode setting
            if (useAnyNode)
            {
                // ClaimAnyNode handles positioning directly
                ClaimAnyNode();
            }
            else
            {
                AssignPermanentHomeNode();
            }

            // Register this agent car as an obstacle so other cars can detect it (with Rigidbody for velocity)
            CarObstacle.Register(roadParent, transform, _rb);
        }

        public override void OnEpisodeBegin()
        {
            _pathManager?.ResetPathState();
            _parkingSpawner.Refresh();

            if (_preservePositionForNextEpisode)
            {
                _preservePositionForNextEpisode = false;
            }
            else if (useAnyNode)
            {
                // Pick a fresh random lane node each episode, avoiding other cars
                Transform spawnNode = ClaimAnyNode();
                if (spawnNode != null)
                {
                    transform.position = spawnNode.position + spawnNode.up * 0.5f;
                    transform.rotation = GetSpawnRotation(spawnNode);
                    Physics.SyncTransforms();
                }
            }
            else if (_myHomeNode != null)
            {
                transform.position = _myHomeNode.position + _myHomeNode.up * 0.5f;
                transform.rotation = GetSpawnRotation(_myHomeNode);
                Physics.SyncTransforms();
            }

            // Goal: any lane node when useAnyNode, otherwise parking nodes as before
            if (useAnyNode)
                _goalNode = GetRandomAnyNodeGoal();
            else
                _goalNode = _parkingSpawner.GetRandomGoal(transform, 2f);

            if (_pathManager != null && _goalNode != null)
            {
                _pathManager.ForceRecomputeNow(transform, _goalNode);
                _isPathMissing = (_pathManager.GetNextNode() == null);

                if (_isPathMissing)
                {
                    _checkingTimer = checkingRate;
                    if (_decisionRequester != null) _decisionRequester.enabled = false;
                }
                else
                {
                    if (_decisionRequester != null) _decisionRequester.enabled = true;
                }
            }

            if (_rb != null)
            {
                _rb.linearVelocity = Vector3.zero;
                _rb.angularVelocity = Vector3.zero;
            }

            _prevPos = transform.position;
            _stallCounter = 0;
            _lastProgressFrac = 0f;
        }

        private void AssignPermanentHomeNode()
        {
            // Build the pool ONCE per play session (first car to call this wins).
            // All subsequent cars draw from the same pre-shuffled list.
            if (!_homePoolReady)
            {
                _homePoolReady = true;
                _homeNodePool.Clear();

                foreach (var go in GameObject.FindGameObjectsWithTag("Parking Node"))
                {
                    if (go != null) _homeNodePool.Add(go.transform);
                }

                // Fisher-Yates shuffle for random assignment order
                for (int i = _homeNodePool.Count - 1; i > 0; i--)
                {
                    int j = UnityEngine.Random.Range(0, i + 1);
                    (_homeNodePool[i], _homeNodePool[j]) = (_homeNodePool[j], _homeNodePool[i]);
                }

                if (debug) Debug.Log($"[HomePool] Built with {_homeNodePool.Count} nodes.");
            }

            if (_homeNodePool.Count > 0)
            {
                // Pop from the end — O(1) remove, guaranteed unique per car
                _myHomeNode = _homeNodePool[_homeNodePool.Count - 1];
                _homeNodePool.RemoveAt(_homeNodePool.Count - 1);
            }
            else
            {
                Debug.LogWarning($"[{name}] No unclaimed home nodes left — cars may overlap!");
                var fallback = GameObject.FindGameObjectsWithTag("Parking Node");
                if (fallback.Length > 0)
                    _myHomeNode = fallback[0].transform;
            }

            if (_myHomeNode == null) return;

            transform.position = _myHomeNode.position + _myHomeNode.up * 0.5f;
            transform.rotation = GetSpawnRotation(_myHomeNode);
            Physics.SyncTransforms();
        }

        /// <summary>
        /// Returns a flat (no pitch/roll) rotation facing away from the parking node,
        /// matching the same -node.right convention used by ParkingSpawner.RandomLocation.
        /// </summary>
        private static Quaternion GetSpawnRotation(Transform node)
        {
            Vector3 flatForward = -node.right;
            flatForward.y = 0f;
            if (flatForward.sqrMagnitude < 1e-6f)
                flatForward = node.forward; // fallback if right is pointing straight up
            Quaternion baseRot = Quaternion.LookRotation(flatForward.normalized, Vector3.up);
            return baseRot * Quaternion.Euler(180f, 0f, 180f);
        }

        /// Returns all nodes that are valid spawn/goal candidates when useAnyNode = true.
        /// - "Parking Node" — always included (can spawn in both modes)
        /// - "Node"         — included only in useAnyNode mode
        /// - "Non Parking Node" — never included in spawning
        private static GameObject[] GetAllLaneNodes()
        {
            var result = new List<GameObject>();
            foreach (var go in GameObject.FindGameObjectsWithTag("Parking Node"))
                result.Add(go);
            foreach (var go in GameObject.FindGameObjectsWithTag("Node"))
                result.Add(go);
            return result.ToArray();
        }

        /// Picks a random lane node for this episode (useAnyNode = true).
        /// Ensures no two cars claim the same node, and respects spawnClearance.
        private Transform ClaimAnyNode()
        {
            // Release the node this car held last episode
            ReleaseSpawnNode();

            var nodeGOs = GetAllLaneNodes();
            if (nodeGOs.Length == 0) return null;

            // Ensure the env dict exists
            if (!_claimedSpawnNodes.ContainsKey(roadParent))
                _claimedSpawnNodes[roadParent] = new Dictionary<Transform, Transform>();
            var claimed = _claimedSpawnNodes[roadParent];

            // Purge stale claims from cars that were destroyed
            var staleKeys = new List<Transform>();
            foreach (var kv in claimed)
                if (kv.Value == null) staleKeys.Add(kv.Key);
            foreach (var k in staleKeys) claimed.Remove(k);

            float clearance = Mathf.Clamp(spawnClearance, 0f, carSenseRange);

            // Shuffle indices for random picking
            int[] indices = new int[nodeGOs.Length];
            for (int i = 0; i < indices.Length; i++) indices[i] = i;
            for (int i = indices.Length - 1; i > 0; i--)
            {
                int j = UnityEngine.Random.Range(0, i + 1);
                (indices[i], indices[j]) = (indices[j], indices[i]);
            }

            foreach (int idx in indices)
            {
                Transform candidate = nodeGOs[idx].transform;

                // Skip if another car already claimed this exact node this episode
                if (claimed.ContainsKey(candidate)) continue;

                // Skip if this node is too close to any ALREADY CLAIMED node this episode.
                // We check claimed node positions rather than current car positions because
                // at episode start all cars are still at their old positions — checking
                // physical car positions would pass for everyone and cause overlaps.
                if (clearance > 0f)
                {
                    bool tooClose = false;
                    foreach (var kv in claimed)
                    {
                        if (kv.Key == null) continue;
                        float d = Vector3.Distance(candidate.position, kv.Key.position);
                        if (d < clearance) { tooClose = true; break; }
                    }
                    if (tooClose) continue;
                }

                // Claim it
                claimed[candidate] = transform;
                _mySpawnNode = candidate;
                return candidate;
            }

            if (debug) Debug.LogWarning($"[{name}] Could not find a clear spawn node — overlapping.");
            return nodeGOs[UnityEngine.Random.Range(0, nodeGOs.Length)].transform;
        }

        private void ReleaseSpawnNode()
        {
            if (_mySpawnNode == null) return;
            if (_claimedSpawnNodes.TryGetValue(roadParent, out var claimed))
                claimed.Remove(_mySpawnNode);
            _mySpawnNode = null;
        }

        /// Picks a random lane node as the goal that isn't the car's current position.
        private Transform GetRandomAnyNodeGoal()
        {
            var nodeGOs = GetAllLaneNodes();
            if (nodeGOs.Length == 0) return null;

            int guard = 0;
            Transform pick = null;
            do
            {
                pick = nodeGOs[UnityEngine.Random.Range(0, nodeGOs.Length)].transform;
                guard++;
            }
            while (Vector3.Distance(transform.position, pick.position) < 2f && guard < 50);

            return pick;
        }

        private void Update()
        {
            if (_isPathMissing && _pathManager != null && _goalNode != null)
            {
                // Freeze the car in place while waiting for the path
                if (_rb != null)
                {
#if UNITY_6000_0_OR_NEWER
                    _rb.linearVelocity = Vector3.zero;
#else
                    _rb.velocity = Vector3.zero;
#endif
                    _rb.angularVelocity = Vector3.zero;
                }

                _checkingTimer -= Time.deltaTime;

                if (_checkingTimer <= 0f)
                {
                    if (debug) Debug.Log("Attempting to recalculate path...");

                    _pathManager.ForceRecomputeNow(transform, _goalNode);
                    _isPathMissing = (_pathManager.GetNextNode() == null);

                    if (!_isPathMissing)
                    {
                        if (debug) Debug.Log("Path found! Agent activated.");
                        if (_decisionRequester != null) _decisionRequester.enabled = true;
                    }
                    else
                    {
                        _checkingTimer = checkingRate;
                    }
                }
            }
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            // Path Nodes Observation
            var nodes = _pathManager?.GetNextKNodes(Mathf.Max(0, lookaheadNodes));
            int validObservations = 0;

            Vector3 fwd = transform.forward;
            fwd.y = 0f;
            fwd.Normalize();

            if (nodes != null)
            {
                int count = Mathf.Min(nodes.Count, lookaheadNodes);
                for (int i = 0; i < count; i++)
                {
                    var currNode = nodes[i];
                    if (currNode == null) continue;

                    Vector3 to = currNode.position - transform.position;
                    to.y = 0f;

                    float dist = to.magnitude;
                    to.Normalize();

                    float dot = Mathf.Clamp(Vector3.Dot(fwd, to), -1f, 1f);
                    float crossY = Vector3.Cross(fwd, to).y;
                    float ang = Mathf.Atan2(crossY, dot);

                    // Normalize: angle → -1..1, distance → 0..1
                    sensor.AddObservation(ang / Mathf.PI);
                    sensor.AddObservation(Mathf.Clamp01(dist / nodeSenseRange));
                    validObservations++;
                }
            }

            // Pad missing node slots with (0=ahead, 1=at max range) — unambiguous "nothing here"
            for (int i = validObservations; i < lookaheadNodes; i++)
            {
                sensor.AddObservation(0f);
                sensor.AddObservation(1f);
            }

            // Forward speed, clamped so reversing or overshoot can't push outside -1..1
            float normSpeed = (car != null && car.maxSpeed > 1e-6f)
                ? Mathf.Clamp(car.forwardSpeed / car.maxSpeed, -1f, 1f)
                : 0f;
            sensor.AddObservation(normSpeed);

            // Current steering angle normalized to -1..1.
            // Without this the network is blind to its own wheel state and learns jerky control.
            float normSteer = (car != null && car.steeringRange > 1e-6f)
                ? Mathf.Clamp(car.steeringAngle / car.steeringRange, -1f, 1f)
                : 0f;
            sensor.AddObservation(normSteer);

            // Obstacle / other car observations — 4 values per slot:
            //   dist     0-1  (normalized by carSenseRange)
            //   bearing  -1..1  (angle from our forward to obstacle position)
            //   speed    0-1  (obstacle's velocity magnitude / our maxSpeed)
            //   heading  -1..1  (dot of our forward with obstacle's velocity direction;
            //                    +1 = moving same way, -1 = coming head-on)
            // Note: heading is based on VELOCITY, not facing — a reversing car reads negative.
            var obstacles = CarObstacle.GetAll(roadParent);

            var obstacleStats = new List<(float dist, float bearing, float speed, float heading)>();
            float myMaxSpeed = (car != null && car.maxSpeed > 1e-6f) ? car.maxSpeed : 1f;

            foreach (var entry in obstacles)
            {
                if (entry.t == null || entry.t == this.transform) continue;

                Vector3 toObs = entry.t.position - transform.position;
                toObs.y = 0f;

                float dist = toObs.magnitude;
                if (dist < 0.001f) continue;

                toObs.Normalize();
                float dot = Mathf.Clamp(Vector3.Dot(fwd, toObs), -1f, 1f);
                float crossY = Vector3.Cross(fwd, toObs).y;
                float bearing = Mathf.Atan2(crossY, dot);

                // Velocity-based speed and heading
                float obsSpeed = 0f;
                float obsHeading = 1f; // default: same direction (least threatening)
                if (entry.rb != null)
                {
#if UNITY_6000_0_OR_NEWER
                    Vector3 vel = entry.rb.linearVelocity;
#else
                    Vector3 vel = entry.rb.velocity;
#endif
                    vel.y = 0f;
                    float velMag = vel.magnitude;
                    obsSpeed = Mathf.Clamp01(velMag / myMaxSpeed);
                    if (velMag > 0.01f)
                        obsHeading = Mathf.Clamp(Vector3.Dot(fwd, vel.normalized), -1f, 1f);
                }

                obstacleStats.Add((dist, bearing, obsSpeed, obsHeading));
            }

            obstacleStats.Sort((a, b) => a.dist.CompareTo(b.dist));

            int obsCount = Mathf.Min(obstacleStats.Count, maxObservedCars);
            for (int i = 0; i < obsCount; i++)
            {
                sensor.AddObservation(Mathf.Clamp01(obstacleStats[i].dist / carSenseRange));
                sensor.AddObservation(obstacleStats[i].bearing / Mathf.PI);
                sensor.AddObservation(obstacleStats[i].speed);
                sensor.AddObservation(obstacleStats[i].heading);
            }

            // Pad empty slots: max distance, no bearing, stopped, same direction
            for (int i = obsCount; i < maxObservedCars; i++)
            {
                sensor.AddObservation(1f);  // dist = max range
                sensor.AddObservation(0f);  // bearing = straight ahead
                sensor.AddObservation(0f);  // speed = stopped
                sensor.AddObservation(1f);  // heading = same direction (least threatening)
            }
        }

        public override void OnActionReceived(ActionBuffers actionBuffers)
        {
            _pathManager?.UpdatePath(transform, _goalNode);

            var nextNode = _pathManager?.GetNextNode();
            var anchor = _pathManager?.GetAnchorNode();
            if (nextNode != null && anchor != null)
            {
                float D0 = Vector3.Distance(anchor.position, nextNode.position);

                // Guard: if anchor and next node are coincident D0 is 0 → NaN reward
                if (D0 > 1e-4f)
                {
                    float dNow = Vector3.Distance(transform.position, nextNode.position);
                    float frac = Mathf.Clamp01(1f - (dNow / D0));
                    float delta = Mathf.Max(0f, frac - _lastProgressFrac);

                    if (delta > 0f && Math.Abs(progressNode) > 1e-8f)
                    {
                        float r = progressNode * delta;
                        AddReward(r);
                        if (debug) Debug.Log($"Reward: {r:F4} - reason: progress ({frac:P0})");
                        _lastProgressFrac = frac;
                    }
                }
            }

#if UNITY_6000_0_OR_NEWER
            float speed = _rb != null ? _rb.linearVelocity.magnitude
                        : (transform.position - _prevPos).magnitude / Time.fixedDeltaTime;
#else
            float speed = _rb != null ? _rb.velocity.magnitude
                        : (transform.position - _prevPos).magnitude / Time.fixedDeltaTime;
#endif

            if (speed < stallSpeedThreshold)
            {
                _stallCounter++;
                AddReward(stillPenalty);
                if (debug) Debug.Log($"Reward: {stillPenalty} - reason: stall");
            }
            else
            {
                _stallCounter = 0;
            }

            if (_stallCounter >= stallTimeoutSteps)
            {
                AddReward(deathPenalty * 0.25f);
                if (debug) Debug.Log($"Reward: {deathPenalty * 0.25f} - reason: stall timeout");
                _pathManager?.ResetPathState();
                EndEpisode();
                return;
            }

            _prevPos = transform.position;

            if (_stepPenalty != 0f)
            {
                AddReward(_stepPenalty);
                if (debug) Debug.Log($"Reward: {_stepPenalty} - reason: step");
            }

            float inputPower = actionBuffers.ContinuousActions[0];
            float inputSteering = actionBuffers.ContinuousActions[1];
            car.Drive(inputPower, inputSteering);

            if (piManager != null)
            {
                int piSpeed = Mathf.RoundToInt(inputPower * 100f);
                int piSteer = Mathf.RoundToInt(inputSteering * 45f);
                piManager.SendMovementCommand(this.gameObject, "drive", piSpeed, piSteer);
            }
        }

        private void OnTriggerEnter(Collider other)
        {
            if (other.CompareTag("Death") || other.CompareTag("Car"))
            {
                AddReward(deathPenalty);
                if (debug) Debug.Log($"Reward: {deathPenalty} - reason: death");

                _pathManager?.ResetPathState();
                EndEpisode(); // Will automatically trigger OnEpisodeBegin and put it back home
                return;
            }

            int laneLayer = LayerMask.NameToLayer("LaneNode");
            if (other.gameObject.layer == laneLayer)
            {
                if (_goalNode != null && SameNode(other.transform, _goalNode))
                {
                    var next = _pathManager?.GetNextNode();
                    var anchor = _pathManager?.GetAnchorNode();
                    if (next != null && anchor != null)
                    {
                        float remaining = Mathf.Clamp01(1f - _lastProgressFrac);
                        float r = progressNode * remaining;
                        if (r > 0f) AddReward(r);
                    }
                    AddReward(goalReward);
                    if (debug) Debug.Log($"Reward: {goalReward} - reason: goal node");

                    // --- THE EXCEPTION ---
                    // Tell OnEpisodeBegin to skip the teleport this time
                    _preservePositionForNextEpisode = true;

                    _pathManager?.ResetPathState();
                    EndEpisode();
                    return;
                }

                if (_pathManager != null)
                {
                    if (_pathManager.IsEpisodeSpawnNode(other.transform) || SameNode(other.transform, _pathManager.GetAnchorNode()))
                    {
                        return;
                    }

                    var expectedNext = _pathManager.GetNextNode();
                    if (expectedNext != null && SameNode(other.transform, expectedNext))
                    {
                        float remain = Mathf.Clamp01(1f - _lastProgressFrac);
                        float rProg = progressNode * remain;
                        if (rProg > 0f) AddReward(rProg);

                        AddReward(immediateValue);
                        if (debug) Debug.Log($"Reward: {immediateValue} - reason: correct node");

                        _pathManager.AdvanceIfNodeReached(other.transform);
                        _lastProgressFrac = 0f;
                    }
                    else
                    {
                        AddReward(-immediateValue);
                        if (debug) Debug.Log($"Reward: {-immediateValue} - reason: wrong node");
                    }
                }
            }
        }

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
            if (Mathf.Abs(accel) < dead) accel = 0f;
            if (Mathf.Abs(steer) < dead) steer = 0f;

            ca[0] = Mathf.Clamp(accel, -1f, 1f);
            ca[1] = Mathf.Clamp(steer, -1f, 1f);
        }

        private static bool SameNode(Transform a, Transform b)
        {
            if (a == null || b == null) return false;
            return a == b || a.IsChildOf(b) || b.IsChildOf(a);
        }


        private void OnDestroy()
        {
            CarObstacle.Deregister(roadParent, transform);
            ReleaseSpawnNode();

            // Return home node to pool for useAnyNode = false mode
            if (_myHomeNode != null && !_homeNodePool.Contains(_myHomeNode))
                _homeNodePool.Add(_myHomeNode);
        }
    }
}