#define VISUALIZE

using System;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;

using UnityEngine.AI;

namespace Tommy.Scripts.Training
{
    public class CarDriver : Agent
    {
        private NavMeshPath _path;
        private Vector3[] _corners = Array.Empty<Vector3>();
        private int _cornerIdx = 0;
        [SerializeField] private int pathObsK = 3;

        private Vector3 target;

        private Vector3 _lastPathStart;
        private Vector3 _lastPathEnd;
        private int _pathRecalcFrameInterval = 10;
        private float _pathRecalcDistanceThreshold = 1.5f;
        private int _frameCounter = 0;

        [SerializeField] private CarController car;

        [Header("Rewards")]
        [SerializeField] private float goalReward = 1f;
        [SerializeField] private float deathPenalty = -1f;

        [SerializeField] private float pathLookReward = 0.003f;
        [SerializeField] private float pathMoveReward = 0.0015f;

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

        private RoadSpawner roadSpawner;

        [Header("Goal & Roads")]
        public Transform goal;
        [SerializeField] private Transform parentRoads;

        [SerializeField] private bool debug;

        [SerializeField] private bool obsIncludeHeading = true;
        [SerializeField] private bool obsIncludeMotion = true;

        // Reminder: Update the Behavior Parameters observation size in the Unity Inspector to at least 12, matching the number of observations in CollectObservations.

        //happens when you start the game
        public override void Initialize()
        {
            //Line between agent and target
            line = GetComponent<LineRenderer>();
            if (line != null) line.positionCount = 2;

            //Punishment every tick in which when the episode ends 
            // it will equal the death penalty
            if (MaxStep > 0)
                stepPenalty = deathPenalty / MaxStep;
            else
                stepPenalty = 0f;

            _path = new NavMeshPath();

            if (parentRoads != null) roadSpawner = new RoadSpawner(parentRoads);
        }

        //triggers every episode
        public override void OnEpisodeBegin()
        {
            if (parentRoads != null && roadSpawner != null)
            {
                //Random spawn on roads
                roadSpawner.RandomLocation(transform);
                if (goal != null) roadSpawner.RandomLocation(goal);
                target = goal != null ? goal.position : transform.position;

                //Rigidbody allows Gravity, forces, velocity to 
                //affect the agent
                var rb = GetComponent<Rigidbody>();
                //Stop the car from moving everytime the next episode start
                if (rb != null)
                {
#if UNITY_6000_0_OR_NEWER
                    rb.linearVelocity = Vector3.zero;
#else
                    rb.velocity = Vector3.zero;
#endif
                    rb.angularVelocity = Vector3.zero;
                }
                BuildPath();

                _lastPathStart = transform.position;
                _lastPathEnd = goal != null ? goal.position : _lastPathEnd;
                _frameCounter = 0;
                _prevPos = transform.position;
                _stallCounter = 0;
            }
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            Vector3 aToB = target - transform.position;
            //ignore height for now
            aToB.y = 0;
            //forward and right are direction towards certain axis'
            //for example if transform.forward = (1,0,0)/(x,y,z) then it is
            //facing towards the x axis
            Vector3 forward = transform.forward;
            Vector3 right = transform.right;
            //ignore height for now
            forward.y = 0;
            right.y = 0;

            //Turns them into unit vectors
            forward.Normalize();
            right.Normalize();
            //instead of being towards the axis',
            //it is now using the target as reference
            float localX = Vector3.Dot(aToB, right);
            float localY = Vector3.Dot(aToB, forward);

            Vector2 localVector = new Vector2(localX, localY);
            sensor.AddObservation(localVector);
            //add speed relative to its max speed
            sensor.AddObservation((car != null && car.maxSpeed > 1e-6f) ? (car.forwardSpeed / car.maxSpeed) : 0f);

            GetPathObservations(sensor);
        }

        //triggers every step
        public override void OnActionReceived(ActionBuffers actionBuffers)
        {
            //change to goal's position if it exists
            //if not then keep current position for target
            target = goal != null ? goal.position : target;

            AdvanceCornerIfReached();

            _frameCounter++;
            bool needRecalc = false;
            if (_corners == null || _corners.Length == 0) needRecalc = true;
            if (_cornerIdx >= (_corners?.Length ?? 0) - 1) needRecalc = true;
            if ((_frameCounter % _pathRecalcFrameInterval) == 0)
            {
                if (Vector3.Distance(transform.position, _lastPathStart) > _pathRecalcDistanceThreshold) needRecalc = true;
                if (goal != null && Vector3.Distance(goal.position, _lastPathEnd) > _pathRecalcDistanceThreshold) needRecalc = true;
            }
            if (needRecalc)
            {
                BuildPath();
                _lastPathStart = transform.position;
                if (goal != null) _lastPathEnd = goal.position;
            }

            Vector3 lookTarget = (_corners != null && _corners.Length > 0) ? _corners[Mathf.Min(_cornerIdx, _corners.Length - 1)] : (goal != null ? goal.position : target);
            Vector3 toNext = lookTarget - transform.position; toNext.y = 0f;
            if (toNext.sqrMagnitude > 1e-6f)
            {
                Vector3 dirNext = toNext.normalized;
                Vector3 fwd = transform.forward; fwd.y = 0f; if (fwd.sqrMagnitude > 1e-6f) fwd.Normalize();
                float align = Mathf.Max(0f, Vector3.Dot(fwd, dirNext));
                if (pathLookReward != 0f) AddReward(pathLookReward * align);

                Vector3 delta = transform.position - _prevPos; delta.y = 0f;
                float proj = Mathf.Max(0f, Vector3.Dot(delta, dirNext));
                if (pathMoveReward != 0f) AddReward(pathMoveReward * proj);
            }

            float speed = 0f;
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

            //lineRenderer
            if (line != null)
            {
                if (debug)
                {
                    line.enabled = true;
                    if (_corners != null && _corners.Length > 0)
                    {
                        line.positionCount = _corners.Length + 1;
                        line.SetPosition(0, SurfaceSnap(transform.position));
                        for (int i = 0; i < _corners.Length; i++)
                        {
                            line.SetPosition(i + 1, SurfaceSnap(_corners[i]));
                        }
                    }
                    else
                    {
                        line.positionCount = 2;
                        line.SetPosition(0, SurfaceSnap(transform.position));
                        line.SetPosition(1, SurfaceSnap(target));
                    }
                }
                else
                {
                    line.enabled = false;
                }
            }

            if (stepPenalty != 0f)
                AddReward(stepPenalty);

            //Actions the car is taking based on observation
            float inputPower = actionBuffers.ContinuousActions[0];
            float inputSteeringAngle = actionBuffers.ContinuousActions[1];

            //Uses car script and inputs to drive car
            car.Drive(inputPower, inputSteeringAngle);
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            var continuousActionsOut = actionsOut.ContinuousActions;
            continuousActionsOut[0] = Input.GetAxis("Vertical");
            continuousActionsOut[1] = Input.GetAxis("Horizontal");
        }

        //triggers Everytime car collides 
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

        private void BuildPath()
        {
            _cornerIdx = 0;
            if (goal == null)
            {
                _corners = Array.Empty<Vector3>();
                return;
            }
            if (NavMesh.SamplePosition(transform.position, out var startHit, 5f, NavMesh.AllAreas) &&
                NavMesh.SamplePosition(goal.position, out var goalHit, 5f, NavMesh.AllAreas) &&
                NavMesh.CalculatePath(startHit.position, goalHit.position, NavMesh.AllAreas, _path) &&
                _path.status == NavMeshPathStatus.PathComplete)
            {
                _corners = _path.corners;
            }
            else
            {
                _corners = Array.Empty<Vector3>();
            }
        }

        private void GetPathObservations(VectorSensor sensor)
        {
            int added = 0;
            for (int i = _cornerIdx; i < (_corners?.Length ?? 0) && added < pathObsK; i++, added++)
            {
                Vector3 world = _corners[i];
                Vector3 local = transform.InverseTransformPoint(new Vector3(world.x, transform.position.y, world.z));
                sensor.AddObservation(local.x);
                sensor.AddObservation(local.z);
            }
            for (; added < pathObsK; added++)
            {
                sensor.AddObservation(0f);
                sensor.AddObservation(0f);
            }

            Vector3 lookTarget = (_corners != null && _corners.Length > 0) ? _corners[Mathf.Min(_cornerIdx, _corners.Length - 1)] : (goal != null ? goal.position : target);
            Vector3 toNext = lookTarget - transform.position; toNext.y = 0f; 
            Vector3 dirNext = toNext.sqrMagnitude > 1e-6f ? toNext.normalized : Vector3.zero;
            if (obsIncludeHeading)
            {
                Vector3 fwd = transform.forward; fwd.y = 0f; if (fwd.sqrMagnitude > 1e-6f) fwd.Normalize();
                float cosH = (dirNext == Vector3.zero || fwd == Vector3.zero) ? 0f : Vector3.Dot(fwd, dirNext);
                float sinH = (dirNext == Vector3.zero || fwd == Vector3.zero) ? 0f : Vector3.Cross(fwd, dirNext).y;
                sensor.AddObservation(sinH);
                sensor.AddObservation(cosH);
            }
            if (obsIncludeMotion)
            {
                Vector3 vel = Vector3.zero;
                var rb = GetComponent<Rigidbody>();
                if (rb != null)
                {
#if UNITY_6000_0_OR_NEWER
                    vel = rb.linearVelocity;
#else
                    vel = rb.velocity;
#endif
                }
                else
                {
                    vel = transform.position - _prevPos;
                }
                vel.y = 0f;
                float speedAlong = (dirNext == Vector3.zero) ? 0f : Vector3.Dot(vel, dirNext);
                float norm = (car != null && car.maxSpeed > 1e-6f) ? Mathf.Clamp(speedAlong / car.maxSpeed, -1f, 1f) : Mathf.Clamp(speedAlong, -1f, 1f);
                sensor.AddObservation(norm);
            }

            float remain = 0f;
            for (int i = _cornerIdx; i + 1 < (_corners?.Length ?? 0); i++)
                remain += Vector3.Distance(_corners[i], _corners[i + 1]);
            sensor.AddObservation(Mathf.Clamp01(remain / 100f));
        }

        private void AdvanceCornerIfReached(float reachRadius = 2.0f)
        {
            if (_corners == null || _corners.Length == 0) return;
            while (_cornerIdx < _corners.Length - 1)
            {
                float d = Vector3.Distance(transform.position, _corners[_cornerIdx]);
                if (d <= reachRadius) _cornerIdx++;
                else break;
            }
        }

        //ignore
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