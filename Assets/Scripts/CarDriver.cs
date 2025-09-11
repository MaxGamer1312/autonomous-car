#define VISUALIZE

using System;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;

namespace Tommy.Scripts.Training
{
    public class CarDriver : Agent
    {
        public float groundTimer;
        private float groundPenalty = 0;
        public float timer;
        private float timerPenalty = 0;
        private Vector3 target;

        [SerializeField] private CarController car;

        private Color drawingColor;
        public LayerMask ground;
        private Vector3 startPosition;
        private Quaternion startRotation;

        private LineRenderer line;

        private int retry = 0;

        [Header("Goal & Roads")]
        public Transform goal;
        [SerializeField] private Transform parentRoads;   
        private Vector3[][] possibleLocations;

        public override void Initialize()
        {
            line = GetComponent<LineRenderer>();
            if (line != null) line.positionCount = 2;

            startPosition = transform.position;
            startRotation = transform.rotation;

            BuildPossibleLocations();
        }

        public override void OnEpisodeBegin()
        {
            if (parentRoads != null && possibleLocations != null)
            {
                RandomLocation(transform);
                if (goal != null) RandomLocation(goal);

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
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            Vector3 aToB = target - transform.position;
            aToB.y = 0;

            Vector3 forward = transform.forward;
            Vector3 right = transform.right;
            forward.y = 0; right.y = 0;
            forward.Normalize(); right.Normalize();

            float localX = Vector3.Dot(aToB, right); 
            float localY = Vector3.Dot(aToB, forward);

            Vector2 localVector = new Vector2(localX, localY);
            sensor.AddObservation(localVector);
            sensor.AddObservation(car.forwardSpeed / car.maxSpeed);
        }

        private float maxDist = 0;
        private float maxHeading = 0;

        public override void OnActionReceived(ActionBuffers actionBuffers)
        {
            target = goal != null ? goal.position : target;

            if (line != null)
            {
                line.SetPosition(0, transform.position);
                line.SetPosition(1, target);
            }

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

#if (UNITY_EDITOR && VISUALIZE)
        private void OnDrawGizmos()
        {
            Gizmos.color = drawingColor;
            Gizmos.DrawSphere(target, 1);
            Gizmos.DrawLine(transform.position, target);
        }
#endif

        private void OnTriggerEnter(Collider other)
        {
            if (other.CompareTag("Death"))
            {
                AddReward(-1f);
                EndEpisode();
            }
            if (other.CompareTag("Goal"))
            {
                Debug.Log("GOAL");
                AddReward(1f);
                EndEpisode();
            }
        }


        private void BuildPossibleLocations()
        {
            if (parentRoads == null) return;

            int n = parentRoads.childCount;
            possibleLocations = new Vector3[n][];

            Transform common = transform.parent == null ? transform : transform.parent;

            for (int i = 0; i < n; i++)
            {
                Transform child = parentRoads.GetChild(i);
                BoxCollider box = child.GetComponent<BoxCollider>();
                if (box == null)
                {
                    possibleLocations[i] = null;
                    continue;
                }

                Vector3[] localTopCorners = new Vector3[4]
                {
                    new Vector3(-0.5f,  0.5f, -0.5f),
                    new Vector3(-0.5f,  0.5f,  0.5f),
                    new Vector3( 0.5f,  0.5f, -0.5f),
                    new Vector3( 0.5f,  0.5f,  0.5f),
                };

                Vector3[] worldTop = new Vector3[4];
                for (int j = 0; j < 4; j++)
                {
                    Vector3 scaled = Vector3.Scale(localTopCorners[j], box.size);
                    worldTop[j] = box.transform.TransformPoint(scaled + box.center);
                    worldTop[j].y += 1f; 
                }

                Vector3[] commonLocalTop = new Vector3[4];
                for (int j = 0; j < 4; j++)
                    commonLocalTop[j] = common.InverseTransformPoint(worldTop[j]);

                float minX = Mathf.Min(Mathf.Min(commonLocalTop[0].x, commonLocalTop[1].x), Mathf.Min(commonLocalTop[2].x, commonLocalTop[3].x));
                float maxX = Mathf.Max(Mathf.Max(commonLocalTop[0].x, commonLocalTop[1].x), Mathf.Max(commonLocalTop[2].x, commonLocalTop[3].x));
                float minZ = Mathf.Min(Mathf.Min(commonLocalTop[0].z, commonLocalTop[1].z), Mathf.Min(commonLocalTop[2].z, commonLocalTop[3].z));
                float maxZ = Mathf.Max(Mathf.Max(commonLocalTop[0].z, commonLocalTop[1].z), Mathf.Max(commonLocalTop[2].z, commonLocalTop[3].z));
                float topY = Mathf.Max(Mathf.Max(commonLocalTop[0].y, commonLocalTop[1].y), Mathf.Max(commonLocalTop[2].y, commonLocalTop[3].y));

                float spanX = maxX - minX;
                float spanZ = maxZ - minZ;

                Vector3 pA, pB;
                if (spanX >= spanZ)
                {
                    float midZ = 0.5f * (minZ + maxZ);
                    pA = new Vector3(minX, topY, midZ);
                    pB = new Vector3(maxX, topY, midZ);
                }
                else
                {
                    float midX = 0.5f * (minX + maxX);
                    pA = new Vector3(midX, topY, minZ);
                    pB = new Vector3(midX, topY, maxZ);
                }

                possibleLocations[i] = new Vector3[2] { pA, pB };
            }
        }

        private void RandomLocation(Transform obj)
        {
            if (parentRoads == null || possibleLocations == null || parentRoads.childCount == 0) return;

            int index = Random.Range(0, parentRoads.childCount);
            Vector3[] linePicked = possibleLocations[index];
            if (linePicked == null) return;

            float t = Random.Range(0.1f, 0.9f);
            Vector3 randomPoint = Vector3.Lerp(linePicked[0], linePicked[1], t);

            obj.localPosition = randomPoint;

            float fixedX = transform.eulerAngles.x;
            float fixedZ = transform.eulerAngles.z;
            float randomY = Random.Range(0f, 360f);
            obj.rotation = Quaternion.Euler(fixedX, randomY, fixedZ);
        }
    }
}