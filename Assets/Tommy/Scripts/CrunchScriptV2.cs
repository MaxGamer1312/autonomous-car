#define VISUALIZE

using System;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;

namespace Tommy.Scripts.Training
{
    public class CrunchScriptV2 : Agent
    {

        public float groundTimer;
        private float groundPenalty = 0;
        public float timer;
        private float timerPenalty = 0;
        private Vector3 target;
        
        [SerializeField]
        private CarController car;
        
        private Color drawingColor;
        public LayerMask ground;
        private Vector3 startPosition;
        private Quaternion startRotation;

        private LineRenderer line;

        private int retry = 0;

        private float averageSpeed;
        
        
        public override void Initialize()
        {
            line = GetComponent<LineRenderer>();
            line.positionCount = 2;
            startPosition = transform.position;
            startRotation = transform.rotation;
        }

        public override void OnEpisodeBegin()
        {
            
            drawingColor = new Color(Random.value, Random.value, Random.value);
            do
            {
                FullGridWorldHard();
            } while (Vector3.Distance(transform.position, target) < 2);

            startPosition = transform.position;
            startRotation = transform.rotation;
            line.SetPosition(0, transform.position);
            line.SetPosition(1, target);
            line.startColor = drawingColor;
            line.startWidth = .3f;
            line.endWidth = .3f;
            line.endColor = drawingColor;
            

            timerPenalty = 0;
            groundPenalty = 0;
            car.Restart();
            
            
            
            void FullGridWorld()
            {
                // variables for full grid world
                float[] xPositions = { -9, -10, 11, 9, 10, 11 };
                float[] zPositions = { -9, -10, 11, 9, 10, 11 };
                float x = xPositions[Random.Range(0, xPositions.Length)];
                float z = zPositions[Random.Range(0, zPositions.Length)];
                transform.localPosition = new Vector3(x, 1.5f, z);
                transform.localRotation = Quaternion.Euler(0, Random.Range(-180f, 180f), 0);
                // target
                x = xPositions[Random.Range(0, xPositions.Length)];
                z = zPositions[Random.Range(0, zPositions.Length)];
                target = new Vector3(x, 1.5f, z);
            }

            void CloseObjects()
            {
                timerPenalty = 0;
                groundPenalty = 0;
                car.Restart();
                transform.localPosition = startPosition;
                transform.rotation = Quaternion.Euler(0, Random.Range(-180f, 180f), 0);;
                var v2 = Random.insideUnitCircle;
                target = startPosition + new Vector3(v2.x, 0, v2.y) * 2f;
                //target = startPosition + new Vector3(-1, 0, Random.Range(0, 2)* 2 - 1) * 2f;
                line.SetPosition(0, transform.position);
                line.SetPosition(1, target);
                line.startColor = drawingColor;
                line.startWidth = .3f;
                line.endWidth = .3f;
                line.endColor = drawingColor;
            }
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            Vector3 aToB = target - transform.position;

            aToB.y = 0;

            Vector3 forward = transform.forward;
            Vector3 right = transform.right;

            forward.y = 0;
            right.y = 0;
            forward.Normalize();
            right.Normalize();

            float localX = Vector3.Dot(aToB, right);    // left/right
            float localY = Vector3.Dot(aToB, forward);  // forward/backward

            Vector2 localVector = new Vector2(localX, localY);
            sensor.AddObservation(localVector);
            sensor.AddObservation(car.forwardSpeed / car.maxSpeed);
            
            
            
            Vector3 dirTo = target - transform.position;
            dirTo.y = 0;
            Vector2 dirTo2D = new Vector2(dirTo.x, dirTo.z).normalized;

            Vector3 forw = transform.forward;
            Vector2 forward2D = new Vector2(forw.x, forw.z).normalized;
            float signedAngle = Vector2.SignedAngle(forward2D, dirTo2D);
            sensor.AddObservation(signedAngle/180f);
        }

        private float maxDist = 0;
        private float maxHeading = 0;
        private Vector3 prevPos;
        public override void OnActionReceived(ActionBuffers actionBuffers)
        {
            if (Physics.Raycast(transform.position, Vector3.down, 1, ground))
            {
                groundPenalty += Time.deltaTime;
                if (groundPenalty >= groundTimer)
                {
                    AddReward(-1f);
                    EndEpisode();
                }
            }
            
            timerPenalty += Time.deltaTime;
            const float alpha = .001f; // for exponential moving average EMA
            float speed = (Vector3.Distance(prevPos, target) - Vector3.Distance(transform.position, target)) / Time.deltaTime;
            averageSpeed = speed * alpha + (1 - alpha) * averageSpeed;
            Debug.Log(averageSpeed);
            prevPos = transform.position;
            if (timerPenalty >= timer)
            {
                float progress = Vector3.Distance(transform.position, target) / Vector3.Distance(startPosition, target);
                AddReward(Mathf.Lerp(1, 0, progress));
                // less is better
                AddReward(averageSpeed);
                EndEpisode();
            }
            
            if (Vector3.SqrMagnitude(target - transform.position) < 2)
            {
                AddReward(1f);
                OnEpisodeBegin();
            }
            
            float inputPower = actionBuffers.ContinuousActions[0];
            float inputSteeringAngle = actionBuffers.ContinuousActions[1];
            car.Drive(inputPower, inputSteeringAngle);
            line.SetPosition(0, transform.position);
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
        }
        void FullGridWorldHard()
        {
            // variables for full grid world
            float[] verticalRoads = { -10, 10 };
            float[] horizontalRoads = { -10, 10 };

            float x, z;
            if (Random.value < .5f)
            {
                //choose verticalRoads
                x = Random.Range(-24f, 24f);
                z = horizontalRoads[Random.Range(0, horizontalRoads.Length)] + Random.Range(-1f, 1f);
            }
            else
            {
                x = verticalRoads[Random.Range(0, horizontalRoads.Length)] + Random.Range(-1f, 1f);
                z = Random.Range(-24f, 24f);
            }
            transform.localPosition = new Vector3(x, 1.5f, z);
            transform.localRotation = Quaternion.Euler(0, Random.Range(-180f, 180f), 0);
            prevPos = transform.position;
            // target
            if (Random.value < .5f)
            {
                //choose verticalRoads
                x = Random.Range(-24f, 24f);
                z = horizontalRoads[Random.Range(0, horizontalRoads.Length)] + Random.Range(-1f, 1f);
            }
            else
            {
                x = verticalRoads[Random.Range(0, horizontalRoads.Length)] + Random.Range(-1f, 1f);
                z = Random.Range(-24f, 24f);
            }
            target = new Vector3(x, 1.5f, z);
            
        }
    }
    
}