#define VISUALIZE

using System;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;

namespace Tommy.Scripts.Training
{
    public class UseCamera : Agent
    {
        [Range(-24, 19)]
        public float distance = 0;
        [Range(0, 25)]
        public float sideVariance = 0;
        [Range(0, 180)]
        public float rotationVariance = 0;

        public bool alternateRoads;

        public float groundTimer;
        private Vector3 target;
        
        
        public Transform groundParent;
        public Transform roadParent;
        
        [SerializeField]
        private CarController car;
        
        #if (UNITY_EDITOR && VISUALIZE)
        private Color drawingColor;
        #endif
        public override void Initialize()
        {
            
            
        }

        private void HalfGridWorld()
        {
            transform.localPosition = new Vector3(-22, 1, 0);
            transform.localRotation = Quaternion.Euler(0, Random.Range(90 - rotationVariance, 90 + rotationVariance), 0);
            target = new Vector3(distance, 1.5f, Random.Range(-sideVariance, sideVariance));

            if (alternateRoads)
            {
                if (Random.value >= .5)
                {
                    target = new Vector3(-0.1f, 1.5f, Random.Range(-sideVariance, sideVariance));
                }
                else
                {
                    target = new Vector3(19, 1.5f, Random.Range(-sideVariance, sideVariance));
                }
            }
        }

        private void FullGridWorld()
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

        public override void OnEpisodeBegin()
        {
            #if (UNITY_EDITOR && VISUALIZE)
            drawingColor = new Color(Random.value, Random.value, Random.value);
            #endif
            
            HalfGridWorld();
            //FullGridWorld();
            

            
            
            groundPenalty = 0;
            car.Restart();
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            const float MAP_EXTENTS = 27f;
            sensor.AddObservation(new Vector2(transform.localPosition.x / MAP_EXTENTS, transform.localPosition.z / MAP_EXTENTS));
            sensor.AddObservation(new Vector2(target.x / MAP_EXTENTS, target.y / MAP_EXTENTS));
            
            Vector3 dirTo = target - transform.position;
            dirTo.y = 0;
            Vector2 dirTo2D = new Vector2(dirTo.x, dirTo.z).normalized;
            
            Vector3 forward = transform.forward;
            Vector2 forward2D = new Vector2(forward.x, forward.z).normalized;
            
            
            float signedAngle = Vector2.SignedAngle(forward2D, dirTo2D)/180f;
            Debug.Log(signedAngle);
            sensor.AddObservation(signedAngle);
            sensor.AddObservation(car.forwardSpeed / car.maxSpeed);
            sensor.AddObservation(dirTo2D.magnitude);
        }

        public LayerMask ground;
        private float groundPenalty = 0;
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
            
            if (Vector3.SqrMagnitude(target - transform.position) < 1)
            {
                AddReward(1f);
                EndEpisode();
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
        }
    }
}