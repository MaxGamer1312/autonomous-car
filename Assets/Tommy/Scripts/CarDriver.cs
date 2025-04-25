#define VISUALIZE

using System;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;

namespace Tommy.Scripts.Training
{
    public class CarDriver: Agent
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

        public Transform goal;
        
        
        public override void Initialize()
        {
            line = GetComponent<LineRenderer>();
            line.positionCount = 2;
            startPosition = transform.position;
            startRotation = transform.rotation;
        }

        public override void OnEpisodeBegin()
        {
            
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
            
            
        }

        private float maxDist = 0;
        private float maxHeading = 0;
        public override void OnActionReceived(ActionBuffers actionBuffers)
        {
            target = goal.position;
            line.SetPosition(0, transform.position);
            line.SetPosition(1, target);
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