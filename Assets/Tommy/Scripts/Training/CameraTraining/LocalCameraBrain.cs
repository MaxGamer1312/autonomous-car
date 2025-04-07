#define VISUALIZE

using System;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Sensors.Reflection;
using UnityEngine;
using Random = UnityEngine.Random;

namespace Tommy.Scripts.Training
{
    public class LocalCameraBrain : Agent
    {
        public Vector2 direction;
        
        [Range(0, 180)]
        public float rotationVariance = 0;
        
        public float groundTimer;
        public float speedRunIncentiveTimer;
        private float _speedRunTimer;
        
        [SerializeField]
        private CarController car;
        
        #if (UNITY_EDITOR && VISUALIZE)
        private Color drawingColor;
        #endif
        
        public override void Initialize()
        {
            
        }
        
        public override void OnEpisodeBegin()
        {
            #if (UNITY_EDITOR && VISUALIZE)
            drawingColor = new Color(Random.value, Random.value, Random.value);
            #endif
            transform.rotation = Quaternion.Euler(0, Random.Range(-rotationVariance, rotationVariance), 0);
            _speedRunTimer = speedRunIncentiveTimer;
            groundPenalty = 0;
            car.Restart();
            transform.position = new Vector3(-10, 1.5f, -10);
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            Vector2 dirTo2D = direction.normalized;
            
            Vector3 forward = transform.forward;
            Vector2 forward2D = new Vector2(forward.x, forward.z).normalized;
            
            float signedAngle = Vector2.SignedAngle(forward2D, dirTo2D)/180f;
            sensor.AddObservation(signedAngle);
            
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
            
            
            float inputPower = actionBuffers.ContinuousActions[0];
            float inputSteeringAngle = actionBuffers.ContinuousActions[1];
            car.Drive(inputPower, inputSteeringAngle);
            
            
            Vector2 dirTo2D = direction.normalized;
            Vector3 forward = transform.forward;
            Vector2 forward2D = new Vector2(forward.x, forward.z).normalized;
            
            float signedAngle = Vector2.SignedAngle(forward2D, dirTo2D)/180f;
            

            if (Math.Abs(signedAngle) <= 3 / 180f)
            {
                AddReward(1f);
                EndEpisode();
            }
            
            _speedRunTimer -= Time.deltaTime;
            if (_speedRunTimer <= 0)
            {
                AddReward(1 - Math.Abs(signedAngle));
                EndEpisode();
            }
            
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
            Vector3 target = new Vector3(direction.x, 0, direction.y).normalized * 3;
            Gizmos.DrawRay(transform.position, target);
            Gizmos.DrawSphere(transform.position + target, .3f);
            
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