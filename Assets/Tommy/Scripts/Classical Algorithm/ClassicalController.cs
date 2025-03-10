using System;
using UnityEngine;
using UnityEngine.UI;

namespace Tommy.Scripts.Classical_Algorithm
{
    public class ClassicalController : MonoBehaviour
    {
        public AStar astar;
        public CarController car;

        public RawImage debugImage;
        
        void Start()
        {
            astar = new AStar(55, 55);
            
            // initialize grid
            // world grid = 55x55
            for (int i = 0; i < astar.grid.GetLength(0); i++)
            {
                for (int j = 0; j < astar.grid.GetLength(1); j++)
                {
                    astar.grid[i, j] = 50;

                    if ((i >= 15 && i <= 19) || (i>=35 && i <=39))
                    {
                        astar.grid[i, j] = 0;
                    }
                    if ((j >= 15 && j <= 19) || (j>=35 && j <=39))
                    {
                        astar.grid[i, j] = 0;
                    }
                }
            }
            
            
            // debugging
            astar.InitDebugGrid(debugImage);
            astar.DebugGrid(debugImage);
            astar.ComputeShortestPath(Vector3.zero, Vector3.back);
        }

        public Transform dummy;
        public float timeStep = .5f;
        void Update()
        {
            float forward = Input.GetAxis("Vertical");
            float turn = Input.GetAxis("Horizontal");
            car.Drive(forward, turn);

            float velocity = car.forwardSpeed;
            float steerAngle = car.steeringAngle;
            
            float carAngleRad = car.transform.eulerAngles.y * Mathf.Deg2Rad;

            // arc math 

            float dx, dz;
            if (steerAngle != 0)
            {
                // L divided by tan
                float r = 2 / Mathf.Tan(steerAngle * Mathf.Deg2Rad);
                dx = velocity * Mathf.Sin(carAngleRad) + velocity/r * Mathf.Cos(carAngleRad);
                dz = velocity * Mathf.Cos(carAngleRad) - velocity/r * Mathf.Sin(carAngleRad);
            }
            else
            {
                dx = velocity * Mathf.Sin(carAngleRad);
                dz = velocity * Mathf.Cos(carAngleRad);
            }
            // constants
            float dTheta = (velocity / 2f) * Mathf.Tan(steerAngle * Mathf.Deg2Rad);

            dummy.position = transform.position + new Vector3(dx * timeStep, 0, dz * timeStep);

            Quaternion rot = transform.rotation * Quaternion.AngleAxis(dTheta * timeStep, Vector3.up);
            dummy.rotation = rot;
            
            DebugBox(dummy.position, dummy.rotation, dummy.lossyScale, Color.grey, 1f);
        }

        private void FixedUpdate()
        {
            astar.carPosition = new Vector2(Mathf.Lerp(0, 1, Mathf.InverseLerp(-27, 27, car.transform.position.z)), 
                                            Mathf.Lerp(0, 1, Mathf.InverseLerp(-27, 27, car.transform.position.x)));
            astar.carAngle = car.transform.eulerAngles.y * Mathf.Deg2Rad;
            astar.DebugGrid(debugImage);
        }


        public static void DebugBox(Vector3 pos, Quaternion rot, Vector3 scale, Color c, float duration = 0, bool depthTest = true)
        {
            Matrix4x4 m = new Matrix4x4();
            m.SetTRS(pos, rot, scale);

            var point1 = m.MultiplyPoint(new Vector3(-0.5f, -0.5f, 0.5f));
            var point2 = m.MultiplyPoint(new Vector3(0.5f, -0.5f, 0.5f));
            var point3 = m.MultiplyPoint(new Vector3(0.5f, -0.5f, -0.5f));
            var point4 = m.MultiplyPoint(new Vector3(-0.5f, -0.5f, -0.5f));

            // var point5 = m.MultiplyPoint(new Vector3(-0.5f, 0.5f, 0.5f));
            // var point6 = m.MultiplyPoint(new Vector3(0.5f, 0.5f, 0.5f));
            // var point7 = m.MultiplyPoint(new Vector3(0.5f, 0.5f, -0.5f));
            // var point8 = m.MultiplyPoint(new Vector3(-0.5f, 0.5f, -0.5f));

            Debug.DrawLine(point1, point2, c, duration, depthTest);
            Debug.DrawLine(point2, point3, c, duration, depthTest);
            Debug.DrawLine(point3, point4, c, duration, depthTest);
            Debug.DrawLine(point4, point1, c, duration, depthTest);

            // Debug.DrawLine(point5, point6, c);
            // Debug.DrawLine(point6, point7, c);
            // Debug.DrawLine(point7, point8, c);
            // Debug.DrawLine(point8, point5, c);
            //
            // Debug.DrawLine(point1, point5, c);
            // Debug.DrawLine(point2, point6, c);
            // Debug.DrawLine(point3, point7, c);
            // Debug.DrawLine(point4, point8, c);
        }
    }
}