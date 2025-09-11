using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace Tommy.Scripts.Classical_Algorithm
{
    public class ClassicalController : MonoBehaviour
    {
        public AStar astar;
        public CarController car;

        public RawImage debugImage;
        public Map map;
        private HybridAstar hybridAstar;

        private Pose2D startPosition;
        void Start()
        {
            startPosition = new Pose2D(transform.position.x, transform.position.z, transform.eulerAngles.y * Mathf.Deg2Rad);
            hybridAstar = new HybridAstar(map);
            
            
            //map.WorldToCell(0, 0) = true;
            // astar = new AStar(55, 55);
            //
            // // initialize grid
            // // world grid = 55x55
            // for (int i = 0; i < astar.grid.GetLength(0); i++)
            // {
            //     for (int j = 0; j < astar.grid.GetLength(1); j++)
            //     {
            //         astar.grid[i, j] = 50;
            //
            //         if ((i >= 15 && i <= 19) || (i>=35 && i <=39))
            //         {
            //             astar.grid[i, j] = 0;
            //         }
            //         if ((j >= 15 && j <= 19) || (j>=35 && j <=39))
            //         {
            //             astar.grid[i, j] = 0;
            //         }
            //     }
            // }


            // debugging
            // astar.InitDebugGrid(debugImage);
            // astar.DebugGrid(debugImage);
            // astar.ComputeShortestPath(Vector3.zero, new Vector3(10, 10, 30));
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
            
            // DebugBox(dummy.position, dummy.rotation, dummy.lossyScale, Color.grey, 1f);
            //
            // AStarState a = new AStarState();
            //a.pose = new Vector3(transform.position.x, transform.position.z, transform.eulerAngles.y);
            //astar.ChildStates(a);
        }

        private void FixedUpdate()
        {
        //     astar.carPosition = new Vector2(Mathf.Lerp(0, 1, Mathf.InverseLerp(-27, 27, car.transform.position.z)), 
        //                                     Mathf.Lerp(0, 1, Mathf.InverseLerp(-27, 27, car.transform.position.x)));
        //     astar.carAngle = car.transform.eulerAngles.y * Mathf.Deg2Rad;
        //     astar.DebugGrid(debugImage);
         }

        private void LateUpdate()
        {
            List<State> path = hybridAstar.FindPath(startPosition, new Pose2D(car.transform.position.x, car.transform.position.z, car.transform.eulerAngles.y * Mathf.Deg2Rad));

            if (path != null)
            {
                // foreach (var pose in path)
                // {
                //     Debug.Log(pose.pose);
                //     DebugCar(pose.pose, Color.green, 0f, 3);
                // }

                for (int i = 0; i < path.Count - 1; i++)
                {
                    int partitionCount = 10;
                    for (int j = 0; j < partitionCount; j++)
                    {
                        float delta = Pose2D.AngleWrap(path[i + 1].pose.heading - path[i].pose.heading);
                        Pose2D deltaPose = new Pose2D()
                        {
                            x = Mathf.Lerp(path[i].pose.x, path[i + 1].pose.x, (float)j / partitionCount),
                            y = Mathf.Lerp(path[i].pose.y, path[i + 1].pose.y, (float)j / partitionCount),
                            heading = path[i].pose.heading + Mathf.Lerp(0, delta, (float)j / partitionCount),
                        };
                        DebugCar(deltaPose, Color.green, 0f, 3);
                    }
                }
            }
        }


        public static void DebugBox(Vector3 pos, Quaternion rot, Vector3 scale, Color c, float duration = 0, bool depthTest = true)
        {
            Matrix4x4 m = new Matrix4x4();
            m.SetTRS(pos, rot, scale);

            var point1 = m.MultiplyPoint(new Vector3(-0.5f, -0.5f, 0.5f));
            var point2 = m.MultiplyPoint(new Vector3(0.5f, -0.5f, 0.5f));
            var point3 = m.MultiplyPoint(new Vector3(0.5f, -0.5f, -0.5f));
            var point4 = m.MultiplyPoint(new Vector3(-0.5f, -0.5f, -0.5f));

            var point5 = m.MultiplyPoint(new Vector3(-0.5f, 0.5f, 0.5f));
            var point6 = m.MultiplyPoint(new Vector3(0.5f, 0.5f, 0.5f));
            var point7 = m.MultiplyPoint(new Vector3(0.5f, 0.5f, -0.5f));
            var point8 = m.MultiplyPoint(new Vector3(-0.5f, 0.5f, -0.5f));

            Debug.DrawLine(point1, point2, c, duration, depthTest);
            Debug.DrawLine(point2, point3, c, duration, depthTest);
            Debug.DrawLine(point3, point4, c, duration, depthTest);
            Debug.DrawLine(point4, point1, c, duration, depthTest);

            // Debug.DrawLine(point5, point6, c);
            // Debug.DrawLine(point6, point7, c);
            // Debug.DrawLine(point7, point8, c);
            // Debug.DrawLine(point8, point5, c);
            
            // Debug.DrawLine(point1, point5, c);
            // Debug.DrawLine(point2, point6, c);
            // Debug.DrawLine(point3, point7, c);
            // Debug.DrawLine(point4, point8, c);
        }

        public static void DebugCar(Pose2D pose, Color c, float duration = 10, float height = 2)
        {
            DebugBox(new Vector3(pose.x, height, pose.y), Quaternion.Euler(0, pose.heading * Mathf.Rad2Deg, 0), new Vector3(1, 1, 2), c, duration);
        }
    }
}