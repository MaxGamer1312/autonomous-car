using System;
using UnityEngine;
using UnityEngine.UI;

namespace Tommy.Scripts.Classical_Algorithm
{
    public class ClassicalController : MonoBehaviour
    {
        public AStar astar;
        public PurePursuit purePursuit;
        public CarController car;

        public RawImage debugImage;
        
        void Start()
        {
            astar = new AStar(100, 100);
            
            // initialize grid
            // world grid = 55x55
            
            
            // debugging
            astar.InitDebugGrid(debugImage);
            astar.DebugGrid(debugImage);
        }

        void Update()
        {
            float forward = Input.GetAxis("Vertical");
            float turn = Input.GetAxis("Horizontal");
            car.Drive(forward, turn);
        }

        private void FixedUpdate()
        {
            astar.carPosition = new Vector2(Mathf.Lerp(0, 1, Mathf.InverseLerp(-27, 27, car.transform.position.z)), 
                                            Mathf.Lerp(0, 1, Mathf.InverseLerp(-27, 27, car.transform.position.x)));
            astar.carAngle = car.transform.eulerAngles.y * Mathf.Deg2Rad;
            astar.DebugGrid(debugImage);
        }
        
    }
}