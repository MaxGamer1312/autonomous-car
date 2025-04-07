using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;
using Object = System.Object;

namespace Tommy.Scripts.Classical_Algorithm
{
    public class HybridAstar
    {
        private Map map;

        public HybridAstar(Map map)
        {
            this.map = map;
        }
        public List<State> FindPath(Pose2D startPose, Pose2D endPose)
        {
            Heap<State> openSet = new Heap<State>();
            HashSet<State> closedSet = new HashSet<State>();

            State startingState = new State
            {
                pose = startPose,
                gCost = 0,
                hCost = startPose.DistanceTo(endPose)
            };
            openSet.Enqueue(startingState);

            int iterations = 0;
            while (!openSet.IsEmpty())
            {
                
                if (iterations >= 3000)
                {
                    Debug.Log("Terminated Early");
                    return null;
                }

                State current = openSet.Dequeue();
                
                if (current.pose.DistanceTo(endPose) < 1f)
                {
                    Debug.Log("Found Path");
                    return BackTrack(current);
                }
                closedSet.Add(current);
                //map.WorldToCell(current.pose.x, current.pose.y) = true;
                //ClassicalController.DebugCar(current.pose);
                ClassicalController.DebugCar(current.pose, Color.red, 0f);
                foreach (var child in ChildStates(current, endPose))
                {
                    if(map.OutOfBounds(child.pose.x, child.pose.y))
                        continue;
                    if (closedSet.Contains(child))
                        continue;
                    openSet.Enqueue(child);
                }

                iterations++;
            }
            Debug.Log("No Paths Existed");
            return null;
        }

        private List<State> BackTrack(State end)
        {
            List<State> result = new();
            State current = end;
            while (current != null)
            {
                result.Add(current);
                current = current.parent;
            }
            result.Reverse();
            return result;
        }

        
        private State[] ChildStates(State parent, Pose2D endPose)
        {
            State[] result = new State[6];
            float maxSpeed = Mathf.Sqrt(map.boardWidth * map.boardWidth * 2) + .01f;
            float steeringAngle = 40 * Mathf.Deg2Rad;
            
            (float driveDistance, float turningAngle, float cost)[] actions =
            {
                (-maxSpeed, -steeringAngle, .2f),
                (-maxSpeed, 0, .1f),
                (-maxSpeed, steeringAngle, .2f),
                (maxSpeed, -steeringAngle, .02f),
                (maxSpeed, 0,  0f),
                (maxSpeed, steeringAngle, .02f),
            };
            for (int i = 0; i < result.Length; i++)
            {
                
                float driveDistance = actions[i].driveDistance;
                //Steering angle
                float alpha = actions[i].turningAngle;

                //TODO: read note below
                //Turning angle             // 1f is wheel base (might need to tinker with later)
                float beta = (driveDistance / 2f) * Mathf.Tan(alpha);

                // calculate new pose

                Pose2D resultingPose = new Pose2D();
                if (Mathf.Abs(beta) < 0.00001f)
                {
                    // drive forward
                    resultingPose.x = parent.pose.x + driveDistance * Mathf.Sin(parent.pose.heading);
                    resultingPose.y = parent.pose.y + driveDistance * Mathf.Cos(parent.pose.heading);
                }
                //Turn
                else
                {
                    //Turning radius 
                    float R = driveDistance / beta;
                
                    float cx = parent.pose.x + Mathf.Cos(parent.pose.heading) * R;
                    float cz = parent.pose.y - Mathf.Sin(parent.pose.heading) * R;
                
                    resultingPose.x = cx - Mathf.Cos(parent.pose.heading + beta) * R;
                    resultingPose.y = cz + Mathf.Sin(parent.pose.heading + beta) * R;
                }

                resultingPose.heading = Pose2D.AngleWrap(parent.pose.heading + actions[i].turningAngle);

                result[i] = new State()
                {
                    pose = resultingPose,
                    gCost = parent.gCost + actions[i].cost,
                    hCost = resultingPose.DistanceTo(endPose),
                    parent = parent,
                    isReversing = actions[i].driveDistance < 0
                }; 
                
            }
            return result;
        }
    }
    
    

    public class State : IHeapElement<State>
    {
        public Pose2D pose;
        public bool isReversing;
        
        public float gCost;
        public float hCost;
        public float fCost => gCost + hCost;

        public State parent;

        public int HeapIndex { get; set; }

        public State()
        {
        }
        

        public int CompareTo(State other)
        {
            if (Mathf.Approximately(fCost, other.fCost))
            {
                return hCost - other.hCost < 0 ? -1 :  1;
            }
            return fCost - other.fCost < 0 ? -1 :  1;
        }

        public override bool Equals(Object obj)
        {
            if (obj is not State other) return false;
            return pose.Equals(other.pose);
        }

        public override int GetHashCode()
        {
            return pose.GetHashCode();
        }
    }
}