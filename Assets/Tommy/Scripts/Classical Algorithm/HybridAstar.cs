using System;
using System.Collections.Generic;
using System.Linq;
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
            HashSet<List<State>> results = new();

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
                
                if (iterations >= 5000)
                {
                    if (results.Any())
                    {
                        return results.OrderBy((a) =>
                        {
                            // float totalDistance = 0;
                            // for (int i = 0; i < a.Count - 1; i++)
                            // {
                            //     float dx = a[i + 1].pose.x - a[i].pose.x;
                            //     float dy = a[i + 1].pose.y - a[i].pose.y;
                            //     totalDistance += Mathf.Sqrt(dx * dx + dy * dy);
                            // }
                            float totalCost = 0;
                            foreach (var aState in a)
                            {
                                totalCost += aState.fCost;
                            }
                            return totalCost;
                        }).FirstOrDefault();
                    }
                    
                    return null;
                }

                State current = openSet.Dequeue();
                
                if (current.pose.DistanceTo(endPose) < 1f)
                {
                    results.Add(BackTrack(current));
                    return BackTrack(current);
                }
                closedSet.Add(current);
                //map.WorldToCell(current.pose.x, current.pose.y) = true;
                //ClassicalController.DebugCar(current.pose);
                ClassicalController.DebugCar(current.pose, new Color(1, 0, 0, .5f), 0f);
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
            
            //TODO: fix this
            const int tileWidth = 1; // map.boardWidth
            float maxSpeed = Mathf.Sqrt(tileWidth * tileWidth * 2) + .01f;
            float steeringAngle = 40 * Mathf.Deg2Rad;
            
            
            List<(float driveDistance, float turningAngle, float cost)> actions = new();

            actions.Add((-maxSpeed, 0, .5f));
            actions.Add((maxSpeed, 0, 0f));
            
            actions.Add((-maxSpeed, -steeringAngle * .25f, 2f));
            actions.Add((-maxSpeed, steeringAngle * .25f, 2f));
            
            
            actions.Add((maxSpeed, -steeringAngle * .25f, 2f));
            actions.Add((maxSpeed, steeringAngle * .25f, 2f));
            

            State[] result = new State[actions.Count];
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
                    gCost = parent.gCost + actions[i].cost + (map.WorldToCell(resultingPose.x, resultingPose.y) ? 0 : 10),
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
    
    public class ListComparer<T> : IEqualityComparer<List<T>>
    {
        public bool Equals(List<T> x, List<T> y)
        {
            if (x == null || y == null) return x == y;
            return x.SequenceEqual(y);
        }

        public int GetHashCode(List<T> obj)
        {
            if (obj == null) return 0;

            int hash = 17;
            foreach (T item in obj)
            {
                hash = hash * 23 + (item == null ? 0 : item.GetHashCode());
            }
            return hash;
        }
    }

}
