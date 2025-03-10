using UnityEngine;
namespace Tommy.Scripts.Classical_Algorithm
{
    public class AStarState : IHeapItem<AStarState>
    {
        // (x, y, theta)
        public Vector3 pose;
        public bool isReversing;

        public float gCost; // cost from starting node
        public float hCost; // heuristic to end node
        public float fcost => gCost + hCost;
        public AStarState parent;
        int heapIndex;
        
        public int HeapIndex {
            get {
                return heapIndex;
            }
            set {
                heapIndex = value;
            }
        }

        public int CompareTo(AStarState otherState)
        {
            int compare = fcost.CompareTo(otherState.fcost);

            if (Mathf.Approximately(fcost, otherState.fcost))
                compare = 0;
            
            if (compare == 0)
            {
                return hCost.CompareTo(otherState.hCost);
            }
            
            return -compare;
        }

        public override bool Equals(object obj)
        {
            if (obj is AStarState otherPose)
            {
                Vector3Int o = Vector3Int.RoundToInt(otherPose.pose);
                Vector3Int thisPose = Vector3Int.RoundToInt(otherPose.pose);
                return thisPose.Equals(o);
            }

            return false;
        }
        
        public override int GetHashCode()
        {
            unchecked
            {
                int hash = 17;
                hash = hash * 23 + pose.x.GetHashCode();
                hash = hash * 23 + pose.y.GetHashCode();
                hash = hash * 23 + pose.z.GetHashCode();
                return hash;
            }
        }
    }
}