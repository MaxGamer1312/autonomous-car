using System.Collections.Generic;
using UnityEngine;

namespace Tommy.Scripts.Training
{
    public class ParkingSpawner
    {
        private readonly List<Transform> spots = new List<Transform>();
        private readonly List<Renderer> goalNodes = new List<Renderer>();
        private Color defaultColor = Color.white;
        private Renderer currentGoalRenderer = null;

        public ParkingSpawner(Transform parent = null)
        {
            Refresh(parent);
            InitializeGoalNodes();
        }

        public void Refresh(Transform parent = null)
        {
            spots.Clear();
            if (parent != null)
            {
                for (int i = 0; i < parent.childCount; i++)
                {
                    var t = parent.GetChild(i);
                    if (t != null && t.CompareTag("Parking"))
                        spots.Add(t);
                }
            }
            else
            {
                foreach (var go in GameObject.FindGameObjectsWithTag("Parking"))
                    spots.Add(go.transform);
            }
            InitializeGoalNodes();
        }

        private void InitializeGoalNodes()
        {
            goalNodes.Clear();
            var nodes = GameObject.FindGameObjectsWithTag("Parking Node");
            foreach (var node in nodes)
            {
                var rend = node.GetComponent<Renderer>();
                if (rend != null)
                {
                    goalNodes.Add(rend);
                }
            }
            if (goalNodes.Count > 0)
            {
                defaultColor = goalNodes[0].material.color;
            }
        }

        public void SetRandomGoalNode()
        {
            if (goalNodes.Count == 0) return;

            // Reset previous goal node color
            if (currentGoalRenderer != null)
            {
                currentGoalRenderer.material.color = defaultColor;
            }

            int idx = Random.Range(0, goalNodes.Count);
            currentGoalRenderer = goalNodes[idx];
            currentGoalRenderer.material.color = Color.green;
        }

        public bool RandomLocation(Transform target, float surfaceYOffset = 0.1f, bool alignUpright = true)
        {
            if (spots.Count == 0 || target == null) return false;
            int idx = Random.Range(0, spots.Count);
            return PlaceAtSpot(spots[idx], target, surfaceYOffset, alignUpright);
        }

        public bool PlaceAtNearest(Transform target, float surfaceYOffset = 0.1f, bool alignUpright = true)
        {
            if (spots.Count == 0 || target == null) return false;

            Transform best = spots[0];
            float bestD = Vector3.SqrMagnitude(target.position - best.position);
            for (int i = 1; i < spots.Count; i++)
            {
                float d = Vector3.SqrMagnitude(target.position - spots[i].position);
                if (d < bestD)
                {
                    bestD = d;
                    best = spots[i];
                }
            }
            return PlaceAtSpot(best, target, surfaceYOffset, alignUpright);
        }

        public bool PlaceAtSpot(Transform spot, Transform target, float surfaceYOffset = 0.02f, bool alignUpright = true)
        {
            if (spot == null || target == null) return false;

            Vector3 pos = spot.position;

            // Use collider bounds if available
            var col = spot.GetComponent<Collider>();
            if (col != null)
            {
                var b = col.bounds;
                pos = new Vector3(b.center.x, b.max.y, b.center.z);
            }
            else
            {
                var rend = spot.GetComponent<Renderer>();
                if (rend != null)
                {
                    var b = rend.bounds;
                    pos = new Vector3(b.center.x, b.max.y, b.center.z);
                }
            }

            // Offset slightly upward to avoid intersection
            pos += spot.up * surfaceYOffset;

            // Align with parking spot's forward direction
            Vector3 flatForward = -spot.right;
            flatForward.y = 0f;

            Quaternion rot = Quaternion.LookRotation(flatForward.normalized, Vector3.up);

            target.SetPositionAndRotation(pos, rot);
            return true;
        }
    }
}