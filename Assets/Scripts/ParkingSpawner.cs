using System.Collections.Generic;
using Unity.MLAgents;
using UnityEngine;

namespace Tommy.Scripts.Training
{
    // Manages the starting location of the car and goal in each episode
    public class ParkingSpawner
    {
        // List of locations where the car will spawn
        private List<Transform> startingLocations = new List<Transform>();
        // List of nodes where the goal will spawn
        private List<Renderer> FinishNodes = new List<Renderer>();
        // Default color to reset green nodes when episode ends
        private Color defaultColor;
        private Renderer currentGoalRenderer = null;
        private Transform _parent;

        public ParkingSpawner(Transform parent = null)
        {
            _parent = parent;
            Refresh();
        }

        // Clears and initializes startingLocations and FinishNodes
        public void Refresh() {
            startingLocations = new List<Transform>();
            FinishNodes = new List<Renderer>();

            if (_parent != null)
            {
                // Needed to localize locations for each individual map
                foreach (Transform child in _parent.GetComponentsInChildren<Transform>(false))
                {
                    if (child == _parent)
                    {
                        continue;
                    }

                    if (child.CompareTag("Parking"))
                    {
                        startingLocations.Add(child);
                    }

                    else if (child.CompareTag("Parking Node"))
                    {
                        var rend = child.GetComponent<Renderer>();
                        if (rend != null)
                        {
                            FinishNodes.Add(rend);
                        }
                    }
                }
            }
            else
            {
                foreach (var startingLocation in GameObject.FindGameObjectsWithTag("Parking"))
                {
                    startingLocations.Add(startingLocation.transform);
                }

                foreach (var finishNode in GameObject.FindGameObjectsWithTag("Parking Node"))
                {
                    var rend = finishNode.GetComponent<Renderer>();
                    if (rend != null)
                    {
                        FinishNodes.Add(rend);
                    }
                }
            }

            if (FinishNodes.Count > 0)
            {
                defaultColor = FinishNodes[0].material.color;
            }
        }

        // Resets green nodes to old color and finds another random goal different from where the car is
        public Transform GetRandomGoal(Transform playerTransform, float minDistance){
            if (FinishNodes.Count == 0) {
                return null;
            }

            // Reset the old color (if we had one)
            if (currentGoalRenderer != null)
            {
                currentGoalRenderer.material.color = defaultColor;
            }

            // Pick a valid random finish node (checking distance)
            Renderer chosenRenderer = null;
            int guard = 0;
            do
            {
                int idx = Random.Range(0, FinishNodes.Count);
                chosenRenderer = FinishNodes[idx];
                guard++;
            }
            // Keep picking if it's too close to the player
            while (Vector3.Distance(playerTransform.position, chosenRenderer.transform.position) < minDistance && guard < 50);

            // Highlight the new goal
            currentGoalRenderer = chosenRenderer;
            currentGoalRenderer.material.color = Color.green;

            // Return the Transform so the Agent can use it
            return currentGoalRenderer.transform;
        }

        // Puts target in a random location from startingLocations
        public bool RandomLocation(Transform target, float surfaceYOffset = 0.1f, bool alignUpright = true) {
            if (startingLocations.Count == 0 || target == null) {
                return false;
            }

            // Picks a random location from startingLocations
            int idx = Random.Range(0, startingLocations.Count);
            Transform chosenLocation = startingLocations[idx];
            if (chosenLocation == null || target == null) {
                return false;
            }
            Vector3 pos = chosenLocation.position;

            // Use collider bounds if available
            var col = chosenLocation.GetComponent<Collider>();
            if (col != null)
            {
                var b = col.bounds;
                pos = new Vector3(b.center.x, b.max.y, b.center.z);
            }
            else
            {
                var rend = chosenLocation.GetComponent<Renderer>();
                if (rend != null)
                {
                    var b = rend.bounds;
                    pos = new Vector3(b.center.x, b.max.y, b.center.z);
                }
            }

            // Offset slightly upward to avoid intersection
            pos += chosenLocation.up * surfaceYOffset;

            // Align with parking spot's forward direction
            Vector3 flatForward = -chosenLocation.right;
            flatForward.y = 0f;
            Quaternion rot = Quaternion.LookRotation(flatForward.normalized, Vector3.up);

            target.SetPositionAndRotation(pos, rot);
            return true;
        }
    }
}