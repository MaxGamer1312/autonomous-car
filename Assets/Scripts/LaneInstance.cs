using System.Collections.Generic;
using UnityEngine;
using System.Collections;
public enum NodeType
{
    None,
    EnterNode,
    ExitNode
}

public class LaneInstance : MonoBehaviour
{
    [System.Serializable]
    public class Node
    {
        public string label = "node";
        public Transform marker;
        public NodeType nodeType = NodeType.None;
        public List<Transform> outputs = new();
    }

    [Header("Nodes")]
    public List<Node> nodes = new();

    [Header("Debug")]
    public bool debug = true;
    public float gizmoRadius = 0.25f;
    public Color nodeColor = new Color(0.2f, 0.9f, 1f, 0.9f);
    public Color arrowColor = new Color(1f, 0.6f, 0.1f, 0.9f);

    void Start()
    {
        // Prepare the Exit Nodes to listen for collision events dynamically
        foreach (Node myNode in nodes)
        {
            if (myNode.nodeType == NodeType.ExitNode && myNode.marker != null)
            {
                GameObject markerObj = myNode.marker.gameObject;

                // 1. Ensure it has a Rigidbody to trigger physics events
                Rigidbody rb = markerObj.GetComponent<Rigidbody>();
                if (rb == null) rb = markerObj.AddComponent<Rigidbody>();
                rb.isKinematic = true; // Prevents gravity from pulling it down
                rb.useGravity = false;

                // 2. Attach our custom event listener to the marker
                NodeCollisionListener listener = markerObj.GetComponent<NodeCollisionListener>();
                if (listener == null) listener = markerObj.AddComponent<NodeCollisionListener>();

                listener.parentLane = this;
                listener.myNode = myNode;
            }
        }
    }

    // This catches the event, but hands it off to a Coroutine to delay it safely
    public void HandleNodeWeldEvent(Node exitNode, Collider hitCollider)
    {
        StartCoroutine(WeldAfterPhysicsStep(exitNode, hitCollider));
    }

    // This waits for the physics engine to finish, THEN does the welding and deleting
    private IEnumerator WeldAfterPhysicsStep(Node exitNode, Collider hitCollider)
    {
        // Wait until the end of the physics frame
        yield return new WaitForFixedUpdate();

        // Guard check: Ensure we haven't already deleted this node this frame
        if (exitNode == null || exitNode.marker == null || !nodes.Contains(exitNode)) yield break;

        LaneInstance[] allLanes = FindObjectsOfType<LaneInstance>();
        LaneInstance otherLane = null;
        Node enterNode = null;

        // Find the specific Enter Node that we just collided with
        foreach (LaneInstance lane in allLanes)
        {
            if (lane == this) continue;

            foreach (Node n in lane.nodes)
            {
                if (n.nodeType == NodeType.EnterNode && hitCollider != null && n.marker == hitCollider.transform)
                {
                    otherLane = lane;
                    enterNode = n;
                    break;
                }
            }
            if (enterNode != null) break;
        }

        // If we found a valid match, wire them up and delete them
        if (enterNode != null && otherLane != null)
        {
            // 1. Reroute connections
            foreach (Node prevNode in nodes)
            {
                if (prevNode.outputs.Contains(exitNode.marker))
                {
                    prevNode.outputs.Remove(exitNode.marker);
                    foreach (Transform nextMarker in enterNode.outputs)
                    {
                        if (!prevNode.outputs.Contains(nextMarker))
                        {
                            prevNode.outputs.Add(nextMarker);
                        }
                    }
                }
            }

            // 2. Remove them from the scripts' tracking lists
            nodes.Remove(exitNode);
            otherLane.nodes.Remove(enterNode);

            // 3. Destroy the physical game objects from the scene safely
            Destroy(exitNode.marker.gameObject);
            Destroy(enterNode.marker.gameObject);

            Debug.Log($"Event Triggered: Welded {gameObject.name} to {otherLane.gameObject.name}!");
        }
    }

    // --- Helper Functions for Pathfinding ---
    public int NodeCount => nodes.Count;

    public Node GetNodeByMarker(Transform marker)
    {
        for (int i = 0; i < nodes.Count; i++)
            if (nodes[i].marker == marker) return nodes[i];
        return null;
    }

    public List<Transform> GetOutputs(int nodeIndex)
    {
        if (nodeIndex < 0 || nodeIndex >= nodes.Count) return null;
        return nodes[nodeIndex].outputs;
    }

    // --- Gizmo Drawing ---
    void OnDrawGizmos()
    {
        if (!debug) return;

        foreach (var n in nodes)
        {
            if (n?.marker == null) continue;

            if (n.nodeType == NodeType.EnterNode) Gizmos.color = Color.green;
            else if (n.nodeType == NodeType.ExitNode) Gizmos.color = Color.red;
            else Gizmos.color = nodeColor;

            Gizmos.DrawSphere(n.marker.position, gizmoRadius);
        }

        Gizmos.color = arrowColor;
        foreach (var n in nodes)
        {
            if (n?.marker == null) continue;
            var a = n.marker.position;
            foreach (var outT in n.outputs)
            {
                if (outT == null) continue;
                DrawArrow(a, outT.position);
            }
        }
    }

    void DrawArrow(Vector3 a, Vector3 b, float headLen = 0.8f, float headAngle = 20f)
    {
        Gizmos.DrawLine(a, b);
        var dir = (b - a);
        if (dir.sqrMagnitude < 1e-6f) return;
        var d = dir.normalized;
        var q = Quaternion.LookRotation(d);
        var r = q * Quaternion.Euler(0, 180 + headAngle, 0) * Vector3.forward * headLen;
        var l = q * Quaternion.Euler(0, 180 - headAngle, 0) * Vector3.forward * headLen;
        Gizmos.DrawLine(b, b + r);
        Gizmos.DrawLine(b, b + l);
    }
}

// --- Small Helper Class to listen for collisions on the individual nodes ---
public class NodeCollisionListener : MonoBehaviour
{
    public LaneInstance parentLane;
    public LaneInstance.Node myNode;

    void OnTriggerEnter(Collider other)
    {
        if (parentLane != null)
        {
            parentLane.HandleNodeWeldEvent(myNode, other);
        }
    }
}