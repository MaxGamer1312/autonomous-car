using System.Collections.Generic;
using UnityEngine;

[ExecuteAlways]
public class LaneInstance : MonoBehaviour
{
    [System.Serializable]
    public class Node
    {
        public string label = "node";
        public Transform marker;                // the sphere placed at lane center
        public List<Transform> outputs = new(); // other sphere markers this node can go to
    }

    [Header("Nodes (add your lane-center spheres)")]
    public List<Node> nodes = new();           // “how many nodes there are” = nodes.Count

    [Header("Debug")]
    public bool debug = true;
    public float gizmoRadius = 0.25f;
    public Color nodeColor = new Color(0.2f, 0.9f, 1f, 0.9f);
    public Color arrowColor = new Color(1f, 0.6f, 0.1f, 0.9f);

    // Optional: quick helper to auto-pick child spheres as nodes once
    [ContextMenu("Sync nodes from child spheres")]
    void SyncFromChildSpheres()
    {
        nodes.Clear();
        foreach (Transform t in GetComponentsInChildren<Transform>())
        {
            if (t == transform) continue;
            if (t.GetComponent<SphereCollider>() != null || t.name.ToLower().Contains("lane"))
            {
                nodes.Add(new Node { label = t.name, marker = t });
            }
        }
    }

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

    void OnDrawGizmos()
    {
        if (!debug) return;

        // draw nodes
        Gizmos.color = nodeColor;
        foreach (var n in nodes)
        {
            if (n?.marker == null) continue;
            Gizmos.DrawSphere(n.marker.position, gizmoRadius);
        }

        // draw per-node outputs
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