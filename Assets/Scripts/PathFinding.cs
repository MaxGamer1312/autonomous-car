using System.Collections.Generic;
using UnityEngine;

using System.Collections.Generic;
using UnityEngine;

public class PathFinding : MonoBehaviour
{
    [Header("Visuals")]
    [SerializeField] private Color pathColor = Color.green;
    [SerializeField] private Color defaultColor = Color.white;
    [SerializeField] private float lineHeight = 0.3f;
    [SerializeField] private float lineWidth = 0.2f;

    [Header("Path Following")]
    [SerializeField] private bool bidirectional = false;

    [Header("Debug")]
    [SerializeField] private bool debugLogs = false;
    [SerializeField] private bool pathDrawing;

    private const string LineLayerName = "BFS Line";
    private const string LineChildName = "BFS_PathLine";

    private List<Transform> currentPath = new List<Transform>();
    private Dictionary<Transform, Renderer> nodeRenderers = new Dictionary<Transform, Renderer>();
    private Transform car;
    private Transform goal;
    private Transform _goalNode;
    private Transform _anchorNode;      // last correct node (start node at episode start)
    private Transform _lastStartNode;   // node the car spawned on / closest at compute time
    private LineRenderer pathLine;
    private int _currentNodeIndex = 0;  // index into currentPath for "next node"
    private readonly HashSet<Transform> _visited = new HashSet<Transform>();

    private void Awake()
    {
        // Ensure a dedicated child GO for the line so we don't change the layer of this whole object
        Transform child = transform.Find(LineChildName);
        GameObject lineGO;
        if (child == null)
        {
            lineGO = new GameObject(LineChildName);
            lineGO.transform.SetParent(transform, false);
        }
        else
        {
            lineGO = child.gameObject;
        }

        // Assign layer
        int bfsLayer = LayerMask.NameToLayer(LineLayerName);
        if (bfsLayer == -1)
        {
            Debug.LogWarning($"[PathFinding] Layer '{LineLayerName}' not found. Create it in Project Settings > Tags and Layers.");
        }
        else
        {
            lineGO.layer = bfsLayer;
        }

        // Get or add LineRenderer on the child
        pathLine = lineGO.GetComponent<LineRenderer>();
        if (pathLine == null) pathLine = lineGO.AddComponent<LineRenderer>();

        pathLine.useWorldSpace = true;
        pathLine.startWidth = lineWidth;
        pathLine.endWidth = lineWidth;
        pathLine.material = new Material(Shader.Find("Unlit/Color"));
        if (pathDrawing)
        {
            pathLine.material.color = pathColor;
        }
        pathLine.positionCount = 0;
        pathLine.enabled = false;
    }

    /// Called each step by CarDriverNew to keep refs and redraw (no recompute here).
    public void UpdatePath(Transform carTransform, Transform goalTransform)
    {
        car = carTransform;
        goal = goalTransform;
        DrawPathLine();
    }

    public void UpdateVisualization() => DrawPathLine();

    public void DisableVisualization()
    {
        if (pathLine != null)
        {
            pathLine.enabled = false;
            pathLine.positionCount = 0;
        }
        ClearCurrentPathVisual();
    }

    public void ResetPathState()
    {
        DisableVisualization();
        currentPath.Clear();
        nodeRenderers.Clear();
        _anchorNode = null;
        _goalNode = null;
        car = null;
        goal = null;
        _currentNodeIndex = 0;
        _lastStartNode = null;
        _visited.Clear();
    }

    /// Used on episode start (or when you explicitly want a fresh path).
    public void ForceRecomputeNow(Transform carT, Transform goalT)
    {
        car = carT;
        goal = goalT;
        _currentNodeIndex = 0;
        ComputeAndHighlightPath();
        DrawPathLine();
    }

    private bool TryGetOutputsForMarker(Transform marker, out List<Transform> outputs)
    {
        outputs = null;
        if (marker == null) return false;
        var owner = marker.GetComponentInParent<LaneInstance>();
        if (owner == null) return false;
        var node = owner.GetNodeByMarker(marker);
        if (node == null) return false;
        outputs = node.outputs;
        return outputs != null;
    }

    private bool TryGetNeighbors(Transform marker, List<Transform> neighbors)
    {
        neighbors.Clear();
        if (!TryGetOutputsForMarker(marker, out var outs)) return false;

        foreach (var t in outs)
            if (t != null) neighbors.Add(t);

        if (bidirectional)
        {
            foreach (var li in FindObjectsOfType<LaneInstance>())
            {
                foreach (var n in li.nodes)
                {
                    if (n?.marker == null || n.outputs == null) continue;
                    foreach (var o in n.outputs)
                    {
                        if (o == marker && n.marker != null)
                            neighbors.Add(n.marker);
                    }
                }
            }
        }
        return neighbors.Count > 0;
    }

    private void ComputeAndHighlightPath()
    {
        ClearCurrentPathVisual();
        currentPath.Clear();

        if (car == null || goal == null) return;

        // Collect ALL navigable nodes — Parking Node, Node, and Non Parking Node are all
        // valid graph vertices for routing. Non Parking Node just can't be a spawn/goal,
        // but the car still needs to drive through them to reach other destinations.
        var nodeGOs = GetAllGraphNodes();
        if (nodeGOs.Length == 0)
        {
            if (debugLogs) Debug.LogWarning("⚠️ No nodes found!");
            return;
        }

        _goalNode = IsNode(goal) ? goal : ClosestNodeTo(goal.position, nodeGOs);
        if (_goalNode == null) return;

        Transform startNode = _anchorNode != null ? _anchorNode : ClosestNodeTo(car.position, nodeGOs);
        if (startNode == null) return;

        _lastStartNode = startNode;
        _anchorNode = startNode;

        var cameFrom = new Dictionary<Transform, Transform>();
        var frontier = new Queue<Transform>();
        var neighbors = new List<Transform>(8);

        frontier.Enqueue(startNode);
        cameFrom[startNode] = null;

        bool found = false;
        while (frontier.Count > 0)
        {
            var current = frontier.Dequeue();
            if (current == _goalNode) { found = true; break; }

            if (!TryGetNeighbors(current, neighbors)) continue;
            foreach (var nb in neighbors)
            {
                if (nb == null || cameFrom.ContainsKey(nb)) continue;
                cameFrom[nb] = current;
                frontier.Enqueue(nb);
            }
        }

        if (!found || !cameFrom.ContainsKey(_goalNode)) return;

        // Reconstruct
        var node = _goalNode;
        while (node != null)
        {
            currentPath.Add(node);
            node = cameFrom[node];
        }
        currentPath.Reverse();

        // Do NOT include the car's starting node as a target (no reward there)
        if (currentPath.Count > 0 && (currentPath[0] == _anchorNode || currentPath[0] == _lastStartNode))
            currentPath.RemoveAt(0);

        // Remove nodes already visited so recomputes don’t turn them green again
        if (_visited.Count > 0)
            currentPath.RemoveAll(t => _visited.Contains(t));

        _currentNodeIndex = 0;

        // color the remaining path nodes
        if (pathDrawing)
        {
            foreach (Transform t in currentPath)
            {
                var r = t.GetComponentInChildren<Renderer>();
                if (r != null)
                {
                    nodeRenderers[t] = r;

                    r.material.color = pathColor;
                }
            }

        }
        DrawPathLine();
    }

    private Transform ClosestNodeTo(Vector3 pos, GameObject[] nodeGOs)
    {
        Transform best = null;
        float bestD = float.MaxValue;
        foreach (var go in nodeGOs)
        {
            var t = go.transform;
            float d = (t.position - pos).sqrMagnitude;
            if (d < bestD) { bestD = d; best = t; }
        }
        return best;
    }

    private static GameObject[] GetAllGraphNodes()
    {
        // All three tags are valid BFS graph vertices.
        // "Non Parking Node" = can't spawn/goal there, but routing still passes through them.
        var result = new List<GameObject>();
        foreach (var go in GameObject.FindGameObjectsWithTag("Parking Node")) result.Add(go);
        foreach (var go in GameObject.FindGameObjectsWithTag("Node")) result.Add(go);
        foreach (var go in GameObject.FindGameObjectsWithTag("Non Parking Node")) result.Add(go);
        return result.ToArray();
    }

    private bool IsNode(Transform t)
    {
        if (t == null) return false;
        return t.CompareTag("Parking Node") || t.CompareTag("Node") || t.CompareTag("Non Parking Node");
    }

    private void DrawPathLine()
    {
        if (pathLine == null) return;
        if (!pathDrawing)
        {
            return;
        }
        if (currentPath == null || currentPath.Count < 2)
        {
            pathLine.enabled = false;
            pathLine.positionCount = 0;
            return;
        }

        pathLine.enabled = true;
        pathLine.positionCount = currentPath.Count;

        for (int i = 0; i < currentPath.Count; i++)
        {
            Vector3 p = currentPath[i].position + Vector3.up * lineHeight;
            pathLine.SetPosition(i, p);
        }
    }

    private void ClearCurrentPathVisual()
    {
        foreach (var kvp in nodeRenderers)
        {
            if (kvp.Value != null) kvp.Value.material.color = defaultColor;
        }
        nodeRenderers.Clear();
    }

    private void Update() => DrawPathLine();

    public Transform GetNextNode()
    {
        if (currentPath == null || currentPath.Count == 0 || _currentNodeIndex >= currentPath.Count) return null;
        return currentPath[_currentNodeIndex];
    }

    /// Return up to k upcoming nodes starting from the current "next".
    public List<Transform> GetNextKNodes(int k)
    {
        var res = new List<Transform>(k);
        if (currentPath == null || k <= 0) return res;
        for (int i = _currentNodeIndex; i < currentPath.Count && res.Count < k; i++)
            res.Add(currentPath[i]);
        return res;
    }

    /// Advance only when the right node is hit; also recompute fresh path from new anchor.
    public void AdvanceIfNodeReached(Transform node)
    {
        if (currentPath == null || _currentNodeIndex >= currentPath.Count) return;

        var expected = currentPath[_currentNodeIndex];
        if (node != expected) return;

        _visited.Add(node);
        _anchorNode = node;
        _currentNodeIndex++;

        ComputeAndHighlightPath();
    }

    /// Expose the current anchor (last correct node; start node at episode start)
    public Transform GetAnchorNode() => _anchorNode != null ? _anchorNode : _lastStartNode;

    public bool IsEpisodeSpawnNode(Transform t) => t == _lastStartNode;
}