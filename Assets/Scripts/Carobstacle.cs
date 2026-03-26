using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Attach to any GameObject to make it visible to nearby CarDriverManager agents.
/// Set Environment Root to the same Roads/Road Parent transform used by
/// CarDriverManager so parallel training environments don't bleed into each other.
/// </summary>
public class CarObstacle : MonoBehaviour
{
    [Tooltip("Set this to the same Roads transform used by CarDriverManager's Road Parent field.")]
    [SerializeField] private Transform environmentRoot;

    // Pairs a Transform with its optional Rigidbody so observers can read velocity.
    public struct Entry
    {
        public Transform t;
        public Rigidbody rb; // null for static obstacles
    }

    private static Dictionary<Transform, List<Entry>> _registry
        = new Dictionary<Transform, List<Entry>>();

    private static readonly List<Entry> _empty = new List<Entry>();

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void ResetStaticState()
    {
        _registry = new Dictionary<Transform, List<Entry>>();
    }

    private void OnEnable()
    {
        if (environmentRoot == null)
        {
            Debug.LogWarning($"[CarObstacle] '{name}' has no Environment Root set — cars won't detect it.");
            return;
        }
        Register(environmentRoot, transform, GetComponent<Rigidbody>());
    }

    private void OnDisable()
    {
        Deregister(environmentRoot, transform);
    }

    // ── Called by CarDriverManager to register/deregister agent cars ────────

    public static void Register(Transform root, Transform t, Rigidbody rb = null)
    {
        if (root == null || t == null) return;
        if (!_registry.ContainsKey(root))
            _registry[root] = new List<Entry>();

        // Don't double-register
        var list = _registry[root];
        for (int i = 0; i < list.Count; i++)
            if (list[i].t == t) return;

        list.Add(new Entry { t = t, rb = rb });
    }

    public static void Deregister(Transform root, Transform t)
    {
        if (root == null || t == null || !_registry.ContainsKey(root)) return;
        var list = _registry[root];
        for (int i = list.Count - 1; i >= 0; i--)
            if (list[i].t == t) { list.RemoveAt(i); return; }
    }

    public static List<Entry> GetAll(Transform root)
    {
        if (root != null && _registry.TryGetValue(root, out var list))
            return list;
        return _empty;
    }
}