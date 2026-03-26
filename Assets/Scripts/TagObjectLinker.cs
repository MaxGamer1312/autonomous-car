using UnityEngine;
using System;
using System.Collections.Generic;

[Serializable]
public struct TagMapping
{
    public int tagID;
    public GameObject prefabToSpawn;

    [Tooltip("Multiplier: 1,1,1 fills the cell exactly. Adjust to shrink/grow slightly.")]
    public Vector3 customScale;

    [Tooltip("Offset rotation if the model faces the wrong way (e.g., 0, 90, 0).")]
    public Vector3 rotationOffset;
}

public class TagObjectLinker : MonoBehaviour
{
    [Header("Tag to Object Mappings")]
    public List<TagMapping> objectMappings = new List<TagMapping>();

    [Header("Settings")]
    public float hideTimeout = 0.5f;
    [Tooltip("If true, road height will match the Y-scale of the map grid.")]
    public bool autoScaleHeight = true;

    private class SpawnedData
    {
        public GameObject instance;
        public float lastSeenTime;
    }

    private Dictionary<int, TagMapping> _mappingsDict = new Dictionary<int, TagMapping>();
    private Dictionary<int, SpawnedData> _spawnedObjects = new Dictionary<int, SpawnedData>();

    void Awake()
    {
        foreach (var m in objectMappings)
        {
            if (m.prefabToSpawn != null && !_mappingsDict.ContainsKey(m.tagID))
            {
                TagMapping safeMapping = m;
                if (safeMapping.customScale == Vector3.zero) safeMapping.customScale = Vector3.one;
                _mappingsDict.Add(m.tagID, safeMapping);
            }
        }
    }

    void Update()
    {
        foreach (var kvp in _spawnedObjects)
        {
            if (kvp.Value.instance.activeSelf && Time.time - kvp.Value.lastSeenTime > hideTimeout)
            {
                kvp.Value.instance.SetActive(false);
            }
        }
    }

    public void UpdateObjectPose(int id, Vector3 pos, Quaternion rot, Vector3 cellSize)
    {
        if (!_mappingsDict.TryGetValue(id, out TagMapping mapping)) return;

        if (_spawnedObjects.TryGetValue(id, out SpawnedData data))
        {
            UpdateTransform(data.instance, mapping, pos, rot, cellSize);
            data.lastSeenTime = Time.time;
            if (!data.instance.activeSelf) data.instance.SetActive(true);
        }
        else
        {
            GameObject newObj = Instantiate(mapping.prefabToSpawn);
            newObj.name = "LinkedObject_Tag_" + id;
            UpdateTransform(newObj, mapping, pos, rot, cellSize);
            _spawnedObjects.Add(id, new SpawnedData { instance = newObj, lastSeenTime = Time.time });
        }
    }

    private void UpdateTransform(GameObject obj, TagMapping mapping, Vector3 pos, Quaternion rot, Vector3 cellSize)
    {
        obj.transform.position = pos;
        // Trust the detector's confirmed, flattened, and cone-snapped rotation
        obj.transform.rotation = rot * Quaternion.Euler(mapping.rotationOffset);
        FitObjectToGrid(obj, cellSize, mapping.customScale);
    }

    private void FitObjectToGrid(GameObject obj, Vector3 targetSize, Vector3 manualTweaks)
    {
        // Reset scale to detect raw mesh size
        obj.transform.localScale = Vector3.one;

        Renderer[] renderers = obj.GetComponentsInChildren<Renderer>();
        if (renderers.Length == 0) return;

        // Measure the actual 3D bounds
        Bounds bounds = renderers[0].bounds;
        foreach (Renderer r in renderers) bounds.Encapsulate(r.bounds);

        float sizeX = bounds.size.x > 0.001f ? bounds.size.x : 1f;
        float sizeY = bounds.size.y > 0.001f ? bounds.size.y : 1f;
        float sizeZ = bounds.size.z > 0.001f ? bounds.size.z : 1f;

        // Since the object might be rotated 90 degrees, the bounds X and Z might be swapped.
        // We use the object's local forward/right to be more precise, but for roads, 
        // the targetSize.x and targetSize.z are usually equal in a square grid anyway.
        float ratioX = targetSize.x / sizeX;
        float ratioY = targetSize.y / sizeY;
        float ratioZ = targetSize.z / sizeZ;

        float finalY = autoScaleHeight ? (ratioY * manualTweaks.y) : manualTweaks.y;

        obj.transform.localScale = new Vector3(
            ratioX * manualTweaks.x,
            finalY,
            ratioZ * manualTweaks.z
        );
    }
}