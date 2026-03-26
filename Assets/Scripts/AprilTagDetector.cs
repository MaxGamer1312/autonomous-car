using AprilTag;
using UnityEngine;
using System.Collections.Generic;

public class AprilTagDetector : MonoBehaviour
{
    WebCamTexture _webcam;
    TagDetector _detector;
    private bool mapLocked;
    private Dictionary<int, List<Vector3>> boundaryTags;

    // --- ENHANCED STABILITY STORAGE ---
    private Dictionary<int, float> _activeSnappedAngles = new Dictionary<int, float>();
    private Dictionary<int, float> _pendingAngles = new Dictionary<int, float>();
    private Dictionary<int, float> _confirmationTimers = new Dictionary<int, float>();

    [SerializeField] private bool debug;

    [Header("Map Calibration")]
    public Transform mapObject;
    public float realToSimScale = 1.0f;

    [Header("Target Tags")]
    public int corner1ID = 0;
    public int corner2ID = 1;

    [Header("Grid & Snapping")]
    public int gridColumnsN = 3;
    public int gridRowsM = 3;
    public TagObjectLinker objectLinker;

    [Header("Movement Inversion")]
    [Tooltip("Check this if moving the tag Left/Right is backwards in Unity.")]
    public bool invertXAxis = false;
    [Tooltip("Check this if moving the tag Up/Down (Forward/Backward) is backwards in Unity.")]
    public bool invertZAxis = true;

    [Header("Rotation Tuning")]
    [Tooltip("How wide the 'Capture Cone' is in degrees. 30 means +/- 15 degrees from the target. Outside of this, movement is ignored.")]
    [Range(10, 80)]
    public float coneAngle = 30f;

    [Tooltip("Seconds the tag must remain inside a new cone before the road actually rotates. Kills 1-frame camera glitches.")]
    public float rotationStabilityTime = 0.25f;

    [Tooltip("If 'Up' reads as 85 degrees in the debug log, put -85 here to force it to 0.")]
    public float globalRotationFix = 0f;

    private Vector3 _gizmoCenter;
    private Vector3 _gizmoSize;

    [Header("Camera Settings")]
    [Tooltip("Type the exact name of your phone camera here. Leave blank to use the default laptop camera.")]
    public string targetCameraName = "";

    void Start()
    {
        boundaryTags = new Dictionary<int, List<Vector3>>();

        // 1. List all cameras in the console so you can copy the exact name
        WebCamDevice[] devices = WebCamTexture.devices;
        for (int i = 0; i < devices.Length; i++)
        {
            Debug.Log($"Camera {i}: {devices[i].name}");
        }

        // 2. Select the right camera
        if (devices.Length > 0)
        {
            if (string.IsNullOrEmpty(targetCameraName))
            {
                _webcam = new WebCamTexture(devices[0].name);
            }
            else
            {
                _webcam = new WebCamTexture(targetCameraName);
            }

            _webcam.Play();
            _detector = new TagDetector(_webcam.width, _webcam.height, 2);
            if (debug) Debug.Log($"Initialized Detector with Camera: {_webcam.deviceName}");
        }
        else
        {
            Debug.LogError("No cameras found!");
        }
    }

    void Update()
    {
        if (_webcam == null || !_webcam.isPlaying) return;
        _detector.ProcessImage(_webcam.GetPixels32(), 60.0f, 0.166f);

        if (!mapLocked) { HandleMapCalibration(); }
        else { HandleContinuousTracking(); }
    }

    private void HandleContinuousTracking()
    {
        float cellW = _gizmoSize.x / (gridColumnsN > 0 ? gridColumnsN : 1);
        float cellD = _gizmoSize.z / (gridRowsM > 0 ? gridRowsM : 1);
        Vector3 cellSize = new Vector3(cellW, 1f, cellD);

        foreach (var tag in _detector.DetectedTags)
        {
            if (tag.ID == corner1ID || tag.ID == corner2ID) continue;

            Vector3 rawPos = tag.Position * realToSimScale;
            Vector3 snappedPos = GetSnappedPosition(rawPos);

            List<Vector3> corners = calculateFourCorners(tag);
            Quaternion snappedRot = GetAbsoluteConeRotation(tag.ID, corners);

            if (objectLinker != null)
            {
                objectLinker.UpdateObjectPose(tag.ID, snappedPos, snappedRot, cellSize);
            }
        }
    }

    private Quaternion GetAbsoluteConeRotation(int id, List<Vector3> corners)
    {
        Vector3 topEdgeCenter = (corners[0] + corners[1]) / 2f;
        Vector3 bottomEdgeCenter = (corners[3] + corners[2]) / 2f;

        Vector3 tagDirection = topEdgeCenter - bottomEdgeCenter;
        tagDirection.y = 0;

        if (tagDirection.sqrMagnitude < 0.001f) return Quaternion.identity;

        float rawAngle = Mathf.Atan2(tagDirection.x, tagDirection.z) * Mathf.Rad2Deg;
        rawAngle += globalRotationFix;
        rawAngle = Mathf.Repeat(rawAngle, 360f);

        float nearest90 = Mathf.Round(rawAngle / 90f) * 90f;
        nearest90 = Mathf.Repeat(nearest90, 360f);

        if (!_activeSnappedAngles.ContainsKey(id))
        {
            _activeSnappedAngles[id] = nearest90;
            _pendingAngles[id] = nearest90;
        }

        float distToNearest = Mathf.Abs(Mathf.DeltaAngle(rawAngle, nearest90));
        bool insideCone = distToNearest <= (coneAngle / 2f);

        if (insideCone && nearest90 != _activeSnappedAngles[id])
        {
            if (_pendingAngles[id] != nearest90)
            {
                _pendingAngles[id] = nearest90;
                _confirmationTimers[id] = Time.time;
            }

            if (Time.time - _confirmationTimers[id] >= rotationStabilityTime)
            {
                _activeSnappedAngles[id] = nearest90;
            }
        }
        else if (!insideCone)
        {
            _pendingAngles[id] = _activeSnappedAngles[id];
        }

        if (debug)
        {
            string status = insideCone ? $"<color=green>IN CONE ({nearest90})</color>" : "<color=red>DEADZONE</color>";
            Debug.Log($"[Tag {id}] Raw: {rawAngle:F1}° | Dist to Target: {distToNearest:F1}° | {status}");
        }

        return Quaternion.Euler(0, _activeSnappedAngles[id], 0);
    }

    private void HandleMapCalibration()
    {
        foreach (var tag in _detector.DetectedTags)
        {
            if (tag.ID == corner1ID || tag.ID == corner2ID)
                boundaryTags[tag.ID] = calculateFourCorners(tag);
        }

        if (boundaryTags.ContainsKey(corner1ID) && boundaryTags.ContainsKey(corner2ID))
        {
            Vector3 p1 = boundaryTags[corner1ID][0] * realToSimScale;
            Vector3 p2 = boundaryTags[corner2ID][0] * realToSimScale;
            float minX = Mathf.Min(p1.x, p2.x); float maxX = Mathf.Max(p1.x, p2.x);
            float minZ = Mathf.Min(p1.y, p2.y); float maxZ = Mathf.Max(p1.y, p2.y);
            _gizmoCenter = new Vector3((minX + maxX) / 2f, 0f, (minZ + maxZ) / 2f);
            _gizmoSize = new Vector3(maxX - minX, 0.1f, maxZ - minZ);
            if (mapObject != null) { mapObject.position = new Vector3(_gizmoCenter.x, -0.1f, _gizmoCenter.z); mapObject.localScale = _gizmoSize; }
            mapLocked = true;
        }
    }

    private Vector3 GetSnappedPosition(Vector3 rawPos)
    {
        float cellW = _gizmoSize.x / gridColumnsN;
        float cellD = _gizmoSize.z / gridRowsM;
        float minX = _gizmoCenter.x - (_gizmoSize.x / 2f);
        float minZ = _gizmoCenter.z - (_gizmoSize.z / 2f);

        int col = Mathf.Clamp(Mathf.FloorToInt((rawPos.x - minX) / cellW), 0, gridColumnsN - 1);
        int row = Mathf.Clamp(Mathf.FloorToInt((rawPos.y - minZ) / cellD), 0, gridRowsM - 1);

        // --- THE INVERSION FIX ---
        // By subtracting the current column/row from the maximum possible column/row, 
        // it instantly mirrors the position to the exact opposite side of the grid.
        if (invertXAxis) col = (gridColumnsN - 1) - col;
        if (invertZAxis) row = (gridRowsM - 1) - row;

        return new Vector3(minX + (col * cellW) + (cellW / 2f), 0f, minZ + (row * cellD) + (cellD / 2f));
    }

    List<Vector3> calculateFourCorners(AprilTag.TagPose tag)
    {
        float h = 0.166f / 2f; Vector3 c = tag.Position; Quaternion r = tag.Rotation;
        return new List<Vector3> { c + (r * new Vector3(-h, h, 0)), c + (r * new Vector3(h, h, 0)), c + (r * new Vector3(h, -h, 0)), c + (r * new Vector3(-h, -h, 0)) };
    }
}