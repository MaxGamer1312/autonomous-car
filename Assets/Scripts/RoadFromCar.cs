using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Video;

public class RoadFromCamera : MonoBehaviour
{
    public enum SourceType { Webcam, Image, Video }


    [Header("Source")]
    public SourceType source = SourceType.Webcam;

    // Webcam
    public bool useFrontFacing = false;
    public int webcamWidth = 640;
    public int webcamHeight = 480;
    public int webcamFPS = 30;

    [Header("Image Source")]
    [Tooltip("Static image to sample when Source = Image")] public Texture2D staticImage;

    [Header("Video Source")] 
    [Tooltip("Optional: Assign a VideoPlayer. If null, one will be added at runtime.")] public VideoPlayer videoPlayer;
    [Tooltip("RenderTexture that receives video frames (Source = Video). If null, one is created at runtime when the video prepares.")] public RenderTexture videoTargetRT;
    [Tooltip("Optional URL/path for the VideoPlayer if you want to load by URL.")] public string videoURL;
    [Tooltip("Loop the video when Source = Video")] public bool loopVideo = true;
    [Tooltip("Play automatically when ready")] public bool playVideoOnStart = true;

    [Header("Webcam Rect (from this GameObject)")]
    [Min(0.01f)] public float rectWidth = 10f;
    [Min(0.01f)] public float rectHeight = 10f;

    [Header("Road Detection")]
    public bool useLuminanceOnly = true;
    [Tooltip("If true, invert detection so bright/light areas are considered road instead of dark.")]
    public bool invertDetection = false;
    [Range(0f, 1f)] public float roadThreshold = 0.2f;
    [Range(0f, 1f)] public float rgbCutoff = 0.2f;

    [Header("Road Settings")]
    public bool continuous = true;
    [Min(0.01f)] public float cubeSize = 0.1f;
    [Min(1)] public int stride = 2;
    public int maxInstancesPerFrame = 0;

    [Header("Rendering (Instanced)")]
    public Material roadMaterial;
    public bool castShadows = false;
    public bool receiveShadows = false;
    [Range(1f, 60f)] public float commitRateHz = 15f;

    [Header("Z-Fight Controls")]
    [Range(0.005f, 0.2f)] public float tileThicknessFrac = 0.05f;
    public float yNudge = 0.01f;

    [Header("Debug")]
    public bool verboseLogs = false;

    [Header("Physics Collider")]
    [Range(0.1f, 20f)] public float instancedColliderUpdateRate = 2f;
    [Tooltip("Sampling stride for collider (>=1). 1 = every tile, 2 = every other tile, etc.")]
    [Min(1)] public int colliderStride = 1;
    public bool colliderConvex = false;

    private Texture2D readbackTex;
    private WebCamTexture webcam;
    private int width, height;
    private bool builtOnce = false;
    private Color32[] pixels32;

    private Vector3 _initialPos;
    private Quaternion _initialRot;
    private Vector3 _initialScale;
    private Rigidbody _rb;


    private static readonly int kBatch = 1023;
    private Mesh cubeMesh;
    private readonly List<Matrix4x4> matrices = new List<Matrix4x4>(kBatch);
    private readonly List<Matrix4x4> _batchMatrices = new List<Matrix4x4>(kBatch);


    private MeshCollider _combinedMC;
    private Mesh _combinedMesh;
    private float _nextColliderTimeInst = 0f;

    void Awake()
    {
        var tempCube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cubeMesh = tempCube.GetComponent<MeshFilter>().sharedMesh;
        DestroyImmediate(tempCube);

        if (roadMaterial == null)
            roadMaterial = new Material(Shader.Find("Universal Render Pipeline/Lit"));
        if (roadMaterial != null) roadMaterial.enableInstancing = true;
    }

    void Start()
    {
        _initialPos = transform.position;
        _initialRot = transform.rotation;
        _initialScale = transform.localScale;
        _rb = GetComponent<Rigidbody>();

        if (source == SourceType.Webcam)
        {
            var devices = WebCamTexture.devices;
            if (devices.Length == 0)
            {
                Debug.LogError("No webcams found.");
                enabled = false; return;
            }
            int index = 0;
            if (useFrontFacing)
            {
                for (int i = 0; i < devices.Length; i++)
                    if (devices[i].isFrontFacing) { index = i; break; }
            }
            webcam = new WebCamTexture(devices[index].name, webcamWidth, webcamHeight, webcamFPS);
            webcam.Play();

            width = height = 2;
            readbackTex = new Texture2D(width, height, TextureFormat.RGB24, false);
            pixels32 = new Color32[width * height];
        }
        else if (source == SourceType.Image)
        {
            if (!staticImage)
            {
                Debug.LogError("Assign a Texture2D to 'staticImage' for Image source.");
                enabled = false; return;
            }
            width = staticImage.width;
            height = staticImage.height;
            readbackTex = new Texture2D(width, height, TextureFormat.RGB24, false);
            pixels32 = new Color32[width * height];
        }
        else
        {
            if (!videoPlayer)
            {
                videoPlayer = gameObject.GetComponent<VideoPlayer>();
                if (!videoPlayer) videoPlayer = gameObject.AddComponent<VideoPlayer>();
            }
            videoPlayer.isLooping = loopVideo;
            if (!string.IsNullOrEmpty(videoURL)) videoPlayer.url = videoURL;
            videoPlayer.renderMode = VideoRenderMode.RenderTexture;
            videoPlayer.Prepare();
            if (playVideoOnStart) videoPlayer.playOnAwake = true;

            width = height = 2;
            readbackTex = new Texture2D(width, height, TextureFormat.RGB24, false);
            pixels32 = new Color32[width * height];
        }

        _nextColliderTimeInst = Time.time + (1f / Mathf.Max(0.1f, instancedColliderUpdateRate));
    }

    void Update()
    {
        if (!continuous && builtOnce)
        {
            DrawMatricesBatched(matrices);
        }
        else
        {
            AcquirePixels();
            matrices.Clear();
            BuildMatrices(matrices);
            DrawMatricesBatched(matrices);
            builtOnce = true;
        }

        if (Time.time >= _nextColliderTimeInst)
        {
            RebuildColliderFromMatrices(matrices);
            _nextColliderTimeInst = Time.time + 1f / Mathf.Max(0.1f, instancedColliderUpdateRate);
        }
    }

    void AcquirePixels()
    {
        if (source == SourceType.Webcam)
        {
            if (!webcam || !webcam.isPlaying || !webcam.didUpdateThisFrame) return;
            int w = webcam.width, h = webcam.height;
            if (w <= 16 || h <= 16) return;
            if (readbackTex.width != w || readbackTex.height != h)
            {
                readbackTex.Reinitialize(w, h);
                pixels32 = new Color32[w * h];
            }
            var frame = webcam.GetPixels32();
            readbackTex.SetPixels32(frame); readbackTex.Apply(false);
            pixels32 = frame;
            width = w; height = h;
        }
        else if (source == SourceType.Image)
        {
            if (!staticImage) return;
            int w = staticImage.width, h = staticImage.height;
            if (readbackTex.width != w || readbackTex.height != h)
            {
                readbackTex.Reinitialize(w, h);
                pixels32 = new Color32[w * h];
            }
            var frame = staticImage.GetPixels32();
            readbackTex.SetPixels32(frame); readbackTex.Apply(false);
            pixels32 = frame;
            width = w; height = h;
        }
        else
        {
            if (!videoPlayer) return;
            if (!videoPlayer.isPrepared)
            {
                videoPlayer.Prepare();
                return;
            }
            int vw = (int)Mathf.Max(2, videoPlayer.width);
            int vh = (int)Mathf.Max(2, videoPlayer.height);
            if (!videoTargetRT || videoTargetRT.width != vw || videoTargetRT.height != vh)
            {
                if (videoTargetRT) videoTargetRT.Release();
                videoTargetRT = new RenderTexture(vw, vh, 0, RenderTextureFormat.ARGB32) { name = "VideoTargetRT", useMipMap = false, autoGenerateMips = false };
                videoPlayer.targetTexture = videoTargetRT;
            }
            if (playVideoOnStart && !videoPlayer.isPlaying) videoPlayer.Play();

            var prev = RenderTexture.active;
            RenderTexture.active = videoTargetRT;
            if (readbackTex.width != videoTargetRT.width || readbackTex.height != videoTargetRT.height)
            {
                readbackTex.Reinitialize(videoTargetRT.width, videoTargetRT.height);
                pixels32 = new Color32[videoTargetRT.width * videoTargetRT.height];
            }
            readbackTex.ReadPixels(new Rect(0, 0, videoTargetRT.width, videoTargetRT.height), 0, 0);
            readbackTex.Apply(false);
            RenderTexture.active = prev;
            pixels32 = readbackTex.GetPixels32();
            width = videoTargetRT.width; height = videoTargetRT.height;
        }
    }

    void BuildMatrices(List<Matrix4x4> outList)
    {
        float thickness = Mathf.Max(0.001f, cubeSize * tileThicknessFrac);
        Vector3 tileScale = new Vector3(cubeSize * 0.98f, thickness, cubeSize * 0.98f);
        Vector3 A = Vector3.zero, xAxis = Vector3.zero, zAxis = Vector3.zero;

        if (source == SourceType.Webcam || source == SourceType.Image || source == SourceType.Video)
        {
            Vector3 center = transform.position;
            Vector3 right = transform.right * rectWidth;
            Vector3 forward = transform.forward * rectHeight;
            A = center - 0.5f * right - 0.5f * forward;
            xAxis = right;
            zAxis = forward;
        }
        float baseY = transform.position.y;
        float centerY = baseY + yNudge;

        for (int y = 0; y < height; y += stride)
        {
            int row = y * width;
            for (int x = 0; x < width; x += stride)
            {
                Color32 c = pixels32[row + x];
                float luma = (0.2126f * c.r + 0.7152f * c.g + 0.0722f * c.b) / 255f;
                bool isRoad;
                if (useLuminanceOnly)
                    isRoad = invertDetection ? (luma >= 1f - roadThreshold) : (luma <= roadThreshold);
                else
                    isRoad = invertDetection
                        ? (c.r / 255f >= 1f - rgbCutoff && c.g / 255f >= 1f - rgbCutoff && c.b / 255f >= 1f - rgbCutoff)
                        : (c.r / 255f <= rgbCutoff && c.g / 255f <= rgbCutoff && c.b / 255f <= rgbCutoff);
                if (!isRoad) continue;

                Vector3 pos;
                float u = (float)x / Mathf.Max(1, width - 1);
                float v = (float)y / Mathf.Max(1, height - 1);
                pos = A + u * xAxis + v * zAxis;
                pos.y = centerY;
                outList.Add(Matrix4x4.TRS(pos, Quaternion.identity, tileScale));
            }
        }
    }


    void DrawMatricesBatched(List<Matrix4x4> src)
    {
        for (int i = 0; i < src.Count; i += kBatch)
        {
            int count = Mathf.Min(kBatch, src.Count - i);
            _batchMatrices.Clear();
            for (int j = 0; j < count; j++) _batchMatrices.Add(src[i + j]);

            Graphics.DrawMeshInstanced(
                cubeMesh, 0, roadMaterial, _batchMatrices, null,
                castShadows ? UnityEngine.Rendering.ShadowCastingMode.On : UnityEngine.Rendering.ShadowCastingMode.Off,
                receiveShadows
            );
        }
    }


    Vector3 PixelToWorld_LerpOnRectangle(int x, int y)
    {
        float u = (float)x / Mathf.Max(1, width - 1);
        float v = (float)y / Mathf.Max(1, height - 1);
        Vector3 center = transform.position;
        Vector3 right = transform.right * rectWidth;
        Vector3 forward = transform.forward * rectHeight;
        Vector3 A = center - 0.5f * right - 0.5f * forward;
        return A + u * right + v * forward;
    }

    void RebuildCombinedColliderFromChildren()
    {
        var filters = GetComponentsInChildren<MeshFilter>();
        if (filters == null || filters.Length == 0)
        {
            DropColliderAssets();
            return;
        }

        var combines = new CombineInstance[filters.Length];
        for (int i = 0; i < filters.Length; i++)
        {
            var mf = filters[i];
            if (mf.sharedMesh == null) { combines[i].mesh = null; continue; }
            combines[i].mesh = mf.sharedMesh;
            combines[i].transform = transform.worldToLocalMatrix * mf.transform.localToWorldMatrix;
        }

        BuildOrAssignCombinedMesh(combines);
    }

    void RebuildColliderFromMatrices(List<Matrix4x4> src)
    {
        if (src == null || src.Count == 0 || cubeMesh == null)
        {
            DropColliderAssets();
            return;
        }

        int count = (colliderStride <= 1) ? src.Count : (src.Count + colliderStride - 1) / colliderStride;
        var combines = new List<CombineInstance>(count);

        for (int i = 0; i < src.Count; i += colliderStride)
        {
            var ci = new CombineInstance
            {
                mesh = cubeMesh,
                transform = transform.worldToLocalMatrix * src[i]
            };
            combines.Add(ci);
        }

        BuildOrAssignCombinedMesh(combines.ToArray());
    }

    void BuildOrAssignCombinedMesh(CombineInstance[] combines)
    {
        if (_combinedMesh == null)
        {
            _combinedMesh = new Mesh { name = "RoadCombinedColliderMesh" };
            _combinedMesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        }
        else
        {
            _combinedMesh.Clear();
        }

        _combinedMesh.CombineMeshes(combines, true, true, false);

        if (_combinedMC == null)
        {
            _combinedMC = gameObject.GetComponent<MeshCollider>();
            if (_combinedMC == null) _combinedMC = gameObject.AddComponent<MeshCollider>();
        }
        _combinedMC.sharedMesh = null;
        _combinedMC.convex = colliderConvex;
        _combinedMC.sharedMesh = _combinedMesh;
    }

    void DropColliderAssets()
    {
        bool useImmediate = !Application.isPlaying;
        if (_combinedMC)
        {
            _combinedMC.sharedMesh = null;
            if (useImmediate) DestroyImmediate(_combinedMC); else Destroy(_combinedMC);
            _combinedMC = null;
        }
        if (_combinedMesh)
        {
            if (useImmediate) DestroyImmediate(_combinedMesh); else Destroy(_combinedMesh);
            _combinedMesh = null;
        }
    }

#if UNITY_EDITOR
    void OnDrawGizmosSelected()
    {
        if (!enabled) return;
        float thickness = Mathf.Max(0.001f, cubeSize * tileThicknessFrac);
        float baseY = transform.position.y;
        float centerY = baseY + yNudge;

        Vector3 center = transform.position; center.y = centerY;
        Vector3 right = transform.right * rectWidth;
        Vector3 fwd = transform.forward * rectHeight;
        Vector3 bl = center - 0.5f * right - 0.5f * fwd;
        Vector3 br = bl + right;
        Vector3 tl = bl + fwd;
        Vector3 tr = bl + right + fwd;

        Gizmos.color = new Color(0, 1, 1, 0.35f);
        Gizmos.DrawLine(bl, br); Gizmos.DrawLine(br, tr); Gizmos.DrawLine(tr, tl); Gizmos.DrawLine(tl, bl);
        Gizmos.DrawSphere(center, 0.03f * Mathf.Max(1f, Mathf.Max(rectWidth, rectHeight)) * 0.05f);
    }
#endif

    void OnDisable()
    {
        DropColliderAssets();
    }

}
