
using UnityEngine;
using System.Collections.Generic;

public class RPLidarVisualizer : MonoBehaviour
{
    public enum VisualizationMode
    {
        RawPoints,     // Instantiate small prefabs for each point (simple, easy to debug)
        LineRenderer,  // Connect points into a ring/contour
        MeshPoints,    // Build a CPU mesh of tiny quads for each point
        GPUShader      // Upload points to GPU (ComputeBuffer) and draw procedurally
    }

    [Header("Mode")]
    public VisualizationMode mode = VisualizationMode.RawPoints;

    [Header("Common settings")]
    [Tooltip("Scale factor for distances. If server sends mm, use 0.001f; if meters, use 1f.")]
    public float distanceScale = 0.001f;

    [Tooltip("Ignore points closer than this (meters after scaling)")]
    public float minDistance = 0.02f;

    [Tooltip("Ignore points farther than this (meters after scaling). Set <=0 to disable.")]
    public float maxDistance = 0f;

    [Tooltip("Downsample factor (process every Nth point). Use 1 for full resolution.")]
    public int downsample = 1;

    [Header("Raw points (prefab pooling)")]
    public GameObject pointPrefab;         // Assign a tiny sphere/quad (scale ~0.02)
    public int maxRawPoints = 2000;        // Pool size

    [Header("Line Renderer")]
    public LineRenderer lineRenderer;      // Assign a LineRenderer component
    public float lineWidth = 0.02f;

    [Header("Mesh point cloud (CPU quads)")]
    public Material meshMaterial;          // Any simple unlit material
    public float pointQuadSize = 0.02f;    // Size of each quad (meters)

    [Header("GPU shader (fast)")]
    public Material gpuMaterial;           // Material that uses the RPLidarPoints shader
    public float gpuPointSize = 4.0f;      // Pixel size (shader-controlled)

    // Internal state
    private readonly List<GameObject> pool = new List<GameObject>();
    private Mesh mesh;
    private Vector3[] positionsBuf = new Vector3[0];

    // GPU buffer
    private ComputeBuffer gpuPositions;
    private int gpuCount = 0;

    private void Awake()
    {
        // Prepare pool for RawPoints
        if (pointPrefab != null && pool.Count == 0 && maxRawPoints > 0)
        {
            for (int i = 0; i < maxRawPoints; i++)
            {
                var go = Instantiate(pointPrefab, transform);
                go.SetActive(false);
                pool.Add(go);
            }
        }

        // LineRenderer setup
        if (lineRenderer != null)
        {
            lineRenderer.useWorldSpace = false; // draw in local space of this object
            lineRenderer.loop = true;
            lineRenderer.widthMultiplier = lineWidth;
        }

        // Mesh init
        mesh = new Mesh();
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32; // allow >65k indices if needed
    }

    private void OnDestroy()
    {
        if (gpuPositions != null)
        {
            gpuPositions.Dispose();
            gpuPositions = null;
        }
    }

    /// <summary>
    /// Entry point called by the WebSocket client.
    /// anglesDeg: degrees (0..360). distances: linear units to be scaled by distanceScale.
    /// </summary>
    public void ShowScan(float[] anglesDeg, float[] distances)
    {
        if (anglesDeg == null || distances == null || anglesDeg.Length != distances.Length)
            return;

        // Preprocess: scale, filter, and downsample
        var pts = CollectPoints(anglesDeg, distances);

        switch (mode)
        {
            case VisualizationMode.RawPoints:
                RenderRawPoints(pts);
                break;

            case VisualizationMode.LineRenderer:
                RenderLine(pts);
                break;

            case VisualizationMode.MeshPoints:
                RenderMeshPoints(pts);
                break;

            case VisualizationMode.GPUShader:
                RenderGpuPoints(pts);
                break;
        }
    }

    // Convert polar to local XY (top‑down) and apply filters/downsample
    private List<Vector3> CollectPoints(float[] anglesDeg, float[] distances)
    {
        int n = anglesDeg.Length;
        var pts = new List<Vector3>(n);
        float maxD = (maxDistance > 0f) ? maxDistance : float.MaxValue;

        for (int i = 0; i < n; i += (downsample <= 0 ? 1 : downsample))
        {
            float d = distances[i] * distanceScale;
            if (d < minDistance || d > maxD) continue;

            float a = anglesDeg[i] * Mathf.Deg2Rad;
            float x = Mathf.Cos(a) * d;
            float y = Mathf.Sin(a) * d;

            // We draw in XZ plane for top‑down view (Y=0)
            pts.Add(new Vector3(x, 0f, y));
        }
        return pts;
    }

    // Mode 1: instantiate pooled prefabs
    private void RenderRawPoints(List<Vector3> pts)
    {
        int count = Mathf.Min(pts.Count, pool.Count);

        for (int i = 0; i < count; i++)
        {
            pool[i].SetActive(true);
            pool[i].transform.localPosition = pts[i];
        }

        // Disable unused pooled objects
        for (int i = count; i < pool.Count; i++)
        {
            pool[i].SetActive(false);
        }
    }

    // Mode 2: line renderer (closed loop)
    private void RenderLine(List<Vector3> pts)
    {
        if (lineRenderer == null)
            return;

        // Ensure buffer size
        if (positionsBuf.Length != pts.Count)
            positionsBuf = new Vector3[pts.Count];

        for (int i = 0; i < pts.Count; i++)
            positionsBuf[i] = pts[i];

        lineRenderer.widthMultiplier = lineWidth;
        lineRenderer.loop = true;
        lineRenderer.positionCount = positionsBuf.Length;
        if (positionsBuf.Length > 0)
            lineRenderer.SetPositions(positionsBuf);
    }

    // Mode 3: CPU mesh of tiny quads (billboards aligned to XZ plane)
    private void RenderMeshPoints(List<Vector3> pts)
    {
        if (meshMaterial == null) return;

        int count = pts.Count;
        if (count == 0)
        {
            mesh.Clear();
            return;
        }

        // Each point → 4 vertices (quad) + 6 indices
        int vCount = count * 4;
        int iCount = count * 6;

        var verts = new Vector3[vCount];
        var uvs = new Vector2[vCount];
        var indices = new int[iCount];

        float s = pointQuadSize * 0.5f;

        for (int p = 0; p < count; p++)
        {
            Vector3 c = pts[p];
            int vBase = p * 4;
            int iBase = p * 6;

            // Quad in XZ plane around c
            verts[vBase + 0] = new Vector3(c.x - s, 0f, c.z - s);
            verts[vBase + 1] = new Vector3(c.x - s, 0f, c.z + s);
            verts[vBase + 2] = new Vector3(c.x + s, 0f, c.z + s);
            verts[vBase + 3] = new Vector3(c.x + s, 0f, c.z - s);

            uvs[vBase + 0] = new Vector2(0, 0);
            uvs[vBase + 1] = new Vector2(0, 1);
            uvs[vBase + 2] = new Vector2(1, 1);
            uvs[vBase + 3] = new Vector2(1, 0);

            // Two triangles
            indices[iBase + 0] = vBase + 0;
            indices[iBase + 1] = vBase + 1;
            indices[iBase + 2] = vBase + 2;
            indices[iBase + 3] = vBase + 0;
            indices[iBase + 4] = vBase + 2;
            indices[iBase + 5] = vBase + 3;
        }

        mesh.Clear();
        mesh.SetVertices(verts);
        mesh.SetUVs(0, new List<Vector2>(uvs));
        mesh.SetIndices(indices, MeshTopology.Triangles, 0, true);

        Graphics.DrawMesh(mesh, transform.localToWorldMatrix, meshMaterial, gameObject.layer);
    }

    // Mode 4: GPU draw (points via ComputeBuffer + DrawProcedural)
    private void RenderGpuPoints(List<Vector3> pts)
    {
        if (gpuMaterial == null)
            return;

        int count = pts.Count;
        if (count == 0)
        {
            gpuCount = 0;
            return;
        }

        // Allocate/resize ComputeBuffer (Vector3 == 12 bytes)
        if (gpuPositions == null || gpuCount != count)
        {
            if (gpuPositions != null) gpuPositions.Dispose();
            gpuPositions = new ComputeBuffer(count, sizeof(float) * 3);
            gpuCount = count;
        }

        gpuPositions.SetData(pts);

        // Feed buffer to the material
        gpuMaterial.SetBuffer("_Positions", gpuPositions);
        gpuMaterial.SetFloat("_PointSize", gpuPointSize);

        // Draw in local space of this object
        // We use Procedural with MeshTopology.Points; shader expands to quads.
        var matrix = transform.localToWorldMatrix;
        Graphics.DrawProcedural(gpuMaterial, new Bounds(transform.position, Vector3.one * 1000f),
                                MeshTopology.Points, gpuCount, 1, null, null, UnityEngine.Rendering.ShadowCastingMode.Off, false);
    }
}