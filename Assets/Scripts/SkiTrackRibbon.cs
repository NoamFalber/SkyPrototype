using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class SkiTrackRibbon : MonoBehaviour
{
    [Header("Dent shape")]
    public float width = 0.07f;
    public float depth = 0.008f;

    [Header("Placement")]
    public float surfaceLift = 0.002f;

    [Header("UVs")]
    public float uvTiling = 1f;

    [Header("Ends")]
    public bool capEnds = true;

    Mesh _mesh;
    MeshFilter _mf;

    struct TrackPoint
    {
        public Vector3 localPos;
        public Vector3 localN;
        public Vector3 localF;   // ski forward on surface (tangent)
        public float dist;
    }

    readonly List<TrackPoint> _points = new();
    public int PointCount => _points.Count;

    bool _hasOrigin;
    Vector3 _originWorld;

    public void Begin()
    {
        _mf = GetComponent<MeshFilter>();
        _mesh = new Mesh { name = "SkiTrackDent_Local" };
        _mesh.MarkDynamic();
        _mf.sharedMesh = _mesh;

        _points.Clear();
        _hasOrigin = false;
        _originWorld = Vector3.zero;

        transform.rotation = Quaternion.identity;
        transform.localScale = Vector3.one;
    }

    public void AddPoint(Vector3 worldPos, Vector3 worldNormal, Vector3 worldForwardOnSurface)
    {
        if (!_hasOrigin)
        {
            _hasOrigin = true;
            _originWorld = worldPos;

            transform.position = _originWorld;
            transform.rotation = Quaternion.identity;
            transform.localScale = Vector3.one;
        }

        Vector3 localPos = worldPos - _originWorld;
        Vector3 localN = worldNormal.normalized;

        Vector3 f = Vector3.ProjectOnPlane(worldForwardOnSurface, worldNormal);
        Vector3 localF = (f.sqrMagnitude > 0.000001f ? f.normalized : Vector3.forward);

        float d = 0f;
        if (_points.Count > 0)
            d = _points[^1].dist + Vector3.Distance(_points[^1].localPos, localPos);

        _points.Add(new TrackPoint { localPos = localPos, localN = localN, localF = localF, dist = d });

        if (_points.Count >= 2)
            RebuildMesh();
    }

    void RebuildMesh()
    {
        int count = _points.Count;
        int vertCount = count * 4;

        Vector3[] verts = new Vector3[vertCount];
        Vector3[] normals = new Vector3[vertCount];
        Vector2[] uvs = new Vector2[vertCount];

        int segCount = count - 1;
        int triCount = segCount * 4 * 2 + (capEnds ? 4 : 0);
        int[] tris = new int[triCount * 3];

        for (int i = 0; i < count; i++)
        {
            TrackPoint p = _points[i];

            // Use ski forward (tangent), NOT the path tangent
            Vector3 forward = p.localF.sqrMagnitude > 0.000001f ? p.localF.normalized : Vector3.forward;

            // Side vector across the ski
            Vector3 side = Vector3.Cross(forward, p.localN).normalized;
            if (side.sqrMagnitude < 0.000001f)
                side = Vector3.right;

            Vector3 lift = p.localN * surfaceLift;

            Vector3 leftTop  = p.localPos - side * (width * 0.5f) + lift;
            Vector3 rightTop = p.localPos + side * (width * 0.5f) + lift;

            Vector3 leftBottom  = leftTop  - p.localN * depth;
            Vector3 rightBottom = rightTop - p.localN * depth;

            int v = i * 4;

            verts[v + 0] = leftTop;
            verts[v + 1] = rightTop;
            verts[v + 2] = leftBottom;
            verts[v + 3] = rightBottom;

            normals[v + 0] = p.localN;
            normals[v + 1] = p.localN;
            normals[v + 2] = p.localN;
            normals[v + 3] = p.localN;

            float u = p.dist * uvTiling;
            uvs[v + 0] = new Vector2(0f, u);
            uvs[v + 1] = new Vector2(1f, u);
            uvs[v + 2] = new Vector2(0f, u);
            uvs[v + 3] = new Vector2(1f, u);
        }

        int t = 0;
        for (int i = 0; i < segCount; i++)
        {
            int a = i * 4;
            int b = (i + 1) * 4;

            AddQuad(tris, ref t, a + 0, a + 1, b + 1, b + 0); // top
            AddQuad(tris, ref t, b + 2, b + 3, a + 3, a + 2); // bottom
            AddQuad(tris, ref t, b + 0, a + 0, a + 2, b + 2); // left wall
            AddQuad(tris, ref t, a + 1, b + 1, b + 3, a + 3); // right wall
        }

        if (capEnds && count >= 2)
        {
            int s = 0;
            AddQuad(tris, ref t, s + 1, s + 0, s + 2, s + 3);

            int e = (count - 1) * 4;
            AddQuad(tris, ref t, e + 0, e + 1, e + 3, e + 2);
        }

        _mesh.Clear(false);
        _mesh.vertices = verts;
        _mesh.normals = normals;
        _mesh.uv = uvs;
        _mesh.triangles = tris;
        _mesh.RecalculateBounds();
    }

    static void AddQuad(int[] tris, ref int t, int a, int b, int c, int d)
    {
        tris[t++] = a; tris[t++] = b; tris[t++] = c;
        tris[t++] = a; tris[t++] = c; tris[t++] = d;
    }
}
