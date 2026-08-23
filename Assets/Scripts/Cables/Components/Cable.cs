using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;

[ExecuteAlways]
[RequireComponent(typeof(MeshFilter))]
public class Cable : MonoBehaviour
{
    // Visuals.
    [SerializeField]
    [Range(1, 250)]
    private int resolution = 100;

    [SerializeField]
    [Range(3, 25)]
    private int sides = 12;

    [SerializeField]
    [Range(0.001f, 0.1f)]
    private float radius = 0.01f;

    // Segments.
    [SerializeField]
    private List<CableSegment> segments = new();

    // Physics.
    [SerializeField]
    [Range(1, 200)]
    private int simulationSubsteps = 20;

    // Control Points.
    [SerializeField]
    private Transform pointsContainer;

    private List<CablePoint> points;

    private ISpline spline;
    private List<float3> previousPointPositions;

    private float3[] positions;
    private float3[] tangents;

    private MeshFilter meshFilter;
    private TubeMeshBuilder meshBuilder;

    private CablePhysics physics;

    // Guards against re-entering Refresh()/Initialize() while already inside one.
    private bool isRefreshing;

    private void Awake()
    {
        Refresh();
    }

    private void OnEnable()
    {
        Refresh();
    }

    private void OnTransformChildrenChanged()
    {
        Refresh();
    }

    private void OnValidate()
    {
        Refresh();
    }

    private void OnDrawGizmos()
    {
        if (points == null) return;

        foreach (CablePoint p in points)
        {
            if (!p) continue;
            Gizmos.DrawSphere(p.transform.position, radius * 2f);
        }
    }

    private void Update()
    {
        if (points == null || previousPointPositions == null || physics == null)
            return;

        // Runtime.
        if (Application.isPlaying)
        {
            if (PointsMoved())
            {
                spline.SetPoints(GetPointPositions());
                CalculatePositionsAndTangentsFromSpline();
                
                physics.SetPinnedNodes(SampleSplineWorldPositions());
            }

            for (int i = 0; i < simulationSubsteps; i++)
            {
                physics.SimulatePhysics(Time.deltaTime / simulationSubsteps);
            }

            BuildMeshFromPhysics();
        }
        // In editor.
        else
        {
            if (PointsChanged())
            {
                Refresh();
                return;
            }

            if (PointsMoved())
            {
                CalculatePositionsAndTangentsFromSpline();
                BuildMesh();
            }
        }
    }

    private void Refresh()
    {
        if (isRefreshing) return;
        if (!pointsContainer) return;

        isRefreshing = true;
        try
        {
            Initialize();
            BuildMesh();
        }
        finally
        {
            isRefreshing = false;
        }
    }

    void Initialize()
    {
        // Find Points.
        PopulatePoints();

        // Get Control Point positions.
        previousPointPositions = GetPointPositions().ToList();

        // Get points from spline.
        spline = new CatmullRomSpline();
        spline.SetPoints(GetPointPositions());

        // Rendering.
        meshFilter = GetComponent<MeshFilter>();
        meshBuilder = new TubeMeshBuilder(sides, radius);

        // Physics.
        physics = new CablePhysics(SampleSplineWorldPositions(), radius, 0f, 0.3f);
        ConfigurePhysicsAnchors();
        ConfigurePinnedNodes();

        // Spline.
        positions = new float3[resolution];
        tangents = new float3[resolution];

        CalculatePositionsAndTangentsFromSpline();
    }

    private bool PointsChanged()
    {
        List<CablePoint> actual = GetPoints();

        // Check for points that have been removed.
        List<CablePoint> removed = points.Except(actual).ToList();

        if (removed.Any())
        {
            ValidateSegments(removed);
            return true;
        }

        // Any other count mismatch.
        if (actual.Count != points.Count)
        {
            return true;
        }

        // Same size, check if points were swapped out for different ones.
        for (int i = 0; i < actual.Count; i++)
        {
            if (actual[i] != points[i])
            {
                return true;
            }
        }

        // Nothing's different.
        return false;
    }

    private List<CablePoint> GetPoints()
    {
        return pointsContainer.GetComponentsInChildren<CablePoint>().ToList();
    }

    private void PopulatePoints()
    {
        points = GetPoints();

        if (points.Count == 0)
        {
            InitializePoints();
        }
    }

    // Rebuilds segments to match the surviving points, in their original order.
    // Any pair of points that were already directly connected keeps its existing
    // CableSegment (preserving whatever extra data it carries); any gap left behind
    // by one or more removed points in between gets bridged with a single new segment.
    private void ValidateSegments(List<CablePoint> removed)
    {
        List<CablePoint> survivors = points.Where(p => !removed.Contains(p)).ToList();
        List<CableSegment> rebuilt = new(math.max(survivors.Count - 1, 0));

        for (int i = 0; i < survivors.Count - 1; i++)
        {
            CablePoint a = survivors[i];
            CablePoint b = survivors[i + 1];

            CableSegment existing = segments.Find(s => s.pointA == a && s.pointB == b);
            rebuilt.Add(existing ?? new CableSegment(a, b));
        }

        segments = rebuilt;
    }

    private void InitializePoints()
    {
        // If there is a points container with no points then make two new points
        // for a new segment.
        points = new List<CablePoint>();

        // Create first point.
        AddPoint(transform.position);

        // Create segment.
        CableSegment segment = AddSegment();

        // Nudge point b over from a.
        segment.pointB.transform.position += segment.pointB.transform.forward;
    }

    private float3[] GetPointPositions()
    {
        return points.Select(x => (float3)x.transform.position).ToArray();
    }

    private float3[] SampleSplineWorldPositions()
    {
        float3[] result = new float3[resolution];
        for (int i = 0; i < resolution; i++)
        {
            float t = i / (float)(resolution - 1);
            result[i] = spline.EvaluatePosition(t);
        }
        return result;
    }

    private void CalculatePositionsAndTangentsFromSpline()
    {
        for (int i = 0; i < resolution; i++)
        {
            float t = i / (float)(resolution - 1);

            positions[i] = transform.InverseTransformPoint(spline.EvaluatePosition(t));
            tangents[i] = transform.InverseTransformDirection(spline.EvaluateTangent(t));
        }
    }

    private void BuildMeshFromPhysics()
    {
        float3[] worldPositions = physics.positions;

        for (int i = 0; i < resolution; i++)
        {
            positions[i] = transform.InverseTransformPoint(worldPositions[i]);

            // Physics-driven shape no longer matches the spline, so derive tangents from
            // the simulated chain itself via finite differences instead of the spline.
            float3 prev = worldPositions[math.max(i - 1, 0)];
            float3 next = worldPositions[math.min(i + 1, resolution - 1)];
            float3 dir = next - prev;

            float3 worldTangent = math.lengthsq(dir) > 1e-12f
                ? math.normalize(dir)
                : new float3(0, 0, 1);

            tangents[i] = transform.InverseTransformDirection(worldTangent);
        }

        BuildMesh();
    }

    private void BuildMesh()
    {
        meshFilter.mesh = meshBuilder.Build(positions, tangents);
    }

    private bool PointsMoved()
    {
        bool changed = false;

        for (int i = 0; i < points.Count; i++)
        {
            if (!points[i]) continue;

            float3 position = points[i].transform.position;

            // New point was added.
            if (i == previousPointPositions.Count)
            {
                previousPointPositions.Add(position);
                spline.SetPoint(i, position);
                changed = true;
            }
            // Point position changed.
            else if (!math.all(position == previousPointPositions[i]))
            {
                previousPointPositions[i] = position;
                spline.SetPoint(i, position);
                changed = true;
            }
        }

        return changed;
    }

    private void ConfigurePhysicsAnchors()
    {
        // Pin certain nodes of the cable to the cable points via physics anchors.
        for (int i = 0; i < points.Count; i++)
        {
            if (!points[i].isAnchor) continue;

            float t = points.Count > 1 ? i / (float)(points.Count - 1) : 0f;
            int nodeIndex = Mathf.RoundToInt(t * (resolution - 1));

            physics.SetAnchor(nodeIndex, points[i].transform);
        }
    }

    private void ConfigurePinnedNodes()
    {
        for (int i = 0; i < segments.Count; i++)
        {
            if (segments[i].mode != CableSegmentMode.Static) continue;

            int startNode = Mathf.RoundToInt(i / (float)segments.Count * (resolution - 1));
            int endNode = Mathf.RoundToInt((i + 1) / (float)segments.Count * (resolution - 1));

            for (int n = startNode; n <= endNode; n++)
            {
                physics.PinNode(n);
            }
        }
    }

    [ContextMenu("Add Segment")]
    private CableSegment AddSegment()
    {
        CablePoint pointA = points[^1];
        CablePoint pointB = AddPoint(pointA.transform.position);

        CableSegment segment = new CableSegment(pointA, pointB);

        // Add new Cable Segments.
        segments.Add(segment);

        Refresh();

        return segment;
    }

    private CablePoint AddPoint(Vector3 position)
    {
        GameObject go = new GameObject($"Point_{points.Count}");

        go.transform.SetParent(pointsContainer);
        go.transform.position = position;

        CablePoint point = go.AddComponent<CablePoint>();
        points.Add(point);

        return point;
    }
}