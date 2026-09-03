using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

namespace estein.Cables
{
    [ExecuteAlways]
    [RequireComponent(typeof(MeshFilter))]
    public class Cable : MonoBehaviour
    {
        // Visuals.
        [SerializeField] [Range(1, 250)] private int resolution = 100;

        [SerializeField] [Range(3, 25)] private int sides = 12;

        [SerializeField] [Range(0.001f, 0.1f)] private float radius = 0.01f;

        // Segments.
        [SerializeField] private List<CableSegment> segments = new();

        // Physics.
        [SerializeField] private bool simulatePhysics = true;

        [SerializeField] [Range(1, 200)] private int simulationSubsteps = 20;

        // Control Points.
        [SerializeField] private Transform pointsContainer;

        private List<CablePoint> points;

        private ISpline spline;
        private List<float3> previousPointPositions;

        // Managed arrays — used by the editor path and the non-physics runtime path.
        private float3[] positions;
        private float3[] tangents;
        private float3[] splineWorldPositions;

        // Rendering.
        private MeshFilter meshFilter;
        private TubeMeshBuilder meshBuilder;

        // Physics simulation.
        private CablePhysics physics;

        // --- DOTS / Burst runtime ---
        private Mesh runtimeMesh;

        // Job-safe anchor snapshot buffers (main-thread → job boundary).
        private NativeArray<float3> anchorPositions;
        private NativeArray<int> anchorNodeIndices;

        // Persistent vertex buffer written by CableMeshBuildJob.
        private NativeArray<CableVertex> vertexBuffer;

        // Cached index buffer — rebuilt only when resolution or sides change.
        private int[] cachedTriangles;
        private int cachedTriangleResolution = -1;
        private int cachedVertexCount = -1;

        // Interleaved vertex layout — must match CableVertex field order exactly.
        private static readonly VertexAttributeDescriptor[] vertexDescriptors = new[]
        {
            new VertexAttributeDescriptor(VertexAttribute.Position, VertexAttributeFormat.Float32, 3),
            new VertexAttributeDescriptor(VertexAttribute.Normal, VertexAttributeFormat.Float32, 3),
            new VertexAttributeDescriptor(VertexAttribute.TexCoord0, VertexAttributeFormat.Float32, 2),
        };

        // Guards against re-entering Refresh()/Initialize() while already inside one.
        private bool isRefreshing;

        // -------------------------------------------------------------------------
        // Unity messages
        // -------------------------------------------------------------------------

        private void Awake() => Refresh();
        private void OnEnable() => Refresh();

        private void OnTransformChildrenChanged() => Refresh();
        private void OnValidate() => Refresh();

        private void OnDestroy()
        {
            if (physics.positions.IsCreated) physics.Dispose();
            if (anchorPositions.IsCreated) anchorPositions.Dispose();
            if (anchorNodeIndices.IsCreated) anchorNodeIndices.Dispose();
            if (vertexBuffer.IsCreated) vertexBuffer.Dispose();
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
            if (points == null || previousPointPositions == null)
                return;

            // ---- Runtime --------------------------------------------------------
            if (Application.isPlaying)
            {
                if (PointsMoved())
                {
                    spline.SetPoints(GetPointPositions());
                    CalculatePositionsAndTangentsFromSpline();

                    if (simulatePhysics)
                        physics.SetPinnedNodes(SampleSplineWorldPositions());
                    else
                        BuildMesh();
                }

                if (simulatePhysics)
                {
                    // Snapshot Transform-based anchors before leaving the main thread.
                    physics.SnapshotAnchors(anchorPositions, anchorNodeIndices, out int anchorCount);

                    float dt = Time.deltaTime / simulationSubsteps;
                    float3 gravity = Physics.gravity;

                    for (int s = 0; s < simulationSubsteps; s++)
                    {
                        // Burst job: Verlet + constraint iterations.
                        physics.CreateSimulationJob(dt, gravity, anchorPositions, anchorNodeIndices, anchorCount)
                            .Schedule()
                            .Complete();

                        // Collision must stay on the main thread (Physics API limitation).
                        physics.ProjectOutOfColliders();
                        physics.ApplyCollisionResponse();
                    }

                    BuildMeshFromPhysicsJob();
                }
            }
            // ---- Editor ---------------------------------------------------------
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

        // -------------------------------------------------------------------------
        // Initialization
        // -------------------------------------------------------------------------

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

        private void Initialize()
        {
            // Control points.
            PopulatePoints();
            previousPointPositions = GetPointPositions().ToList();

            // Spline.
            spline = new CatmullRomSpline();
            spline.SetPoints(GetPointPositions());

            // Editor-path rendering.
            meshFilter = GetComponent<MeshFilter>();
            meshBuilder = new TubeMeshBuilder(sides, radius);

            // Managed sample arrays (editor path + spline-only runtime).
            positions = new float3[resolution];
            tangents = new float3[resolution];
            splineWorldPositions = new float3[resolution];

            // Dispose previous NativeArrays before reallocating — OnValidate can
            // call Initialize() repeatedly while the game is running.
            if (physics.positions.IsCreated) physics.Dispose();
            if (anchorPositions.IsCreated) anchorPositions.Dispose();
            if (anchorNodeIndices.IsCreated) anchorNodeIndices.Dispose();
            if (vertexBuffer.IsCreated) vertexBuffer.Dispose();

            if (simulatePhysics)
            {
                physics = new CablePhysics(SampleSplineWorldPositions(), radius, 0f, 0.3f);
                ConfigurePhysicsAnchors();
                ConfigurePinnedNodes();

                // Size anchor buffers to the maximum possible anchor count.
                int maxAnchors = math.max(points.Count, 1);
                anchorPositions = new NativeArray<float3>(maxAnchors, Allocator.Persistent);
                anchorNodeIndices = new NativeArray<int>(maxAnchors, Allocator.Persistent);

                // Separate dynamic mesh for the runtime Burst path.
                runtimeMesh = new Mesh { name = "CableRuntimeMesh" };
                runtimeMesh.MarkDynamic();

                // Force vertex / index buffer rebuild on next BuildMeshFromPhysicsJob().
                cachedVertexCount = -1;
                cachedTriangleResolution = -1;
            }

            CalculatePositionsAndTangentsFromSpline();
        }

        // -------------------------------------------------------------------------
        // Runtime Burst mesh build
        // -------------------------------------------------------------------------

        private void BuildMeshFromPhysicsJob()
        {
            int vertexCount = resolution * sides;

            // Reallocate vertex buffer when resolution or sides changed.
            if (cachedVertexCount != vertexCount)
            {
                cachedVertexCount = vertexCount;
                runtimeMesh.SetVertexBufferParams(vertexCount, vertexDescriptors);

                if (vertexBuffer.IsCreated) vertexBuffer.Dispose();
                vertexBuffer = new NativeArray<CableVertex>(vertexCount, Allocator.Persistent);
            }

            // Parallel job: one Execute() per segment ring.
            new CableMeshBuildJob
            {
                positions = physics.positions,
                vertices = vertexBuffer,
                sides = sides,
                nodeCount = resolution,
                radius = radius,
                worldToLocal = (float4x4)transform.worldToLocalMatrix,
            }.Schedule(resolution, 8).Complete();

            runtimeMesh.SetVertexBufferData(
                vertexBuffer, 0, 0, vertexCount,
                0, // stream index
                MeshUpdateFlags.DontValidateIndices |
                MeshUpdateFlags.DontNotifyMeshUsers |
                MeshUpdateFlags.DontRecalculateBounds);

            // Index buffer is topology-only — skip when nothing changed.
            if (cachedTriangleResolution != resolution)
                GenerateCachedTriangles();

            // Fast approximate bounds from the two endpoint positions.
            float3 p0 = physics.positions[0];
            float3 pEnd = physics.positions[resolution - 1];
            float3 centre = (p0 + pEnd) * 0.5f;
            float extent = math.length(pEnd - p0) * 0.5f + radius;
            runtimeMesh.bounds = new Bounds(
                transform.InverseTransformPoint(centre),
                Vector3.one * (extent * 2f));

            meshFilter.mesh = runtimeMesh;
        }

        private void GenerateCachedTriangles()
        {
            cachedTriangleResolution = resolution;

            int segmentCount = resolution - 1;
            cachedTriangles = new int[segmentCount * sides * 6];

            int ti = 0;
            for (int seg = 0; seg < segmentCount; seg++)
            {
                for (int s = 0; s < sides; s++)
                {
                    int a = seg * sides + s;
                    int b = seg * sides + (s + 1) % sides;
                    int c = (seg + 1) * sides + s;
                    int d = (seg + 1) * sides + (s + 1) % sides;

                    cachedTriangles[ti++] = a;
                    cachedTriangles[ti++] = b;
                    cachedTriangles[ti++] = c;

                    cachedTriangles[ti++] = b;
                    cachedTriangles[ti++] = d;
                    cachedTriangles[ti++] = c;
                }
            }

            runtimeMesh.SetIndexBufferParams(cachedTriangles.Length, IndexFormat.UInt32);
            runtimeMesh.SetIndexBufferData(
                cachedTriangles, 0, 0, cachedTriangles.Length,
                MeshUpdateFlags.DontValidateIndices | MeshUpdateFlags.DontNotifyMeshUsers);
            runtimeMesh.SetSubMesh(
                0,
                new SubMeshDescriptor(0, cachedTriangles.Length),
                MeshUpdateFlags.DontValidateIndices | MeshUpdateFlags.DontNotifyMeshUsers);
        }

        // -------------------------------------------------------------------------
        // Editor mesh build (managed path)
        // -------------------------------------------------------------------------

        private void BuildMesh()
        {
            meshFilter.mesh = meshBuilder.Build(positions, tangents);
        }

        // -------------------------------------------------------------------------
        // Spline sampling
        // -------------------------------------------------------------------------

        private void CalculatePositionsAndTangentsFromSpline()
        {
            for (int i = 0; i < resolution; i++)
            {
                float t = i / (float)(resolution - 1);
                positions[i] = transform.InverseTransformPoint(spline.EvaluatePosition(t));
                tangents[i] = transform.InverseTransformDirection(spline.EvaluateTangent(t));
            }
        }

        private float3[] SampleSplineWorldPositions()
        {
            for (int i = 0; i < resolution; i++)
            {
                float t = i / (float)(resolution - 1);
                splineWorldPositions[i] = spline.EvaluatePosition(t);
            }

            return splineWorldPositions;
        }

        // -------------------------------------------------------------------------
        // Change detection
        // -------------------------------------------------------------------------

        private bool PointsChanged()
        {
            List<CablePoint> actual = GetPoints();
            List<CablePoint> removed = points.Except(actual).ToList();

            if (removed.Any())
            {
                ValidateSegments(removed);
                return true;
            }

            if (actual.Count != points.Count) return true;

            for (int i = 0; i < actual.Count; i++)
                if (actual[i] != points[i])
                    return true;

            return false;
        }

        private bool PointsMoved()
        {
            bool changed = false;

            for (int i = 0; i < points.Count; i++)
            {
                if (!points[i]) continue;

                float3 position = points[i].transform.position;

                if (i == previousPointPositions.Count)
                {
                    previousPointPositions.Add(position);
                    spline.SetPoint(i, position);
                    changed = true;
                }
                else if (!math.all(position == previousPointPositions[i]))
                {
                    previousPointPositions[i] = position;
                    spline.SetPoint(i, position);
                    changed = true;
                }
            }

            return changed;
        }

        // -------------------------------------------------------------------------
        // Physics configuration
        // -------------------------------------------------------------------------

        private void ConfigurePhysicsAnchors()
        {
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
                    physics.PinNode(n);
            }
        }

        // -------------------------------------------------------------------------
        // Point / segment management
        // -------------------------------------------------------------------------

        private List<CablePoint> GetPoints()
            => pointsContainer.GetComponentsInChildren<CablePoint>().ToList();

        private void PopulatePoints()
        {
            points = GetPoints();
            if (points.Count == 0) InitializePoints();
        }

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
            points = new List<CablePoint>();
            AddPoint(transform.position);
            CableSegment segment = AddSegment();
            segment.pointB.transform.position += segment.pointB.transform.forward;
        }

        private float3[] GetPointPositions()
            => points.Select(x => (float3)x.transform.position).ToArray();

        [ContextMenu("Add Segment")]
        private CableSegment AddSegment()
        {
            CablePoint pointA = points[^1];
            CablePoint pointB = AddPoint(pointA.transform.position);
            CableSegment segment = new CableSegment(pointA, pointB);
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
}
