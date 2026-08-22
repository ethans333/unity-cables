using System;
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
    private int numberOfSegments = 100;

    [SerializeField]
    [Range(3, 25)]
    private int sides = 12;

    [SerializeField]
    [Range(0.001f, 0.1f)]
    private float radius = 0.01f;

    // Physics.
    [SerializeField]
    [Range(1, 200)]
    private int simulationSubsteps = 20; 
    
    [NonSerialized]
    public List<ControlPoint> controlPoints;
    
    private ISpline spline;
    private List<float3> previousControlPointPositions;

    private float3[] positions;
    private float3[] tangents;

    private MeshFilter meshFilter;
    private TubeMeshBuilder meshBuilder;

    private CablePhysics physics;

    void Initialize()
    {
        // Find Control Points.
        controlPoints = GetComponentsInChildren<ControlPoint>().ToList();

        if (controlPoints.Count == 0)
        {
            // No control points exist.
            return;
        }

        // Get Control Point positions.
        float3[] P = GetControlPointPositions();
        previousControlPointPositions = P.ToList();

        // Get points from spline.
        spline = new CatmullRomSpline();
        spline.SetPoints(P);

        // Rendering.
        meshFilter = GetComponent<MeshFilter>();
        meshBuilder = new TubeMeshBuilder(sides, radius);

        physics = new CablePhysics(SampleSplineWorldPositions(), radius, 0f, 0.3f);

        ConfigurePhysicsAnchors();
        
        positions = new float3[numberOfSegments];
        tangents = new float3[numberOfSegments];
        
        CalculatePositionsAndTangentsFromSpline();
    }

    private void Awake()
    {
        Initialize();
        BuildMesh();
    }

    private void OnEnable()
    {
        Initialize();
        BuildMesh();
    }

    private void OnTransformChildrenChanged()
    {
        Initialize();
        BuildMesh();
    }

    private void OnValidate()
    {
        Initialize();
        BuildMesh();
    }
    
    private void OnDrawGizmos()
    {
        foreach (ControlPoint p in controlPoints)
        {
            Gizmos.DrawSphere(p.transform.position, radius * 2f);
        }
    }

    private void Update()
    {
        if (controlPoints == null || previousControlPointPositions == null || physics == null)
            return;

        if (Application.isPlaying)
        {
            ControlPointsMoved();
            
            for (int i = 0; i < simulationSubsteps; i++)
            {
                physics.SimulatePhysics(Time.deltaTime / simulationSubsteps);
            }
            
            BuildMeshFromPhysics();
        }
        else
        {
            if (ControlPointsStructureChanged())
            {
                Initialize();
                BuildMesh();
                return;
            }

            if (ControlPointsMoved())
            {
                CalculatePositionsAndTangentsFromSpline();
                BuildMesh();
            }
        }
    }

    private bool ControlPointsStructureChanged()
    {
        List<ControlPoint> current = GetComponentsInChildren<ControlPoint>().ToList();

        if (current.Count != controlPoints.Count) return true;

        for (int i = 0; i < current.Count; i++)
        {
            if (current[i] != controlPoints[i]) return true;
        }

        return false;
    }

    private bool ControlPointsValid()
    {
        return controlPoints != null &&
               controlPoints.Count > 0 &&
               controlPoints.All(x => x != null);
    }

    private float3[] GetControlPointPositions()
    {
        return controlPoints.Select(x => (float3)x.transform.position).ToArray();
    }

    private float3[] SampleSplineWorldPositions()
    {
        float3[] result = new float3[numberOfSegments];
        for (int i = 0; i < numberOfSegments; i++)
        {
            float t = i / (float)(numberOfSegments - 1);
            result[i] = spline.EvaluatePosition(t);
        }
        return result;
    }

    private void CalculatePositionsAndTangentsFromSpline()
    {
        for (int i = 0; i < numberOfSegments; i++)
        {
            float t = i / (float)(numberOfSegments - 1);

            positions[i] = transform.InverseTransformPoint(spline.EvaluatePosition(t));
            tangents[i] = transform.InverseTransformDirection(spline.EvaluateTangent(t));
        }
    }

    private void BuildMeshFromPhysics()
    {
        float3[] worldPositions = physics.positions;

        for (int i = 0; i < numberOfSegments; i++)
        {
            positions[i] = transform.InverseTransformPoint(worldPositions[i]);

            // Physics-driven shape no longer matches the spline, so derive tangents from
            // the simulated chain itself via finite differences instead of the spline.
            float3 prev = worldPositions[math.max(i - 1, 0)];
            float3 next = worldPositions[math.min(i + 1, numberOfSegments - 1)];
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

    private bool ControlPointsMoved()
    {
        bool changed = false;

        for (int i = 0; i < controlPoints.Count; i++)
        {
            if (controlPoints[i] == null) continue;

            float3 position = controlPoints[i].transform.position;

            // Check if position of control point changed.
            if (!math.all(position == previousControlPointPositions[i]))
            {
                // Update control point in spline if so.
                previousControlPointPositions[i] = position;
                spline.SetPoint(i, position);
                changed = true;
            }
        }

        return changed;
    }

    private void ConfigurePhysicsAnchors()
    {
        // Pin certain nodes of the cable to the control points via physics anchors.
        for (int i = 0; i < controlPoints.Count; i++)
        {
            if (!controlPoints[i].isFixed) continue;
            
            float t = controlPoints.Count > 1 ? i / (float)(controlPoints.Count - 1) : 0f;
            int nodeIndex = Mathf.RoundToInt(t * (numberOfSegments - 1));

            physics.SetAnchor(nodeIndex, controlPoints[i].transform);
        }
    }
}