using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Serialization;

[ExecuteAlways]
[RequireComponent(typeof(MeshFilter))]
public class Cable : MonoBehaviour
{
    [SerializeField]
    private int numberOfSegments = 100;
    
    [SerializeField]
    private int sides = 12;

    [SerializeField]
    private float radius = 0.01f;
    
    [SerializeField]
    private bool showControlPointGizmos = true;
    
    [NonSerialized]
    public List<ControlPoint> controlPoints;
    
    private ISpline spline;
    private LineRenderer lineRenderer;
    private List<float3> previousControlPointPositions;
    
    private MeshFilter meshFilter;
    private TubeMeshBuilder meshBuilder;

    private CablePhysics physics;

    void Initialize()
    {
        controlPoints = GetComponentsInChildren<ControlPoint>().ToList();
        previousControlPointPositions = GetControlPointPositions().ToList();
        
        spline = new CatmullRomSpline();
        spline.SetPoints(GetControlPointPositions());
        
        meshFilter = GetComponent<MeshFilter>();
        meshBuilder = new TubeMeshBuilder(sides, radius);
        physics = new CablePhysics(controlPoints, radius, 0f, 0.3f);
    }
    
    private void Awake()
    {
        Initialize();
        BuildMesh();
    }

    private void OnValidate()
    {
        // On Engine start points aren't ready yet.
        if (!ControlPointsValid()) return;
        
        Initialize();
        BuildMesh();
    }

    private void Update()
    {
        if (Application.isPlaying)
        {
            int substeps = 20;
            
            for (int i = 0; i < substeps; i++)
            {
                physics.SimulatePhysics(Time.deltaTime / substeps);
            }
        }
        
        if (ControlPointsMoved())
        {
            BuildMesh();
        }
    }

    private bool ControlPointsValid()
    {
        return controlPoints != null && 
               controlPoints.Count > 0 && 
               controlPoints.Any(x => x == null);
    }

    private float3[] GetControlPointPositions()
    {
        return controlPoints.Select(x => (float3)x.transform.position).ToArray();
    }

    private void BuildMesh()
    {
        float3[] positions = new float3[numberOfSegments];
        float3[] tangents  = new float3[numberOfSegments];

        // Copy points from spline into line renderer.
        for (int i = 0; i < numberOfSegments; i++)
        {
            float t =  i / (float)numberOfSegments;
            
            positions[i] = transform.InverseTransformPoint(spline.EvaluatePosition(t));
            tangents[i]  = transform.InverseTransformDirection(spline.EvaluateTangent(t));
        }
        
        meshFilter.mesh = meshBuilder.Build(positions, tangents);
    }

    private bool ControlPointsMoved()
    {
        bool changed = false;
        
        for (int i = 0; i < controlPoints.Count; i++)
        {
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

    private void OnDrawGizmos()
    {
        if (!showControlPointGizmos) return;
        
        foreach (ControlPoint p in controlPoints)
        {
            Gizmos.DrawSphere(p.transform.position, radius * 2f);
        }
    }
}