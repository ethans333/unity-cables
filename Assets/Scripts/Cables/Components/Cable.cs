using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;

[RequireComponent(typeof(LineRenderer))]
public class Cable : MonoBehaviour
{
    [SerializeField]
    private List<ControlPoint> controlPoints;
    
    [SerializeField]
    private int numberOfSegments = 100;

    private ISpline spline;
    private LineRenderer lineRenderer;
    private List<float3> previousControlPoints;
    
    private void Awake()
    {
        previousControlPoints = GetControlPointPositions().ToList();
        spline = new CatmullRomSpline();
        spline.SetPoints(GetControlPointPositions());
    }

    private void Start()
    {
        RenderDebugLine();
    }

    private void Update()
    {
        if (ControlPointsMoved())
        {
            RenderDebugLine();
        }
    }

    private float3[] GetControlPointPositions()
    {
        return controlPoints
            .Select(x => (float3)x.transform.position)
            .ToArray();
    }

    private void RenderDebugLine()
    {
        // Get reference to line renderer.
        if (lineRenderer == null)
        {
            lineRenderer = GetComponent<LineRenderer>();
        }
        
        float3[] points = new float3[numberOfSegments];
        lineRenderer.positionCount = numberOfSegments;

        // Copy points from spline into line renderer.
        for (int i = 0; i < numberOfSegments; i++)
        {
            float t =  i / (float)numberOfSegments;
            
            points[i] = spline.EvaluatePosition(t);
            
            lineRenderer.SetPosition(i, points[i]);
        }
    }

    private bool ControlPointsMoved()
    {
        bool changed = false;
        
        for (int i = 0; i < controlPoints.Count; i++)
        {
            float3 position = controlPoints[i].transform.position;

            // Check if position of control point changed.
            if (!math.all(position == previousControlPoints[i]))
            {
                // Update control point in spline if so.
                previousControlPoints[i] = position;
                spline.SetPoint(i, position);
                changed = true;
            }
        }

        return changed;
    }
}