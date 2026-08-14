using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using static Unity.Mathematics.math;

public class CatmullRomSpline : ISpline
{
    private List<float3> controlPoints;
    
    public float3 EvaluatePosition(float t)
    {
        var (s, lT) = GetSegmentAndLocalT(t);
        
        return CatmullRom(
            controlPoints[s], 
            controlPoints[s + 1], 
            controlPoints[s + 2], 
            controlPoints[s + 3], 
            lT);
    }

    public float3 EvaluateTangent(float t)
    {
        var (s, lT) = GetSegmentAndLocalT(t);
        
        return FirstOrderDerivativeCatmullRom(
            controlPoints[s], 
            controlPoints[s + 1], 
            controlPoints[s + 2], 
            controlPoints[s + 3], 
            lT);
    }

    public void SetPoints(float3[] points)
    {
        if (points == null)
            throw new ArgumentNullException(nameof(points));

        if (points.Length < 2)
            throw new ArgumentException("A Catmull-Rom spline requires at least two control points.", nameof(points));

        controlPoints = new List<float3>(points.Length + 2)
        {
            // Duplicate the first point.
            points[0]
        };

        // Add all points.
        controlPoints.AddRange(points);
        
        // Duplicate the last point.
        controlPoints.Add(points[^1]);
    }

    public void SetPoint(int i, float3 p)
    {
        int N  = controlPoints.Count;     // Actual count.
        int oN = controlPoints.Count - 2; // Original count.

        // End edge case.
        if (i == oN-1)
        {
            controlPoints[N - 2] = p;
            controlPoints[N - 1] = p;
        }
        // Start edge case.
        else if (i == 0)
        {
            controlPoints[0] = p;
            controlPoints[1] = p;
        }
        else
        {
            controlPoints[i+1] = p;
        }
    }

    private float3 CatmullRom(float3 p0, float3 p1, float3 p2, float3 p3, float t)
    {
        float t2 = t * t;
        float t3 = t2 * t;

        return 0.5f * (
            (2f * p1) +
            (-p0 + p2) * t +
            (2f * p0 - 5f * p1 + 4f * p2 - p3) * t2 +
            (-p0 + 3f * p1 - 3f * p2 + p3) * t3
        );
    }

    private float3 FirstOrderDerivativeCatmullRom(float3 p0, float3 p1, float3 p2, float3 p3, float t)
    {
        float t2 = t * t;

        return normalizesafe(
            0.5f * (
                (-p0 + p2) +
                2f * (2f * p0 - 5f * p1 + 4f * p2 - p3) * t +
                3f * (-p0 + 3f * p1 - 3f * p2 + p3) * t2
            )
        );
    }

    private Tuple<int, float> GetSegmentAndLocalT(float t)
    {
        int   segmentCount = controlPoints.Count - 3;
        float scaled       = t * segmentCount;
        int   segment      = min((int)scaled, segmentCount - 1);
        float localT       = scaled - segment;
        
        return new Tuple<int, float>(segment, localT);
    }
}