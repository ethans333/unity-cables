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
    
    private void Awake()
    {
        spline = new CatmullRomSpline(GetControlPointPositions());
        lineRenderer = GetComponent<LineRenderer>();
    }

    private void Start()
    {
        float3[] points = new float3[numberOfSegments];
        lineRenderer.positionCount = numberOfSegments;

        for (int i = 0; i < numberOfSegments; i++)
        {
            float t =  i / (float)numberOfSegments;
            
            points[i] = spline.EvaluatePosition(t);
            
            lineRenderer.SetPosition(i, points[i]);
        }
    }

    private List<float3> GetControlPointPositions()
    {
        return controlPoints
            .Select(x => (float3)x.transform.position)
            .ToList();
    }
}