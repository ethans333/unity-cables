using System;
using UnityEngine;

public class MovingControlPoints : MonoBehaviour
{
    [SerializeField, Range(0f, 10f)]
    private float intensity = 1f;
    
    private Vector3 originalPosition;
    private Cable cable;

    private void Awake()
    {
        cable = GetComponent<Cable>();
        originalPosition = cable.transform.position;
    }

    private void Update()
    {
        SinWaveOnControlPoints();
    }

    private void SinWaveOnControlPoints()
    {
        foreach (ControlPoint point in cable.controlPoints)
        {
            Vector3 p = point.transform.position;
            float delta = (float)Math.Sin(Time.realtimeSinceStartup + p.x + p.z);
            
            point.transform.position = new Vector3(
                p.x, 
                delta * intensity + originalPosition.y, 
                p.z);
        }
    }
}