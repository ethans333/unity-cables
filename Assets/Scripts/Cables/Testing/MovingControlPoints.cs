using System;
using UnityEngine;

public class MovingControlPoints : MonoBehaviour
{
    [SerializeField, Range(0f, 10f)]
    private float intensity = 1f;
    
    private Cable cable;

    private void Awake()
    {
        cable = GetComponent<Cable>();
    }

    private void Update()
    {
        SinWaveOnControlPoints();
    }

    private void SinWaveOnControlPoints()
    {
        foreach (Transform p in cable.GetControlPoints())
        {
            float newY = (float)Math.Sin(Time.realtimeSinceStartup + p.position.x + p.position.z) * intensity;
            p.position = new Vector3(p.position.x, newY, p.position.z);
        }
    }
}