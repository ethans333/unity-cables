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
        foreach (Transform p in cable.GetControlPoints())
        {
            float delta = (float)Math.Sin(Time.realtimeSinceStartup + p.position.x + p.position.z);
            
            p.position = new Vector3(
                p.position.x, 
                delta * intensity + originalPosition.y, 
                p.position.z);
        }
    }
}