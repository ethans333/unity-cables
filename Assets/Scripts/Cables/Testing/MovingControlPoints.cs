using System;
using UnityEngine;

public class MovingControlPoints : MonoBehaviour
{
    private Cable cable;

    private void Awake()
    {
        cable = GetComponent<Cable>();
    }

    private void Update()
    {
        foreach (Transform p in cable.GetControlPoints())
        {
            p.position = new Vector3(
                p.position.x, 
                (float)Math.Sin(Time.realtimeSinceStartup + p.position.x + p.position.z), 
                p.position.z);
        }
    }
}