using System;
using UnityEngine;

public class ControlPoint : MonoBehaviour
{
    [SerializeField]
    public float mass = 1f;
    
    [SerializeField]
    public bool IsFixed = false;
    
    [NonSerialized]
    public Vector3 velocity;
}