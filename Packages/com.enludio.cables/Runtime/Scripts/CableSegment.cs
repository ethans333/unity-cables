
using System;
using UnityEngine;

public enum CableSegmentMode
{
    Static,
    Dynamic,
}

[Serializable]
public class CableSegment
{
    public CableSegmentMode mode;

    [NonSerialized]
    public CablePoint pointA;
    
    [NonSerialized]
    public CablePoint pointB;
    
    public CableSegment(CablePoint pointA, CablePoint pointB)
    {
        this.pointA = pointA;
        this.pointB = pointB;
        
        mode = CableSegmentMode.Static;
    }
}