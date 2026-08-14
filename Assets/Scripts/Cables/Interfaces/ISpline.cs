using System.Collections.Generic;
using Unity.Mathematics;

public interface ISpline
{
    void SetPoints(float3[] points);
    void SetPoint(int i, float3 p);
    float3 EvaluatePosition(float t);
    float3 EvaluateTangent(float t);
    float3 EvaluateNormal(float t);
}