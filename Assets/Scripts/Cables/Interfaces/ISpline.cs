using Unity.Mathematics;

public interface ISpline
{
    float3 EvaluatePosition(float t);
    float3 EvaluateTangent(float t);
    float3 EvaluateNormal(float t); 
}