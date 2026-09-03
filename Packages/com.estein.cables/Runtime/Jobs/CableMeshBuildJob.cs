using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

/// <summary>
/// Interleaved vertex layout written directly into the GPU vertex buffer.
/// Must match the VertexAttributeDescriptors declared in Cable.cs exactly.
/// </summary>
[StructLayout(LayoutKind.Sequential)]
public struct CableVertex
{
    public float3 position;
    public float3 normal;
    public float2 uv;
}

/// <summary>
/// Burst-compiled parallel job that builds one ring of tube vertices per Execute call.
/// Tangents are derived from finite differences on the simulation positions,
/// so no separate tangent pass is needed.
/// </summary>
[BurstCompile]
public struct CableMeshBuildJob : IJobParallelFor
{
    [ReadOnly]  public NativeArray<float3>    positions;
    [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<CableVertex> vertices;

    public int     sides;
    public int     nodeCount;
    public float   radius;
    public float4x4 worldToLocal;

    // Each index is one segment — writes `sides` vertices starting at segment * sides.
    public void Execute(int segment)
    {
        float3 worldPos = positions[segment];

        // Tangent via central finite differences on the physics chain.
        float3 prev = positions[math.max(segment - 1, 0)];
        float3 next = positions[math.min(segment + 1, nodeCount - 1)];
        float3 dir  = next - prev;

        float3 worldTangent = math.lengthsq(dir) > 1e-12f
            ? math.normalize(dir)
            : new float3(0f, 0f, 1f);

        // Transform position and tangent into local space.
        float3 localPos     = math.transform(worldToLocal, worldPos);
        float3 localTangent = math.normalize(
            math.mul(worldToLocal, new float4(worldTangent, 0f)).xyz);

        // Stable frame — fall back to right when tangent is nearly parallel to up,
        // which would make cross(tangent, up) degenerate and produce NaN normals.
        float3 reference = math.abs(math.dot(localTangent, math.up())) > 0.99f
            ? math.right()
            : math.up();

        float3 normal   = math.normalize(math.cross(localTangent, reference));
        float3 binormal = math.cross(localTangent, normal);

        float v         = segment / (float)(nodeCount - 1);
        int   baseIndex = segment * sides;

        for (int j = 0; j < sides; j++)
        {
            float  angle     = math.PI2 * j / sides;
            float3 direction = math.cos(angle) * normal + math.sin(angle) * binormal;

            vertices[baseIndex + j] = new CableVertex
            {
                position = localPos + direction * radius,
                normal   = math.normalize(direction),
                uv       = new float2(j / (float)sides, v),
            };
        }
    }
}
