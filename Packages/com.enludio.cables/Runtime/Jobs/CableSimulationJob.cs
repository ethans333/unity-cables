using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

[BurstCompile]
public struct CableSimulationJob : IJob
{
    public NativeArray<float3> positions;
    public NativeArray<float3> previousPositions;

    [ReadOnly] public NativeArray<bool>   isPinned;
    [ReadOnly] public NativeArray<float>  restLengths;
    [ReadOnly] public NativeArray<float>  bendRestLengths;
    [ReadOnly] public NativeArray<float>  bendStiffness;

    // Anchor positions are snapshotted from Transform on the main thread before
    // scheduling — Transform cannot be accessed inside a Burst job.
    [ReadOnly] public NativeArray<float3> anchorPositions;
    [ReadOnly] public NativeArray<int>    anchorNodeIndices;

    public int    nodeCount;
    public int    anchorCount;
    public int    constraintIterations;
    public float  dt;
    public float3 gravity;
    public float  velocityDamping;
    public float  maxSpeed;

    public void Execute()
    {
        ApplyAnchors();
        VerletIntegrate();

        // NOTE: Physics.OverlapSphere is a main-thread-only API and cannot run in a
        // Burst job. Collision projection is therefore handled on the main thread after
        // this job completes — once per substep rather than once per constraint iteration.
        // This is a known quality tradeoff for parallelism; the visible difference is
        // negligible at typical substep counts.
        for (int iter = 0; iter < constraintIterations; iter++)
        {
            SolveDistanceConstraints();
            SolveBendingConstraints();
            ApplyAnchors();
        }
    }

    void ApplyAnchors()
    {
        for (int i = 0; i < anchorCount; i++)
        {
            int idx = anchorNodeIndices[i];
            positions[idx]         = anchorPositions[i];
            previousPositions[idx] = anchorPositions[i];
        }
    }

    void VerletIntegrate()
    {
        float dtSq = dt * dt;

        for (int i = 0; i < nodeCount; i++)
        {
            if (isPinned[i]) continue;

            float3 current  = positions[i];
            float3 velocity = (current - previousPositions[i]) * (1f - velocityDamping);

            float speed   = math.length(velocity);
            float maxStep = maxSpeed * dt;
            if (speed > maxStep && speed > 0f)
                velocity *= maxStep / speed;

            previousPositions[i] = current;
            positions[i]         = current + velocity + gravity * dtSq;
        }
    }

    void SolveDistanceConstraints()
    {
        for (int i = 0; i < nodeCount - 1; i++)
        {
            bool aPinned = isPinned[i];
            bool bPinned = isPinned[i + 1];
            if (aPinned && bPinned) continue;

            float3 a     = positions[i];
            float3 b     = positions[i + 1];
            float3 delta = b - a;
            float  dist  = math.length(delta);

            if (dist < 1e-6f) continue;

            float3 correction = delta * ((dist - restLengths[i]) / dist) * 0.5f;

            if (aPinned)
                positions[i + 1] -= correction * 2f;
            else if (bPinned)
                positions[i] += correction * 2f;
            else
            {
                positions[i]     += correction;
                positions[i + 1] -= correction;
            }
        }
    }

    void SolveBendingConstraints()
    {
        for (int i = 1; i < nodeCount - 1; i++)
        {
            float stiffness = bendStiffness[i];
            if (stiffness <= 0f) continue;

            bool aPinned = isPinned[i - 1];
            bool bPinned = isPinned[i];
            bool cPinned = isPinned[i + 1];

            if (aPinned && bPinned && cPinned) continue;

            float3 a        = positions[i - 1];
            float3 c        = positions[i + 1];
            float3 delta    = c - a;
            float  distance = math.length(delta);

            if (distance < 1e-6f) continue;

            float3 correction = delta * ((distance - bendRestLengths[i]) / distance) * 0.5f * stiffness;

            if (aPinned && !cPinned)
                positions[i + 1] -= correction * 2f;
            else if (cPinned && !aPinned)
                positions[i - 1] += correction * 2f;
            else if (!aPinned && !cPinned)
            {
                positions[i - 1] += correction;
                positions[i + 1] -= correction;
            }
        }
    }
}
