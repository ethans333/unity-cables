using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

public struct CablePhysics : IDisposable
{
    // Public — read directly by Cable when building the mesh.
    public NativeArray<float3> positions;

    // Internal — accessed via CreateSimulationJob so job structs stay decoupled
    // from CablePhysics field layout.
    internal NativeArray<float3> previousPositions;
    internal NativeArray<bool>   isPinned;
    internal NativeArray<float>  restLengths;
    internal NativeArray<float>  bendRestLengths;
    internal NativeArray<float>  bendStiffness;

    private NativeArray<float3> collisionNormal;
    private NativeArray<bool>   hadCollision;

    private readonly int       nodeCount;
    private readonly float     radius;
    private readonly float     restitution;
    private readonly float     friction;
    private readonly LayerMask collisionMask;

    // Exposed so Cable can forward them into the simulation job.
    public int   constraintIterations;
    public float velocityDamping;
    public float maxSpeed;

    // Anchors hold Transform references — managed objects that cannot live in a
    // NativeArray. Call SnapshotAnchors() on the main thread each frame to copy
    // their world positions into job-safe NativeArrays before scheduling.
    private List<(int nodeIndex, Transform anchor)> anchors;

    public int AnchorCount => anchors?.Count ?? 0;

    private const float anchorBendStiffnessMax  = 0.9f;
    private const int   anchorBendFalloffNodes  = 15;

    private static readonly Collider[]        overlapBuffer   = new Collider[16];
    private static readonly HashSet<Collider> warnedColliders = new();

    /// <param name="initialPositions">World-space positions for every simulation node.</param>
    /// <param name="restitution">Bounciness on collision impact.</param>
    public CablePhysics(float3[] initialPositions, float radius, float restitution, float friction)
    {
        nodeCount = initialPositions.Length;

        positions         = new NativeArray<float3>(nodeCount,     Allocator.Persistent);
        previousPositions = new NativeArray<float3>(nodeCount,     Allocator.Persistent);
        isPinned          = new NativeArray<bool>  (nodeCount,     Allocator.Persistent);
        restLengths       = new NativeArray<float> (nodeCount - 1, Allocator.Persistent);
        bendRestLengths   = new NativeArray<float> (nodeCount,     Allocator.Persistent);
        bendStiffness     = new NativeArray<float> (nodeCount,     Allocator.Persistent);
        collisionNormal   = new NativeArray<float3>(nodeCount,     Allocator.Persistent);
        hadCollision      = new NativeArray<bool>  (nodeCount,     Allocator.Persistent);

        for (int i = 0; i < nodeCount; i++)
        {
            positions[i]         = initialPositions[i];
            previousPositions[i] = initialPositions[i];
        }

        this.radius      = radius;
        this.restitution = restitution;
        this.friction    = friction;
        collisionMask    = ~0;
        anchors          = new List<(int, Transform)>();

        constraintIterations = 8;
        velocityDamping      = 0.01f;
        maxSpeed             = 5f;

        CalculateRestLengths();
        CalculateBendRestLengths();
    }

    public void Dispose()
    {
        if (positions.IsCreated)         positions.Dispose();
        if (previousPositions.IsCreated) previousPositions.Dispose();
        if (isPinned.IsCreated)          isPinned.Dispose();
        if (restLengths.IsCreated)       restLengths.Dispose();
        if (bendRestLengths.IsCreated)   bendRestLengths.Dispose();
        if (bendStiffness.IsCreated)     bendStiffness.Dispose();
        if (collisionNormal.IsCreated)   collisionNormal.Dispose();
        if (hadCollision.IsCreated)      hadCollision.Dispose();
    }

    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------

    /// <summary>Pins a node to follow a Transform. Pass null to release.</summary>
    public void SetAnchor(int nodeIndex, Transform anchor)
    {
        anchors.RemoveAll(a => a.nodeIndex == nodeIndex);
        if (anchor) anchors.Add((nodeIndex, anchor));
        isPinned[nodeIndex] = anchor != null;
        ConfigureAnchorBendStiffness();
    }

    public void PinNode(int nodeIndex)
    {
        isPinned[nodeIndex] = true;
    }

    /// <summary>Teleports pinned nodes to match new spline samples (e.g. when a control point moves).</summary>
    public void SetPinnedNodes(float3[] newPositions)
    {
        if (newPositions.Length != nodeCount) return;

        for (int i = 0; i < nodeCount; i++)
        {
            if (!isPinned[i]) continue;
            positions[i]         = newPositions[i];
            previousPositions[i] = newPositions[i];
        }
    }

    // -------------------------------------------------------------------------
    // Job scheduling helpers
    // -------------------------------------------------------------------------

    /// <summary>
    /// Copies current anchor Transform positions into job-safe NativeArrays.
    /// Must be called on the main thread before scheduling a simulation job.
    /// outPositions and outIndices must be allocated with at least AnchorCount entries.
    /// </summary>
    public void SnapshotAnchors(NativeArray<float3> outPositions, NativeArray<int> outIndices, out int count)
    {
        count = anchors.Count;
        for (int i = 0; i < count; i++)
        {
            outPositions[i] = anchors[i].anchor.position;
            outIndices[i]   = anchors[i].nodeIndex;
        }
    }

    /// <summary>
    /// Returns a configured simulation job ready to be scheduled.
    /// Complete the job handle, then call ProjectOutOfColliders and
    /// ApplyCollisionResponse on the main thread.
    /// </summary>
    public CableSimulationJob CreateSimulationJob(
        float dt, float3 gravity,
        NativeArray<float3> anchorPositions,
        NativeArray<int>    anchorNodeIndices,
        int                 anchorCount)
    {
        return new CableSimulationJob
        {
            positions            = positions,
            previousPositions    = previousPositions,
            isPinned             = isPinned,
            restLengths          = restLengths,
            bendRestLengths      = bendRestLengths,
            bendStiffness        = bendStiffness,
            anchorPositions      = anchorPositions,
            anchorNodeIndices    = anchorNodeIndices,
            nodeCount            = nodeCount,
            anchorCount          = anchorCount,
            constraintIterations = constraintIterations,
            dt                   = dt,
            gravity              = gravity,
            velocityDamping      = velocityDamping,
            maxSpeed             = maxSpeed,
        };
    }

    // -------------------------------------------------------------------------
    // Collision — main-thread only (Physics API)
    // -------------------------------------------------------------------------

    /// <summary>
    /// Projects penetrating nodes out of colliders and records contact normals.
    /// Call on the main thread after the simulation job completes, once per substep.
    /// </summary>
    public void ProjectOutOfColliders()
    {
        ClearArray(hadCollision);

        for (int i = 0; i < nodeCount; i++)
        {
            if (isPinned[i]) continue;

            Vector3 position = positions[i];

            int count = Physics.OverlapSphereNonAlloc(
                position, radius, overlapBuffer, collisionMask, QueryTriggerInteraction.Ignore);

            for (int c = 0; c < count; c++)
            {
                Collider other = overlapBuffer[c];
                if (!TryGetClosestPoint(other, position, out Vector3 closest)) continue;

                Vector3 offset   = position - closest;
                float   distance = offset.magnitude;

                if (distance < radius)
                {
                    Vector3 direction = distance > 0.0001f ? offset / distance : Vector3.up;
                    position += direction * (radius - distance);

                    collisionNormal[i] = direction;
                    hadCollision[i]    = true;
                }
            }

            positions[i] = position;
        }
    }

    /// <summary>
    /// Applies restitution and friction using contact normals from ProjectOutOfColliders.
    /// Call immediately after ProjectOutOfColliders on the main thread.
    /// </summary>
    public void ApplyCollisionResponse()
    {
        for (int i = 0; i < nodeCount; i++)
        {
            if (isPinned[i] || !hadCollision[i]) continue;

            float3 direction = collisionNormal[i];
            float3 velocity  = positions[i] - previousPositions[i];

            float vDotN = math.dot(velocity, direction);
            if (vDotN < 0f)
                velocity -= direction * (vDotN * (1f + restitution));

            velocity -= (velocity - direction * math.dot(velocity, direction)) * friction;

            previousPositions[i] = positions[i] - velocity;
        }
    }

    // -------------------------------------------------------------------------
    // Private
    // -------------------------------------------------------------------------

    private void CalculateRestLengths()
    {
        for (int i = 0; i < nodeCount - 1; i++)
            restLengths[i] = math.distance(positions[i], positions[i + 1]);
    }

    private void CalculateBendRestLengths()
    {
        for (int i = 1; i < nodeCount - 1; i++)
            bendRestLengths[i] = math.distance(positions[i - 1], positions[i + 1]);
    }

    private void ConfigureAnchorBendStiffness()
    {
        ClearArray(bendStiffness);

        foreach (var (anchorNode, _) in anchors)
        {
            for (int i = 1; i < nodeCount - 1; i++)
            {
                int distanceFromAnchor = math.abs(i - anchorNode);
                if (distanceFromAnchor > anchorBendFalloffNodes) continue;

                float t         = distanceFromAnchor / (float)anchorBendFalloffNodes;
                float stiffness = (1f - math.smoothstep(0f, 1f, t)) * anchorBendStiffnessMax;
                bendStiffness[i] = math.max(bendStiffness[i], stiffness);
            }
        }
    }


    // Collider.ClosestPoint only supports Box/Sphere/Capsule and CONVEX MeshColliders.
    private static bool TryGetClosestPoint(Collider collider, Vector3 point, out Vector3 closest)
    {
        if (collider is MeshCollider mesh && !mesh.convex)
        {
            if (warnedColliders.Add(collider))
                Debug.LogWarning(
                    $"CablePhysics: '{collider.name}' is a non-convex MeshCollider. " +
                    "Enable 'Convex' or replace with a primitive collider.",
                    collider);

            closest = collider.bounds.ClosestPoint(point);
            return true;
        }

        closest = collider.ClosestPoint(point);
        return true;
    }

    private static void ClearArray(NativeArray<bool> array)
    {
        for (int i = 0; i < array.Length; i++) array[i] = false;
    }

    private static void ClearArray(NativeArray<float> array)
    {
        for (int i = 0; i < array.Length; i++) array[i] = 0f;
    }
}
