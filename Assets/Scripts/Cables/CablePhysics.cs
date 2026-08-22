using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

public class CablePhysics
{
    public float3[] positions;

    private float3[] previousPositions;
    private bool[] isPinned;

    private readonly int nodeCount;
    private readonly float radius;
    private readonly float restitution;
    private readonly float friction;
    private readonly float restLength;

    private const int constraintIterations = 8;
    private const float maxSpeed = 5f;

    private const float velocityDamping = 0.01f;

    private readonly LayerMask collisionMask = ~0;
    private static readonly Collider[] overlapBuffer = new Collider[16];
    
    private readonly List<(int nodeIndex, Transform anchor)> anchors = new();
    
    private float3[] collisionNormal;
    private bool[] hadCollision;
    
    private static readonly HashSet<Collider> warnedColliders = new();

    /// <param name="initialPositions">World-space positions for every simulation node,
    /// one per segment. Sample these from your spline at construction time.</param>
    public CablePhysics(float3[] initialPositions, float radius, float restitution, float friction)
    {
        nodeCount = initialPositions.Length;
        positions = (float3[])initialPositions.Clone();
        previousPositions = (float3[])initialPositions.Clone();
        isPinned = new bool[nodeCount];

        this.radius = radius;
        this.restitution = restitution;
        this.friction = friction;

        // Rest length is derived from the initial spacing so the chain starts unstressed
        // regardless of how numberOfSegments was chosen.
        float total = 0f;
        for (int i = 1; i < nodeCount; i++)
            total += math.distance(initialPositions[i], initialPositions[i - 1]);
        restLength = nodeCount > 1 ? total / (nodeCount - 1) : 0f;
    }

    /// <summary>
    /// Pins a node to follow a Transform exactly (e.g. cable endpoint attached to a
    /// control point or socket). Pass null to release the pin.
    /// </summary>
    public void SetAnchor(int nodeIndex, Transform anchor)
    {
        anchors.RemoveAll(a => a.nodeIndex == nodeIndex);
        if (anchor != null)
            anchors.Add((nodeIndex, anchor));
        isPinned[nodeIndex] = anchor != null;
    }

    public void SimulatePhysics(float dt)
    {
        if (dt <= 0f || nodeCount == 0) return;

        collisionNormal ??= new float3[nodeCount];
        hadCollision ??= new bool[nodeCount];
        System.Array.Clear(hadCollision, 0, nodeCount);

        ApplyAnchors();
        VerletIntegrate(dt);

        // Repeatedly relax the distance constraints and re-project out of colliders each
        // iteration. This is what gives high fidelity without jitter - a single pass
        // per substep leaves visible stretch/pop under fast motion or many segments.
        // Collision handling here is POSITION-ONLY - velocity/restitution/friction are
        // applied exactly once below, after the loop has converged, not on every pass.
        for (int i = 0; i < constraintIterations; i++)
        {
            SolveDistanceConstraints();
            ApplyAnchors();
            ProjectOutOfColliders();
        }

        ApplyCollisionResponse();
    }

    private void ApplyAnchors()
    {
        foreach (var (nodeIndex, anchor) in anchors)
        {
            positions[nodeIndex] = anchor.position;
            previousPositions[nodeIndex] = anchor.position;
        }
    }

    private void VerletIntegrate(float dt)
    {
        float3 gravity = Physics.gravity;
        float dtSq = dt * dt;

        for (int i = 0; i < nodeCount; i++)
        {
            if (isPinned[i]) continue;

            float3 current = positions[i];
            float3 velocity = (current - previousPositions[i]) * (1f - velocityDamping);

            float speed = math.length(velocity);
            float maxStep = maxSpeed * dt;
            if (speed > maxStep && speed > 0f)
                velocity *= maxStep / speed;

            float3 next = current + velocity + gravity * dtSq;

            previousPositions[i] = current;
            positions[i] = next;
        }
    }

    private void SolveDistanceConstraints()
    {
        for (int i = 0; i < nodeCount - 1; i++)
        {
            bool aPinned = isPinned[i];
            bool bPinned = isPinned[i + 1];
            if (aPinned && bPinned) continue;

            float3 a = positions[i];
            float3 b = positions[i + 1];

            float3 delta = b - a;
            float dist = math.length(delta);
            if (dist < 1e-6f) continue;

            float error = (dist - restLength) / dist;
            float3 correction = delta * error * 0.5f;

            if (aPinned)
            {
                positions[i + 1] -= correction * 2f;
            }
            else if (bPinned)
            {
                positions[i] += correction * 2f;
            }
            else
            {
                positions[i] += correction;
                positions[i + 1] -= correction;
            }
        }
    }

    // Collider.ClosestPoint only supports Box/Sphere/Capsule and CONVEX MeshColliders.
    // Called on anything else (e.g. a raw, non-convex ground/terrain MeshCollider) it
    // silently returns the query point itself - which our solver was reading as "dead
    // center inside the collider" every iteration and shoving the node up by a full
    // radius each time. That degenerate case was the actual cause of the boiling.
    private static bool TryGetClosestPoint(Collider collider, Vector3 point, out Vector3 closest)
    {
        if (collider is MeshCollider mesh && !mesh.convex)
        {
            if (warnedColliders.Add(collider))
            {
                Debug.LogWarning(
                    $"CablePhysics: '{collider.name}' is a non-convex MeshCollider, which doesn't " +
                    "support accurate closest-point queries. Enable 'Convex' on it (fine for simple/" +
                    "flat shapes) or swap in a Box/Sphere/Capsule collider for anything the cable " +
                    "should collide with. Falling back to a coarse bounds-based approximation for now.",
                    collider);
            }
            
            closest = collider.bounds.ClosestPoint(point);
            return true;
        }

        closest = collider.ClosestPoint(point);
        return true;
    }

    // Pure position correction - pushes penetrating nodes out of colliders and records
    // the contact normal, but never touches previousPositions/velocity. Safe to call
    // repeatedly within the constraint-iteration loop without injecting energy.
    private void ProjectOutOfColliders()
    {
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

                Vector3 offset = position - closest;
                float distance = offset.magnitude;

                if (distance < radius)
                {
                    Vector3 direction = distance > 0.0001f ? offset / distance : Vector3.up;
                    float penetration = radius - distance;
                    position += direction * penetration;

                    collisionNormal[i] = direction;
                    hadCollision[i] = true;
                }
            }

            positions[i] = position;
        }
    }

    // Applies restitution + friction exactly once per substep, using the final resolved
    // position and the contact normal recorded during the iteration loop.
    private void ApplyCollisionResponse()
    {
        for (int i = 0; i < nodeCount; i++)
        {
            if (isPinned[i] || !hadCollision[i]) continue;

            float3 direction = collisionNormal[i];
            float3 velocity = positions[i] - previousPositions[i];

            float vDotN = math.dot(velocity, direction);
            if (vDotN < 0f)
                velocity -= direction * (vDotN * (1f + restitution));

            float3 tangentVel = velocity - direction * math.dot(velocity, direction);
            velocity -= tangentVel * friction;

            previousPositions[i] = positions[i] - velocity;
        }
    }
}