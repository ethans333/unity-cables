using System.Collections.Generic;
using UnityEngine;

public class CablePhysics
{
    private List<ControlPoint> controlPoints;
    
    private float radius;
    private float restitution;
    private float friction;
    
    private const float maxVelocity = 5f;
    
    private LayerMask collisionMask = ~0;
    
    private static readonly Collider[] overlapBuffer = new Collider[16];
    
    public CablePhysics(List<ControlPoint> controlPoints, float radius, float restitution, float friction)
    {
        this.controlPoints = controlPoints;
        this.radius = radius;
        this.restitution = restitution;
        this.friction = friction;
    }

    public void SimulatePhysics(float deltaTime)
    {
        Integrate(deltaTime);
        ResolveCollisions();
    }
    
    private void ResolveCollisions()
    {
        foreach (ControlPoint point in controlPoints)
        {
            if (point.IsFixed) continue;

            Vector3 position = point.transform.position;

            int count = Physics.OverlapSphereNonAlloc(
                position, radius, overlapBuffer, collisionMask, QueryTriggerInteraction.Ignore);

            for (int i = 0; i < count; i++)
            {
                Collider other = overlapBuffer[i];

                // ClosestPoint returns the point itself if position is already inside
                // a convex collider - fine for our purposes, but be aware for concave meshes.
                Vector3 closest = other.ClosestPoint(position);
                Vector3 offset = position - closest;
                float distance = offset.magnitude;

                if (distance < radius)
                {
                    Vector3 direction = distance > 0.0001f
                        ? offset / distance
                        : Vector3.up; // fallback if we're exactly on the surface point

                    float penetration = radius - distance;
                    position += direction * penetration;

                    // Kill/bounce velocity along the normal
                    float vDotN = Vector3.Dot(point.velocity, direction);
                    if (vDotN < 0f)
                        point.velocity -= direction * (vDotN * (1f + restitution));

                    // Damp tangential velocity (friction)
                    Vector3 tangentVel = point.velocity - direction * Vector3.Dot(point.velocity, direction);
                    point.velocity -= tangentVel * friction;
                }
            }

            point.transform.position = position;
        }
    }
    
    private void Integrate(float dt)
    {
        foreach (ControlPoint point in controlPoints)
        {
            if (point.IsFixed) continue;

            point.velocity += Physics.gravity * dt;
            point.velocity = Vector3.ClampMagnitude(point.velocity, maxVelocity);
            point.transform.position += point.velocity * dt;
        }
    }
}