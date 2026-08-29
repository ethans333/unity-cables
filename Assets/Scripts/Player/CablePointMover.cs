using UnityEngine;

/// <summary>
/// Attach to a cable point to make it wander to random positions in world space.
/// Useful for stress-testing cable physics.
/// </summary>
public class CablePointMover : MonoBehaviour
{
    [SerializeField] private float range = 2f;
    [SerializeField] private float speed = 1.5f;
    [SerializeField] private float arrivalThreshold = 0.05f;

    private Vector3 origin;
    private Vector3 target;

    private void Start()
    {
        origin = transform.position;
        PickNewTarget();
    }

    private void Update()
    {
        transform.position = Vector3.Lerp(transform.position, target, speed * Time.deltaTime);

        if (Vector3.Distance(transform.position, target) < arrivalThreshold)
            PickNewTarget();
    }

    private void PickNewTarget()
    {
        target = origin + Random.insideUnitSphere * range;
    }
}
