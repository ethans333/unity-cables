using UnityEngine;
using UnityEngine.Serialization;

public class CablePoint : MonoBehaviour
{
    [FormerlySerializedAs("isAffectedByGravity")] [FormerlySerializedAs("affectedByGravity")] [SerializeField]
    public bool isAnchor;
}