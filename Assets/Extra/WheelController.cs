using UnityEngine;

public class WheelController : MonoBehaviour
{
    public bool steerable;
    public bool motorized;
    public Transform wheelModel;

    [HideInInspector] public WheelCollider wheelCollider;

    public void Awake()
    {
        wheelCollider = GetComponent<WheelCollider>();
    }
}