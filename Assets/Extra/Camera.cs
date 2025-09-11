using System;
using System.Numerics;
using UnityEngine;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;

public class Camera : MonoBehaviour
{
    private Transform[] targets;
    private int targetIndex = 0;
    [SerializeField] private Vector3 distance;
    [SerializeField] private float rotationSpeed;
    [SerializeField] private float linearSmoothing;
    [SerializeField] private float maxSpeed;
    [SerializeField] private float rotationSmoothing;
    private Vector3 velocity = Vector3.zero;

    private Vector3 originalPosition;
    private Quaternion originalRotation;
    private bool shouldFollow = false;
    
    void Start()
    {
        targets = Array.ConvertAll(FindObjectsByType<CarController>(FindObjectsSortMode.None), (a) => a.transform);
        originalPosition = transform.position;
        originalRotation = transform.rotation;
    }

    // Update is called once per frame
    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.F))
        {
            shouldFollow = !shouldFollow;
            if (!shouldFollow)
            {
                transform.position = originalPosition;
                transform.rotation = originalRotation;
            }
        }
    }

    void LateUpdate()
    {
        if (shouldFollow)
        {
            if (Input.GetKeyDown(KeyCode.RightArrow)) 
                targetIndex = (targetIndex + 1) % targets.Length;
            if (Input.GetKeyUp(KeyCode.LeftArrow)) 
                targetIndex = Math.Abs((targetIndex - 1) % targets.Length);
            FollowTarget();
        }
    }

    void FollowTarget()
    {
        
        Vector3 targetVector =  targets[targetIndex].position + targets[targetIndex].rotation * distance;
        transform.position = Vector3.SmoothDamp(transform.position, targetVector, ref velocity, linearSmoothing, maxSpeed);
        
        // Determine which direction to rotate towards
        Vector3 targetDirection = targets[targetIndex].position - transform.position;
        
        // The step size is equal to speed times frame time.
        float singleStep = rotationSpeed * Time.deltaTime;
        
        // Rotate the forward vector towards the target direction by one step
        Vector3 newDirection = Vector3.RotateTowards(transform.forward, targetDirection, singleStep, 0.0f);
        
        // Calculate a rotation a step closer to the target and applies rotation to this object
        Quaternion targetRotation = Quaternion.LookRotation(newDirection);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, rotationSmoothing * Time.deltaTime);
    }
}
