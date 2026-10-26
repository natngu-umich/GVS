using System;
using UnityEngine;
using TMPro;

public class WheelController : MonoBehaviour
{
    [Range(0, 180)]
    [SerializeField] private float angleClamp = 45f;
    [SerializeField] private Transform primaryHand; // Right Hand
    [SerializeField] private Transform secondaryHand; // Left Hand
    [SerializeField] private Transform wheel;
    [SerializeField] private Vector3 defaultLocalRotation;

    [SerializeField] private CarController controller;
    [SerializeField] private StartingManager startManager;

    [SerializeField] private TMP_Text positionText;
    [SerializeField] private LeaderboardManager leaderboard;
    [SerializeField] private CarData carData;

    // Track proximity separately from actual grabbing
    public bool leftHandInRange = false;
    public bool rightHandInRange = false;

    public bool leftHandGrabbed = false;
    public bool rightHandGrabbed = false;
    public float deadZone = 0.2f;
    public float tanAngle;

    // Stores the initial angle when the user first grabs the wheel so it doesn't snap
    private float initialGrabOffset = 0f;
    private bool wasGrabbedLastFrame = false;

    void Update()
    {
        // 1. Gas and Brake Logic
        if (startManager.raceStarted == false)
        {
            controller.braking(1f);
        }
        else
        {
            controller.accelerating(Input.GetAxis("XRI_Right_Trigger") * -1f);
            controller.braking(Input.GetAxis("XRI_Left_Trigger"));
        }

        // 2. Sticky Grab Logic
        // If in range and button pressed, grab. If button released, ungrab.
        if (leftHandInRange && Input.GetAxis("XRI_Left_Grip") > deadZone) leftHandGrabbed = true;
        if (Input.GetAxis("XRI_Left_Grip") <= deadZone) leftHandGrabbed = false;

        if (rightHandInRange && Input.GetAxis("XRI_Right_Grip") > deadZone) rightHandGrabbed = true;
        if (Input.GetAxis("XRI_Right_Grip") <= deadZone) rightHandGrabbed = false;

        // 3. Two-Handed Steering Math
        bool isCurrentlyGrabbed = leftHandGrabbed && rightHandGrabbed;

        if (isCurrentlyGrabbed)
        {
            // Create a vector pointing from the left hand to the right hand
            Vector3 handDirection = primaryHand.position - secondaryHand.position;

            // Convert that vector into the car's local space (so steering works when the car turns)
            Vector3 localHandDir = wheel.parent.InverseTransformDirection(handDirection);

            // Calculate the angle of the hands on the Y/Z plane (matching your original axis logic)
            float currentHandAngle = Mathf.Atan2(localHandDir.y, localHandDir.z) * Mathf.Rad2Deg;

            // Prevent snapping when first grabbing the wheel
            if (!wasGrabbedLastFrame)
            {
                initialGrabOffset = currentHandAngle - tanAngle;
            }

            // Apply offset and clamp
            float rawAngle = currentHandAngle - initialGrabOffset;
            tanAngle = Mathf.Clamp(rawAngle, -angleClamp, angleClamp);

            // Apply rotation and steer the car
            wheel.localRotation = Quaternion.Euler(defaultLocalRotation.x + tanAngle, defaultLocalRotation.y, defaultLocalRotation.z);
            controller.turnCar(tanAngle / angleClamp);
        }

        wasGrabbedLastFrame = isCurrentlyGrabbed;
    }
}