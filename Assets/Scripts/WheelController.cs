using System;
using UnityEngine;
using TMPro;

public class WheelController : MonoBehaviour
{
    [Header("Steering Settings")]
    [Range(0, 360)]
    [SerializeField] private float angleClamp = 45f;
    [Tooltip("Toggle this if turning the wheel left makes the wheel/car go right.")]
    [SerializeField] private bool invertSteering = true;
    [Tooltip("Toggle this if the steering wheel rotates the right way, but the car tyres turn the opposite way.")]
    [SerializeField] private bool invertCarTyres = false;

    [Header("Core Transforms")]
    [SerializeField] private Transform primaryHand;   // Right Tracked Controller (has RHand & Collider)
    [SerializeField] private Transform secondaryHand; // Left Tracked Controller (has LHand & Collider)
    [SerializeField] private Transform wheel;         // The rotating steering wheel mesh
    [SerializeField] private Vector3 defaultLocalRotation;

    [Header("Hand Visual Locking")]
    [Tooltip("The 3D mesh/model of your Right Hand (must be a child of primaryHand, NOT primaryHand itself).")]
    [SerializeField] private Transform rightHandVisual;
    [Tooltip("The 3D mesh/model of your Left Hand (must be a child of secondaryHand, NOT secondaryHand itself).")]
    [SerializeField] private Transform leftHandVisual;
    [Tooltip("Empty GameObject parented to the wheel rim at the 3 o'clock position.")]
    [SerializeField] private Transform rightSnapPoint;
    [Tooltip("Empty GameObject parented to the wheel rim at the 9 o'clock position.")]
    [SerializeField] private Transform leftSnapPoint;

    [Header("Game References")]
    [SerializeField] private CarController controller;
    [SerializeField] private StartingManager startManager;
    [SerializeField] private TMP_Text positionText;
    [SerializeField] private LeaderboardManager leaderboard;
    [SerializeField] private CarData carData;

    // Proximity flags triggered by LeftSide.cs and RightSide.cs
    [HideInInspector] public bool leftHandInRange = false;
    [HideInInspector] public bool rightHandInRange = false;

    public bool leftHandGrabbed = false;
    public bool rightHandGrabbed = false;
    public float deadZone = 0.2f;
    public float tanAngle = 0f;

    // Internal reference frame & frame-to-frame angle tracking
    private Transform wheelPivot;
    private float prevLeftAngle = 0f;
    private float prevRightAngle = 0f;
    private bool wasLeftGrabbed = false;
    private bool wasRightGrabbed = false;

    // Saved resting positions for the visual hand meshes
    private Vector3 leftVisualOrigPos;
    private Quaternion leftVisualOrigRot;
    private Vector3 rightVisualOrigPos;
    private Quaternion rightVisualOrigRot;

    void Start()
    {
        // 1. Automatically create a stationary reference pivot at the wheel's exact neutral pose.
        // Because it never rotates with the wheel, its local Y-Z plane stays completely stable!
        GameObject pivotObj = new GameObject("WheelReferencePivot_Auto");
        wheelPivot = pivotObj.transform;
        wheelPivot.SetParent(wheel.parent, false);
        wheelPivot.localPosition = wheel.localPosition;
        wheelPivot.localRotation = Quaternion.Euler(defaultLocalRotation);
        wheelPivot.localScale = Vector3.one;

        // 2. Save the original local offset of the visual hand meshes so they return cleanly on release
        if (leftHandVisual != null)
        {
            leftVisualOrigPos = leftHandVisual.localPosition;
            leftVisualOrigRot = leftHandVisual.localRotation;
        }

        if (rightHandVisual != null)
        {
            rightVisualOrigPos = rightHandVisual.localPosition;
            rightVisualOrigRot = rightHandVisual.localRotation;
        }
    }

    void Update()
    {
        // 1. Gas and Brake Logic
        if (!startManager.raceStarted)
        {
            controller.braking(1f);
        }
        else
        {
            // Mathf.Abs ensures triggers work whether Unity maps them to [0..1] or [-1..0]
            controller.accelerating(Mathf.Abs(Input.GetAxis("XRI_Right_Trigger")));
            controller.braking(Mathf.Abs(Input.GetAxis("XRI_Left_Trigger")));
        }

        // 2. Sticky Grab Logic
        // Once grabbed, the hand stays grabbed until you release the Grip button—even if your real hand pulls away from the collider!
        if (leftHandInRange && Input.GetAxis("XRI_Left_Grip") > deadZone) leftHandGrabbed = true;
        if (Input.GetAxis("XRI_Left_Grip") <= deadZone) leftHandGrabbed = false;

        if (rightHandInRange && Input.GetAxis("XRI_Right_Grip") > deadZone) rightHandGrabbed = true;
        if (Input.GetAxis("XRI_Right_Grip") <= deadZone) rightHandGrabbed = false;

        // 3. One- or Two-Handed Steering Math
        HandleSteering();
    }

    private void HandleSteering()
    {
        float totalDeltaAngle = 0f;
        int activeHands = 0;

        // Track Left Hand rotation around the wheel center
        if (leftHandGrabbed)
        {
            float currentLeftAngle = GetHandAngle(secondaryHand.position);
            if (wasLeftGrabbed)
            {
                totalDeltaAngle += Mathf.DeltaAngle(prevLeftAngle, currentLeftAngle);
                activeHands++;
            }
            prevLeftAngle = currentLeftAngle;
        }

        // Track Right Hand rotation around the wheel center
        if (rightHandGrabbed)
        {
            float currentRightAngle = GetHandAngle(primaryHand.position);
            if (wasRightGrabbed)
            {
                totalDeltaAngle += Mathf.DeltaAngle(prevRightAngle, currentRightAngle);
                activeHands++;
            }
            prevRightAngle = currentRightAngle;
        }

        // Apply rotation if at least one hand has been holding since the previous frame
        if (activeHands > 0)
        {
            float averageDelta = totalDeltaAngle / activeHands;
            float dirMultiplier = invertSteering ? -1f : 1f;

            tanAngle += averageDelta * dirMultiplier;
            tanAngle = Mathf.Clamp(tanAngle, -angleClamp, angleClamp);

            // Rotate the visual steering wheel around its local X axis
            wheel.localRotation = Quaternion.Euler(defaultLocalRotation) * Quaternion.Euler(tanAngle, 0f, 0f);

            // Turn the car's front tyres
            float normalizedSteer = (tanAngle / angleClamp) * (invertCarTyres ? -1f : 1f);
            controller.turnCar(normalizedSteer);
        }

        wasLeftGrabbed = leftHandGrabbed;
        wasRightGrabbed = rightHandGrabbed;
    }

    /// <summary>
    /// Converts the controller's world position into the stationary pivot's local space
    /// and measures the angle around the X-axis (on the Y-Z plane).
    /// </summary>
    private float GetHandAngle(Vector3 handWorldPos)
    {
        Vector3 localPos = wheelPivot.InverseTransformPoint(handWorldPos);
        return Mathf.Atan2(localPos.y, localPos.z) * Mathf.Rad2Deg;
    }

    /// <summary>
    /// Runs after Update and animation passes to visually lock the hand meshes to the wheel rim
    /// without interfering with the physical VR controller tracking.
    /// </summary>
    void LateUpdate()
    {
        if (leftHandVisual != null)
        {
            if (leftHandGrabbed && leftSnapPoint != null)
            {
                leftHandVisual.position = leftSnapPoint.position;
                leftHandVisual.rotation = leftSnapPoint.rotation;
            }
            else
            {
                leftHandVisual.localPosition = leftVisualOrigPos;
                leftHandVisual.localRotation = leftVisualOrigRot;
            }
        }

        if (rightHandVisual != null)
        {
            if (rightHandGrabbed && rightSnapPoint != null)
            {
                rightHandVisual.position = rightSnapPoint.position;
                rightHandVisual.rotation = rightSnapPoint.rotation;
            }
            else
            {
                rightHandVisual.localPosition = rightVisualOrigPos;
                rightHandVisual.localRotation = rightVisualOrigRot;
            }
        }
    }
}