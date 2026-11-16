using UnityEngine;
using UnityEngine.XR;

/// <summary>
/// Test input for GVSController: the VR controller thumbstick sets the acceleration by hand.
/// Stick left / right = acceleration left / right; stick centered = zero.
/// Put this on the same GameObject as GVSController (or drag the controller into "Gvs").
/// </summary>
public class GVSJoystickTest : MonoBehaviour
{
    public GVSController gvs;
    public XRNode hand = XRNode.RightHand;
    [Tooltip("Only output while the grip button is held. Letting go drops the signal to zero.")]
    public bool requireGrip = true;
    [Range(0f, 0.5f)] public float stickDeadzone = 0.1f;

    [Header("Read-Only")]
    public bool controllerFound;
    public float stickX;

    void OnEnable()
    {
        if (gvs == null) gvs = GetComponent<GVSController>();
        if (gvs != null) gvs.manualMode = true;
    }

    void OnDisable()
    {
        if (gvs != null)
        {
            gvs.manualInput = 0f;
            gvs.manualMode = false;
        }
    }

    void Update()
    {
        if (gvs == null) return;

        float x = 0f;
        InputDevice device = InputDevices.GetDeviceAtXRNode(hand);
        controllerFound = device.isValid;

        if (controllerFound && device.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 stick))
        {
            bool held = true;
            if (requireGrip)
                held = device.TryGetFeatureValue(CommonUsages.gripButton, out bool grip) && grip;

            if (held && Mathf.Abs(stick.x) > stickDeadzone) x = stick.x;
        }

        stickX = x;
        gvs.manualInput = x;
    }
}