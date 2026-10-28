using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using UnityEngine.XR;
public class CarController : MonoBehaviour
{
    [System.Serializable]
    public class wheel{
        public WheelCollider rightTyre;
        public WheelCollider leftTyre;
        public bool motor;
        public bool turnable;
    }

    [SerializeField] private TMP_Text stateText;
    [SerializeField] private float maxAccelTorqueN = 600f;
    [SerializeField] private float maxBrakeTorqueN = 600f;
    [SerializeField] private float maxAngle = 25f;
    [SerializeField] private List<wheel> axles = new List<wheel>();


    [SerializeField] private TMP_Text positionText;
    [SerializeField] private TMP_Text lapInfo;
    [SerializeField] private LeaderboardManager leaderboard;
    [SerializeField] private RaceControl raceControl;
    [SerializeField] private CarData carData;
    [SerializeField] private GameObject endCam;
    [SerializeField] private GameObject carCam;
    [SerializeField] private Transform player;
    private bool isReversed = false;
    // Applies the rotation from the wheel into the cars wheels
    public void turnCar(float wheelAngle){
        float angleToTurn = wheelAngle*maxAngle;
        foreach(wheel axle in axles){
            if(axle.turnable){
                axle.leftTyre.steerAngle = angleToTurn;
                axle.rightTyre.steerAngle = angleToTurn;
            }
        }
    }

    // Takes the acceleration paddle is mapped from 0-1 so the acceleration is % of max torque
    public void accelerating(float pedalAmount){
        float torqueProduced = maxAccelTorqueN*pedalAmount;
        if(isReversed){
            torqueProduced *= -1f;
        }
        foreach(wheel axle in axles){
            if(axle.motor){
                axle.rightTyre.motorTorque = torqueProduced;
                axle.leftTyre.motorTorque = torqueProduced;

            }
        }
    }
    // Brake (decelarate) paddle is expressed between 0 and 1 so braking is % of max brake force
    public void braking(float brakePedal){
        float brakeAmount = maxBrakeTorqueN*brakePedal;
        foreach(wheel axle in axles){
            axle.rightTyre.brakeTorque = brakeAmount;
            axle.leftTyre.brakeTorque = brakeAmount;

        }
    }
    private void Start() {
        endCam.SetActive(false);
        carCam.SetActive(true);
    }

    private bool wasReversePressed = false;
    private bool wasResetPressed = false;
    private void Update()
    {
        // Updates the position and lap the player car is on
        positionText.text = leaderboard.leaderboardPosition(carData.carName).ToString() + "/" + leaderboard.carCp.Count.ToString();
        lapInfo.text = "Lap:\n" + carData.numLaps.ToString() + "/" + raceControl.maxLaps.ToString();

        if (carData.isFinished == true)
        {
            endCam.SetActive(true);
            carCam.SetActive(false);
        }

        // Unrolls car if it is rolled over
        // 1. Get the physical left VR controller directly
        InputDevice leftController = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);

        // 2. Read the raw state of the "X" button (Left Primary)
        bool isLeftPrimaryPressing = false;
        if (leftController.isValid)
        {
            leftController.TryGetFeatureValue(CommonUsages.primaryButton, out isLeftPrimaryPressing);
        }

        // 3. Check if either the VR button or the keyboard Jump button is being pressed
        bool isResetPressing = isLeftPrimaryPressing || Input.GetButton("Jump");

        // 4. Trigger the reset ONLY on the initial press (mimicking GetButtonDown)
        if (isResetPressing && !wasResetPressed)
        {
            player.position = carData.lastCp.position;
            player.rotation = Quaternion.Euler(carData.lastCp.eulerAngles.x, player.eulerAngles.y, carData.lastCp.eulerAngles.z);
            Debug.Log("Car unrolled to last checkpoint.");
        }

        // 5. Save the state for the next frame
        wasResetPressed = isResetPressing;



        // Foolproof Reverse Toggle (Removed the crash-causing GetAxis check)
        // 1. Get the physical right VR controller directly
        InputDevice rightController = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);

        // 2. Read the raw, un-cached state of the "A" button directly from the hardware
        bool isPressing = false;
        if (rightController.isValid)
        {
            rightController.TryGetFeatureValue(CommonUsages.primaryButton, out isPressing);
        }

        if (isPressing && !wasReversePressed)
        {
            isReversed = !isReversed;
            stateText.text = isReversed ? "R" : "F";
        }

        wasReversePressed = isPressing;
    }
}
