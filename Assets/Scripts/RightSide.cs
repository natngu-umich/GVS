using UnityEngine;

public class RightSide : MonoBehaviour
{
    [SerializeField] private WheelController wheel;

    private void OnTriggerEnter(Collider hand)
    {
        if (hand.gameObject.GetComponent<RHand>()) wheel.rightHandInRange = true;
    }
    private void OnTriggerExit(Collider hand)
    {
        if (hand.gameObject.GetComponent<RHand>()) wheel.rightHandInRange = false;
    }
}