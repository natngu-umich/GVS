using UnityEngine;

public class LeftSide : MonoBehaviour
{
    [SerializeField] private WheelController wheel;

    private void OnTriggerEnter(Collider hand)
    {
        if (hand.gameObject.GetComponent<LHand>()) wheel.leftHandInRange = true;
    }
    private void OnTriggerExit(Collider hand)
    {
        if (hand.gameObject.GetComponent<LHand>()) wheel.leftHandInRange = false;
    }
}