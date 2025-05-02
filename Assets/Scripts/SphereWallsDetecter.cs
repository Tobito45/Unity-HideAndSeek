using UnityEngine;

public class SphereWallsDetecter : MonoBehaviour
{
    [SerializeField]
    private AgentWalls _agentWalls;

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.CompareTag("LocalWall"))
            _agentWalls.ResetWalls();
    }
}
