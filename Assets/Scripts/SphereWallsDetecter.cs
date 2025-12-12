using UnityEngine;

public class SphereWallsDetecter : MonoBehaviour
{
    [SerializeField]
    private AgentWalls _agentWalls;

    [SerializeField]
    private AgentWallsGrid _agentGridWalls;

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.CompareTag("LocalWall"))
        {
            //_agentGridWalls?.ResetWalls();
            _agentWalls?.ResetWalls();
        }
    }
}
