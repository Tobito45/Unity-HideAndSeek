using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class MoveToGoalAgent : Agent
{
    [SerializeField]
    private Transform targetPosition;
    [SerializeField]
    private Material winMat, loseMat;
    [SerializeField]
    private MeshRenderer floorMeshRender;

    public override void OnEpisodeBegin()
    {
        transform.localPosition = new Vector3(Random.Range(-2.7f, +2f), 0, Random.Range(-3f, +2f));
        targetPosition.localPosition = new Vector3(Random.Range(2.4f, +8.3f), -0.26f, Random.Range(-2.62f, +2.66f));
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition); 
        sensor.AddObservation(targetPosition.localPosition); 
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float moveX = actions.ContinuousActions[0];
        float moveZ = actions.ContinuousActions[1];

        float moveSpeed = 6f;
        transform.localPosition += new Vector3(moveX, 0, moveZ) * Time.deltaTime * moveSpeed;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continiusActions = actionsOut.ContinuousActions;
        continiusActions[0] = Input.GetAxisRaw("Horizontal");
        continiusActions[1] = Input.GetAxisRaw("Vertical");
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.tag == "Goal")
        {
            SetReward(1f);
            EndEpisode();
            floorMeshRender.material = winMat;
        }

        if (other.tag == "Wall")
        {
            SetReward(-1f);
            EndEpisode();
            floorMeshRender.material = loseMat;
        }
    }
}
