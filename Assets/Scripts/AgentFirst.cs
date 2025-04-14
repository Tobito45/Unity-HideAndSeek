using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;



public class AgentFirst : Agent
{
    [SerializeField]
    private Transform targetPosition;
    [SerializeField]
    private Material winMat, loseMat;
    [SerializeField]
    private MeshRenderer floorMeshRender;
    [SerializeField]
    private float moveSpeed = 6f;
    [SerializeField] 
    private float rotationSpeed = 60f;

    [SerializeField]
    private float viewRadius = 5f;
    [SerializeField]
    [Range(0, 360)]
    private float viewAngle = 90f;
    [SerializeField]
    private LayerMask obstacleMask;
    [SerializeField]
    public int rayCount = 50;

    private Mesh meshArea;
    [SerializeField]
    private MeshFilter meshFilterArea;

    private void Start()
    {
        meshArea = new Mesh();
        meshArea.name = "View Mesh";
       // meshFilterArea = GetComponent<MeshFilter>();
        meshFilterArea.mesh = meshArea;
    }

    public override void OnEpisodeBegin()
    {
        transform.localPosition = new Vector3(0, 0.26f * 3, 0);
        transform.rotation = Quaternion.identity;
        targetPosition.localPosition = GetRandomPositionInCircle((floorMeshRender.gameObject.transform.localScale.x - 0.5f) / 2);
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
        float rotationY = actions.ContinuousActions[2];

        transform.localPosition += new Vector3(moveX, 0, moveZ) * Time.deltaTime * moveSpeed;

        transform.Rotate(Vector3.up, rotationY * rotationSpeed * Time.deltaTime);

        if (CanSeeTarget())
        {
            SetReward(1f);
            EndEpisode();
            floorMeshRender.material = winMat;
        }
    }
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continiusActions = actionsOut.ContinuousActions;
        continiusActions[0] = Input.GetAxisRaw("Horizontal");
        continiusActions[1] = Input.GetAxisRaw("Vertical");
        continiusActions[2] = Input.GetKey(KeyCode.Q) ? -1f : Input.GetKey(KeyCode.E) ? 1f : 0f;
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

    private void LateUpdate()
    {
        DrawVisionCone();
    }

    void DrawVisionCone()
    {
        float angleStep = viewAngle / rayCount;
        float startAngle = -viewAngle / 2;

        List<Vector3> vertices = new List<Vector3>();
        List<int> triangles = new List<int>();

        vertices.Add(Vector3.zero); // центр

        for (int i = 0; i <= rayCount; i++)
        {
            float angle = startAngle + angleStep * i;
            Vector3 dir = DirFromAngle(angle);
            Vector3 endPoint = transform.InverseTransformPoint(CastRay(dir));
            vertices.Add(endPoint);
        }

        for (int i = 1; i < vertices.Count - 1; i++)
        {
            triangles.Add(0);
            triangles.Add(i);
            triangles.Add(i + 1);
        }

        meshArea.Clear();
        meshArea.vertices = vertices.ToArray();
        meshArea.triangles = triangles.ToArray();
        meshArea.RecalculateNormals();
    }

    Vector3 CastRay(Vector3 direction)
    {
        Vector3 worldOrigin = transform.position;
        if (Physics.Raycast(worldOrigin, direction, out RaycastHit hit, viewRadius, obstacleMask))
        {
            return hit.point;
        }
        return worldOrigin + direction * viewRadius;
    }


    private bool CanSeeTarget()
    {
        Vector3 dirToTarget = (targetPosition.position - transform.position).normalized;
        float dstToTarget = Vector3.Distance(transform.position, targetPosition.position);

        if (Vector3.Angle(transform.forward, dirToTarget) < viewAngle / 2 && dstToTarget <= viewRadius)
            if (!Physics.Raycast(transform.position, dirToTarget, dstToTarget, obstacleMask))
                return true;

        return false;
    }

    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position, viewRadius);

        Vector3 viewAngleA = DirFromAngle(-viewAngle / 2);
        Vector3 viewAngleB = DirFromAngle(viewAngle / 2);

        Gizmos.DrawLine(transform.position, transform.position + viewAngleA * viewRadius);
        Gizmos.DrawLine(transform.position, transform.position + viewAngleB * viewRadius);

    }

    private Vector3 DirFromAngle(float angleInDegrees)
    {
        angleInDegrees += transform.eulerAngles.y;
        return new Vector3(Mathf.Sin(angleInDegrees * Mathf.Deg2Rad), 0, Mathf.Cos(angleInDegrees * Mathf.Deg2Rad));
    }

    private Vector3 GetRandomPositionInCircle(float radius)
    {
        Vector2 randomCircle = Random.insideUnitCircle * radius;
        Vector3 center = new Vector3(0f, 0.26f * 3f, 0f);
        Vector3 finalPosition = new Vector3(randomCircle.x, center.y, randomCircle.y);
        return finalPosition;
    }
}
