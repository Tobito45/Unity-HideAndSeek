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
    private float viewGetRadius = 5f;
    [SerializeField]
    [Range(0, 360)]
    private float viewGetAngle = 90f;
    [SerializeField]
    private LayerMask obstacleMask;
    [SerializeField]
    public int rayCount = 50;

    [SerializeField]
    private float viewSeeRadius = 10f;
    [SerializeField]
    [Range(0, 360)]
    private float viewSeeAngle = 90f;

    private Mesh meshAreaGet, meshAreaSee;
    [SerializeField]
    private MeshFilter meshFilterAreaGet, meshFilterAreaSee;


    private Vector3? lastSeenTargetPosition = null;
    private bool hasGivenSightReward = false;
    private float forgetTime = 5f;
    private float lastSeenTime = -10f;

    private void Start()
    {
        meshAreaGet = new Mesh();
        meshAreaGet.name = "View Mesh Get";
        meshFilterAreaGet.mesh = meshAreaGet;

        meshAreaSee = new Mesh();
        meshAreaSee.name = "View Mesh See";
        meshFilterAreaSee.mesh = meshAreaSee;
    }

    public override void OnEpisodeBegin()
    {
        transform.localPosition = new Vector3(0, 0.26f * 3, 0);
        lastSeenTargetPosition = null;
        transform.localRotation = Quaternion.identity;
        targetPosition.localPosition = GetRandomPositionInCircle((floorMeshRender.gameObject.transform.localScale.x - 0.5f) / 2);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition);

        if (lastSeenTargetPosition.HasValue)
            sensor.AddObservation(lastSeenTargetPosition.Value);
        else
            sensor.AddObservation(Vector3.zero);
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
            lastSeenTargetPosition = targetPosition.localPosition;
            lastSeenTime = Time.time;

            if (!hasGivenSightReward)
            {
                SetReward(0.1f);
                hasGivenSightReward = true;
            }
        }

        if (lastSeenTargetPosition.HasValue && CanGetTarget())
        {
            SetReward(1f);
            EndEpisode();
            floorMeshRender.material = winMat;
        }

        if (hasGivenSightReward && Time.time - lastSeenTime > forgetTime)
        {
            lastSeenTargetPosition = null;
            hasGivenSightReward = false;
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
            //SetReward(1f);
            //EndEpisode();
            //floorMeshRender.material = winMat;
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
        DrawVisionCone(viewGetAngle, viewGetRadius, meshAreaGet);
        DrawVisionCone(viewSeeAngle, viewSeeRadius, meshAreaSee);
    }

    private void DrawVisionCone(float viewAngle, float radius, Mesh meshArea)
    {
        float angleStep = viewAngle / rayCount;
        float startAngle = -viewAngle / 2;

        List<Vector3> vertices = new List<Vector3>();
        List<int> triangles = new List<int>();

        vertices.Add(Vector3.zero);

        for (int i = 0; i <= rayCount; i++)
        {
            float angle = startAngle + angleStep * i;
            Vector3 dir = DirFromAngle(angle);
            Vector3 endPoint = transform.InverseTransformPoint(CastRay(dir, radius));
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

    Vector3 CastRay(Vector3 direction, float radius)
    {
        Vector3 worldOrigin = transform.position;
        if (Physics.Raycast(worldOrigin, direction, out RaycastHit hit, radius, obstacleMask))
        {
            return hit.point;
        }
        return worldOrigin + direction * radius;
    }


    private bool CanGetTarget()
    {
        Vector3 dirToTarget = (targetPosition.position - transform.position).normalized;
        float dstToTarget = Vector3.Distance(transform.position, targetPosition.position);

        if (Vector3.Angle(transform.forward, dirToTarget) < viewGetAngle / 2 && dstToTarget <= viewGetRadius)
            if (!Physics.Raycast(transform.position, dirToTarget, dstToTarget, obstacleMask))
                return true;

        return false;
    }

    private bool CanSeeTarget()
    {
        Vector3 dirToTarget = (targetPosition.position - transform.position).normalized;
        float dstToTarget = Vector3.Distance(transform.position, targetPosition.position);

        if (Vector3.Angle(transform.forward, dirToTarget) < viewSeeAngle / 2 && dstToTarget <= viewSeeRadius)
            if (!Physics.Raycast(transform.position, dirToTarget, dstToTarget, obstacleMask))
                return true;

        return false;
    }

    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position, viewGetRadius);

        Vector3 viewAngleA = DirFromAngle(-viewGetAngle / 2);
        Vector3 viewAngleB = DirFromAngle(viewGetAngle / 2);

        Gizmos.DrawLine(transform.position, transform.position + viewAngleA * viewGetRadius);
        Gizmos.DrawLine(transform.position, transform.position + viewAngleB * viewGetRadius);

        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position, viewSeeRadius);

        Vector3 viewSeeAngleA = DirFromAngle(-viewSeeAngle / 2);
        Vector3 viewSeeAngleB = DirFromAngle(viewSeeAngle / 2);

        Gizmos.DrawLine(transform.position, transform.position + viewSeeAngleA * viewSeeRadius);
        Gizmos.DrawLine(transform.position, transform.position + viewSeeAngleB * viewSeeRadius);

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
