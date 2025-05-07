using Grpc.Core;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.VisualScripting;
using UnityEngine;

public class AgentWalls : Agent
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

    [SerializeField]
    private GameObject _wallPrefab;
    private List<GameObject> _walls = new();
    private HashSet<Transform> _sawWalls = new();

    [SerializeField]
    private int wallCount = 3;

    private Vector3? lastSeenTargetPosition = null;
    private bool hasGivenSightReward = false;
    private float forgetTime = 5f;
    private float lastSeenTime = -10f;

    //TODO Freeze X & Z rotation of Agent's RigidBody? (Agent may crash into the wall and flip over)
    
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

        ResetWalls();
    }

    public void ResetWalls()
    {
        for (int i = _walls.Count - 1; i >= 0; i--)
            Destroy(_walls[i]);

        _walls.Clear();
        _sawWalls.Clear();

        float floorRadius = (floorMeshRender.gameObject.transform.localScale.x - 0.5f) / 2;
        int maxAttempts = 50;

        for (int i = 0; i < wallCount; i++)
        {
            bool placedWall = false;
            int attempts = 0;

            while (!placedWall && attempts < maxAttempts)
            {
                attempts++;
                float radius = floorRadius * 0.9f;
                Vector2 randomCircle = Random.insideUnitCircle * radius;
                Vector3 randomPosition = new Vector3(
                    randomCircle.x,
                    0.5f,
                    randomCircle.y
                );
                Vector3 randomScale = new Vector3(
                    Random.Range(2f, 6f),
                    1.0f,
                    1.0f
                );
                Quaternion randomRotation = Quaternion.Euler(0, Random.Range(0f, 360f), 0);

                if (IsPositionValidSimple(randomPosition, randomScale, randomRotation))
                {
                    GameObject cube = Instantiate(_wallPrefab, transform.parent);
                    cube.transform.localPosition = randomPosition;
                    cube.transform.localScale = randomScale;
                    cube.transform.rotation = randomRotation;
                    _walls.Add(cube);
                    placedWall = true;
                    cube.layer = LayerMask.NameToLayer("LocalWall");
                }
            }
        }
    }

    private bool IsPositionValidSimple(Vector3 position, Vector3 scale, Quaternion rotation)
    {
        /*Vector3 agentMin = transform.localPosition - transform.localScale / 2f;
        Vector3 agentMax = transform.localPosition + transform.localScale / 2f;
        Vector3 targetMin = targetPosition.localPosition - targetPosition.localScale / 2f;
        Vector3 targetMax = targetPosition.localPosition + targetPosition.localScale / 2f;
        Vector3 wallMin = position - scale / 2f;
        Vector3 wallMax = position + scale / 2f;

        float dx_wallAgent = Mathf.Max(0, Mathf.Max(wallMin.x - agentMax.x, agentMin.x - wallMax.x));
        float dy_wallAgent = Mathf.Max(0, Mathf.Max(wallMin.y - agentMax.y, agentMin.y - wallMax.y));
        float dz_wallAgent = Mathf.Max(0, Mathf.Max(wallMin.z - agentMax.z, agentMin.z - wallMax.z));
        float minDistanceFromAgent = Mathf.Sqrt(dx_wallAgent * dx_wallAgent + dy_wallAgent * dy_wallAgent + dz_wallAgent * dz_wallAgent);
        if (Vector3.Distance(position, transform.localPosition) < minDistanceFromAgent + 1.0f)
            return false;

        float dx_wallTarget = Mathf.Max(0, Mathf.Max(wallMin.x - targetMax.x, targetMin.x - wallMax.x));
        float dy_wallTarget = Mathf.Max(0, Mathf.Max(wallMin.y - targetMax.y, targetMin.y - wallMax.y));
        float dz_wallTarget = Mathf.Max(0, Mathf.Max(wallMin.z - targetMax.z, targetMin.z - wallMax.z));
        float minDistanceFromTarget = Mathf.Sqrt(dx_wallTarget * dx_wallTarget + dy_wallTarget * dy_wallTarget + dz_wallTarget * dz_wallTarget);
        if (Vector3.Distance(position, targetPosition.localPosition) < minDistanceFromTarget + 1.0f)
            return false;*/

        float minDistanceFromAgent = 0.5f;
        float actualDistance = Vector3.Distance(position, transform.localPosition);
        if (actualDistance < minDistanceFromAgent + (scale.magnitude / 2) + (transform.localScale.magnitude / 2))
            return false;

        float minDistanceFromTarget = 0.5f;
        float actualTargetDistance = Vector3.Distance(position, targetPosition.localPosition);
        if (actualTargetDistance < minDistanceFromTarget + (scale.magnitude / 2) + (targetPosition.localScale.magnitude / 2))
            return false;

        foreach (GameObject existingWall in _walls)
        {
            float minDistanceBetweenWalls = 1.0f;
            float combinedExtent = (scale.magnitude + existingWall.transform.localScale.magnitude) / 2;

            if (Vector3.Distance(position, existingWall.transform.localPosition) < minDistanceBetweenWalls + combinedExtent)
            {
                return false;
            }
        }
        return true;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition);

        if (lastSeenTargetPosition.HasValue)
            sensor.AddObservation(lastSeenTargetPosition.Value);
        else
            sensor.AddObservation(Vector3.zero);

        if(_sawWalls.Count > 0)
        {
            foreach (Transform v in _sawWalls)
            {
                sensor.AddObservation(v.localPosition);
                sensor.AddObservation(v.localScale);
            }
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(Vector3.zero);
        }
    }



    public override void OnActionReceived(ActionBuffers actions)
    {
        // With SetReward you set the reward of a specific step during learning. With AddReward you add a value to the current reward value of that step.

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
        } else
        {
            AddReward(-0.2f);
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
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;
        //continiusActions[0] = Input.GetAxisRaw("Horizontal");
        //continiusActions[1] = Input.GetAxisRaw("Vertical");
        //continiusActions[2] = Input.GetKey(KeyCode.Q) ? -1f : Input.GetKey(KeyCode.E) ? 1f : 0f;

        if (Input.GetKey(KeyCode.R))
            EndEpisode();

        float rotation = 0f;
        if (Input.GetKey(KeyCode.A))
            rotation = -1f;
        else if (Input.GetKey(KeyCode.D))
            rotation = 1f;

        float moveForward = 0f;
        if (Input.GetKey(KeyCode.W))
            moveForward = 1f;
        else if (Input.GetKey(KeyCode.S))
            moveForward = -1f;

        float moveX = moveForward * Mathf.Sin(transform.eulerAngles.y * Mathf.Deg2Rad);
        float moveZ = moveForward * Mathf.Cos(transform.eulerAngles.y * Mathf.Deg2Rad);

        continuousActions[0] = moveX;
        continuousActions[1] = moveZ;
        continuousActions[2] = rotation;
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.tag == "Goal")
        {
            // Reward given for reaching target has to be larger than reward obtained during episode run
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
        //Vector3 dirToTarget = (targetPosition.position - transform.position).normalized;
        //float dstToTarget = Vector3.Distance(transform.position, targetPosition.position);
        int rayCount = 20;
        float angleStep = viewSeeAngle / (rayCount - 1); 
        float startAngle = -viewSeeAngle / 2;

        bool sawGoal = false;

        for (int i = 0; i < rayCount; i++)
        {
            float currentAngle = startAngle + angleStep * i;
            Vector3 dir = Quaternion.Euler(0, currentAngle, 0) * transform.forward;

            if (Physics.Raycast(transform.position, dir, out RaycastHit hit, viewSeeRadius, obstacleMask))
            {
                Debug.DrawRay(transform.position, dir * hit.distance, Color.red);
                if (hit.collider.CompareTag("LocalWall"))
                {
                    if (!_sawWalls.Contains(hit.transform))
                        _sawWalls.Add(hit.transform);
                    continue;
                }
                else if (hit.collider.CompareTag("Goal"))
                    sawGoal = true;
            }
            else
            {
                Debug.DrawRay(transform.position, dir * viewSeeRadius, Color.green); 
            }
        }
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
