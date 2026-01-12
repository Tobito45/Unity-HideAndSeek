using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class AgentHider : Agent
{
    [Header("Movement")]
    [SerializeField] private float moveSpeed = 6f;
    [SerializeField] private float rotationSpeed = 120f;

    [Header("References")]
    public Transform seekerTransform;

    [Header("View")]
    public float viewRadius = 8f;
    [Range(0, 360)] public float viewAngle = 100f;
    public LayerMask obstacleMask;

    [Header("Distances")]
    public float dangerDistance = 5f;
    public float catchDistance = 2f;

    [Header("Rewards")]
    public float escapeRewardScale = 0.05f;
    public float closePenaltyScale = 0.08f;
    public float seenPenalty = -0.05f;
    public float caughtPenalty = -1.0f;
    public float idlePenalty = -0.001f;

    [Header("Debug")]
    [SerializeField] private bool drawDebugRays = true;
    [SerializeField] private bool drawGizmos = true;

    private Rigidbody rb;
    private float prevDist;
    private Vector3 prevPos;

    // =============================
    // Episode
    // =============================
    public override void OnEpisodeBegin()
    {
        if (rb == null) rb = GetComponent<Rigidbody>();

        transform.localPosition = new Vector3(
            Random.Range(-3f, 3f),
            transform.localPosition.y,
            Random.Range(-3f, 3f)
        );
        transform.localRotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        prevDist = Vector3.Distance(transform.position, seekerTransform.position);
        prevPos = transform.position;
    }

    // =============================
    // Observations
    // =============================
    public override void CollectObservations(VectorSensor sensor)
    {
        // position
        sensor.AddObservation(transform.localPosition.x / 10f);
        sensor.AddObservation(transform.localPosition.z / 10f);

        // direction and distance to seeker
        Vector3 toSeeker = seekerTransform.position - transform.position;
        sensor.AddObservation(toSeeker.x / 10f);
        sensor.AddObservation(toSeeker.z / 10f);
        sensor.AddObservation(Mathf.Clamp01(toSeeker.magnitude / 10f));

        // orientation
        float a = transform.eulerAngles.y * Mathf.Deg2Rad;
        sensor.AddObservation(Mathf.Cos(a));
        sensor.AddObservation(Mathf.Sin(a));

        // is seen by seeker
        sensor.AddObservation(IsSeenBySeeker(false) ? 1f : 0f);
    }

    // =============================
    // Actions
    // =============================
    public override void OnActionReceived(ActionBuffers actions)
    {
        float moveX = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float moveZ = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float rot = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);

        Vector3 move = transform.forward * moveZ + transform.right * moveX;
        rb.MovePosition(rb.position + move * moveSpeed * Time.deltaTime);
        rb.MoveRotation(rb.rotation * Quaternion.Euler(0f, rot * rotationSpeed * Time.deltaTime, 0f));

        float curDist = Vector3.Distance(transform.position, seekerTransform.position);

        // === 1. Reward only for actual escaping
        float delta = curDist - prevDist;
        if (delta > 0f)
            AddReward(delta * escapeRewardScale);
        else
            AddReward(delta * escapeRewardScale * 2f);

        // === 2. Exponential fear of proximity
        float danger = Mathf.Clamp01(1f - curDist / dangerDistance);
        AddReward(-danger * danger * closePenaltyScale);

        // === 3. Penalty for idling
        if (Vector3.Distance(transform.position, prevPos) < 0.01f)
            AddReward(idlePenalty);

        // === 4. If seeker sees — penalty
        if (IsSeenBySeeker(true))
            AddReward(seenPenalty);

        // === 5. Caught
        if (curDist < catchDistance && IsSeenBySeeker(true))
        {
            AddReward(caughtPenalty);
            EndEpisode();
        }

        // Debug rays
        if (drawDebugRays)
        {
            IsSeenBySeeker(true);
            HiderSeesSeeker(true);
        }

        prevDist = curDist;
        prevPos = transform.position;
    }

    // =============================
    // Vision
    // =============================
    private bool IsSeenBySeeker(bool draw)
    {
        Vector3 toHider = transform.position - seekerTransform.position;
        float dist = toHider.magnitude;

        if (dist > 10f) return false;
        if (Vector3.Angle(seekerTransform.forward, toHider.normalized) > 45f) return false;

        Vector3 origin = seekerTransform.position + Vector3.up * 0.3f;

        if (Physics.Raycast(origin, toHider.normalized, out RaycastHit hit, dist, obstacleMask))
        {
            if (draw && drawDebugRays)
                Debug.DrawRay(origin, toHider.normalized * hit.distance, Color.green);
            return false;
        }
        else
        {
            if (draw && drawDebugRays)
                Debug.DrawRay(origin, toHider.normalized * dist, Color.red);
            return true;
        }
    }

    private bool HiderSeesSeeker(bool draw)
    {
        Vector3 toSeeker = seekerTransform.position - transform.position;
        float dist = toSeeker.magnitude;

        if (dist > viewRadius) return false;
        if (Vector3.Angle(transform.forward, toSeeker.normalized) > viewAngle / 2f) return false;

        Vector3 origin = transform.position + Vector3.up * 0.3f;

        if (Physics.Raycast(origin, toSeeker.normalized, out RaycastHit hit, dist, obstacleMask))
        {
            if (draw && drawDebugRays)
                Debug.DrawRay(origin, toSeeker.normalized * hit.distance, Color.gray);
            return false;
        }
        else
        {
            if (draw && drawDebugRays)
                Debug.DrawRay(origin, toSeeker.normalized * dist, Color.cyan);
            return true;
        }
    }

    // =============================
    // Gizmos
    // =============================
    private void OnDrawGizmos()
    {
        if (!drawGizmos) return;

        // danger radius
        Gizmos.color = new Color(1f, 0f, 0f, 0.2f);
        Gizmos.DrawWireSphere(transform.position, dangerDistance);

        // view cone
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(transform.position, viewRadius);

        Vector3 a = DirFromAngle(-viewAngle / 2);
        Vector3 b = DirFromAngle(viewAngle / 2);
        Gizmos.DrawLine(transform.position, transform.position + a * viewRadius);
        Gizmos.DrawLine(transform.position, transform.position + b * viewRadius);
    }

    private Vector3 DirFromAngle(float angle)
    {
        angle += transform.eulerAngles.y;
        return new Vector3(Mathf.Sin(angle * Mathf.Deg2Rad), 0f, Mathf.Cos(angle * Mathf.Deg2Rad));
    }


    private void OnCollisionEnter(Collision collision)
    {
        if (collision.collider.CompareTag("Wall"))
        {
            AddReward(caughtPenalty);
            EndEpisode();

            FindAnyObjectByType<AgentWallsGrid>().EndEpisode();
        }
            //AddReward(-0.01f);
    }
    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Wall"))
        {
            AddReward(caughtPenalty);
            EndEpisode();

            FindAnyObjectByType<AgentWallsGrid>().EndEpisode();
        }
    }
}


/*
 * 
 *     private void OnCollisionEnter(Collision collision)
    {
        if (collision.collider.CompareTag("Wall"))
            AddReward(boundaryPenalty);
    }

    private void OnCollisionStay(Collision collision)
    {
        if (collision.collider.CompareTag("Wall"))
            AddReward(-0.005f);
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Wall"))
            AddReward(boundaryPenalty);
    }
*/