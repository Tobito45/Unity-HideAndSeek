using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

//rb.constraints =
//    RigidbodyConstraints.FreezeRotationX |
//    RigidbodyConstraints.FreezeRotationZ | RigidbodyConstraints.FreezePositionY;

public class AgentHider : Agent
{
    [Header("Movement")]
    public float moveSpeed = 6f;
    public float rotationSpeed = 120f;

    [Header("References")]
    public Transform seekerTransform;

    [Header("Stealth distances")]
    public float dangerDistance = 6f;
    public float catchDistance = 2f;

    [Header("Rewards")]
    public float distanceRewardScale = 0.05f;
    public float fovPenaltyScale = 0.08f;
    public float idlePenalty = -0.002f;
    public float caughtPenalty = -1f;

    [Header("Debug")]
    public bool drawGizmos = true;

    private Rigidbody rb;
    private Vector3 lastPos;
    private float prevDistance;

    // =============================
    // Episode
    // =============================
    public override void OnEpisodeBegin()
    {
        if (rb == null)
            rb = GetComponent<Rigidbody>();

        rb.constraints =
            RigidbodyConstraints.FreezeRotationX |
            RigidbodyConstraints.FreezeRotationZ | RigidbodyConstraints.FreezePositionY;


        transform.localPosition = new Vector3(
            Random.Range(-4f, 4f),
            transform.localPosition.y,
            Random.Range(-4f, 4f)
        );
        transform.localRotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        lastPos = transform.position;
        prevDistance = Vector3.Distance(transform.position, seekerTransform.position);
    }

    // =============================
    // Observations
    // =============================
    public override void CollectObservations(VectorSensor sensor)
    {
        // --- relative position to seeker (flat)
        Vector3 toSeeker = seekerTransform.position - transform.position;
        Vector3 toSeekerFlat = Vector3.ProjectOnPlane(toSeeker, Vector3.up);

        sensor.AddObservation(toSeekerFlat.x / 10f);
        sensor.AddObservation(toSeekerFlat.z / 10f);
        sensor.AddObservation(Mathf.Clamp01(toSeekerFlat.magnitude / 10f));

        // --- seeker forward (flat)
        Vector3 seekerForward = Vector3.ProjectOnPlane(seekerTransform.forward, Vector3.up).normalized;
        sensor.AddObservation(seekerForward.x);
        sensor.AddObservation(seekerForward.z);

        // --- how centered I am in seeker's FOV (-1 ... 1)
        Vector3 toHiderFromSeeker = (transform.position - seekerTransform.position);
        Vector3 toHiderFlat = Vector3.ProjectOnPlane(toHiderFromSeeker, Vector3.up).normalized;
        float fovDot = Vector3.Dot(seekerForward, toHiderFlat);
        sensor.AddObservation(fovDot);

        // --- my velocity (flat)
        Vector3 vel = Vector3.ProjectOnPlane(rb.linearVelocity, Vector3.up);
        sensor.AddObservation(vel.x / moveSpeed);
        sensor.AddObservation(vel.z / moveSpeed);

        // --- my orientation
        float ang = transform.eulerAngles.y * Mathf.Deg2Rad;
        sensor.AddObservation(Mathf.Cos(ang));
        sensor.AddObservation(Mathf.Sin(ang));
    }

    // =============================
    // Actions
    // =============================
    public override void OnActionReceived(ActionBuffers actions)
    {
        float moveX = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float moveZ = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float rot = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);

        Vector3 move = transform.right * moveX + transform.forward * moveZ;
        rb.MovePosition(rb.position + move * moveSpeed * Time.deltaTime);
        rb.MoveRotation(rb.rotation * Quaternion.Euler(0f, rot * rotationSpeed * Time.deltaTime, 0f));

        // =============================
        // Stealth rewards
        // =============================

        float curDist = Vector3.Distance(transform.position, seekerTransform.position);
        float deltaDist = curDist - prevDistance;

        // 1. reward for increasing distance
        AddReward(deltaDist * distanceRewardScale);

        // 2. penalty for being in seeker's FOV center
        Vector3 seekerForward = Vector3.ProjectOnPlane(seekerTransform.forward, Vector3.up).normalized;
        Vector3 toHider = Vector3.ProjectOnPlane(
            transform.position - seekerTransform.position,
            Vector3.up
        ).normalized;

        float fovDot = Vector3.Dot(seekerForward, toHider); // 1 = center
        float fovDanger = Mathf.Clamp01((fovDot + 1f) * 0.5f);
        AddReward(-fovDanger * fovPenaltyScale);

        // 3. idle penalty
        if (Vector3.Distance(transform.position, lastPos) < 0.01f)
            AddReward(idlePenalty);

        // 4. caught
        if (curDist < catchDistance && fovDot > 0.7f)
        {
            AddReward(caughtPenalty);
            EndEpisode();
            transform.parent.GetComponentInChildren<AgentWallsGrid>().EndEpisode();
        }

        prevDistance = curDist;
        lastPos = transform.position;
    }

    // =============================
    // Gizmos
    // =============================
    private void OnDrawGizmos()
    {
        if (!drawGizmos || seekerTransform == null)
            return;

        // danger radius
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position, dangerDistance);

        // line to seeker
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(transform.position, seekerTransform.position);

        // seeker forward
        Gizmos.color = Color.magenta;
        Gizmos.DrawLine(
            seekerTransform.position,
            seekerTransform.position + seekerTransform.forward * 3f
        );
    }

    public void WasCaught()
    {
        AddReward(caughtPenalty);
        EndEpisode();
    }

    // =============================
    // Wall = death
    // =============================
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.collider.CompareTag("Wall"))
        {
            AddReward(caughtPenalty);
            EndEpisode();

            transform.parent.GetComponentInChildren<AgentWallsGrid>().EndEpisode();
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Wall"))
        {
            AddReward(caughtPenalty);
            EndEpisode();

            transform.parent.GetComponentInChildren<AgentWallsGrid>().EndEpisode();
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