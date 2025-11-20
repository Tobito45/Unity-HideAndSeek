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

    [Header("Hider View Settings")]
    public float viewRadius = 8f;
    [Range(0, 360)] public float viewAngle = 100f;
    public LayerMask obstacleMask;

    [Header("Rewards")]
    public float rewardWhenSeeingSeeker = 0.01f;
    public float penaltyWhenSeenBySeeker = -0.02f;
    public float penaltyCaught = -1f;
    public float boundaryPenalty = -0.1f;
    public float rewardForLiving = 0.001f;

    private Vector3? lastSeenSeekerPos = null;
    private Rigidbody rb;

    public override void OnEpisodeBegin()
    {
        transform.localPosition = new Vector3(Random.Range(-3f, 3f), transform.localPosition.y, Random.Range(-3f, 3f));
        transform.localRotation = Quaternion.Euler(0, Random.Range(0, 360f), 0);

        if (rb == null) rb = GetComponent<Rigidbody>();
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        lastSeenSeekerPos = null;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition.x / 5f);
        sensor.AddObservation(transform.localPosition.z / 5f);

        if (lastSeenSeekerPos.HasValue)
        {
            Vector3 toLast = lastSeenSeekerPos.Value - transform.position;
            sensor.AddObservation(toLast.x / 10f);
            sensor.AddObservation(toLast.z / 10f);
        }
        else
        {
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
        }

        float myAngle = transform.eulerAngles.y * Mathf.Deg2Rad;
        sensor.AddObservation(Mathf.Cos(myAngle));
        sensor.AddObservation(Mathf.Sin(myAngle));
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float moveX = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float moveZ = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float rot = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);

        Vector3 move = transform.forward * moveZ + transform.right * moveX;
        //transform.localPosition += move * moveSpeed * Time.deltaTime;
        rb.MovePosition(rb.position + move * moveSpeed * Time.deltaTime);
        transform.localPosition = new Vector3(
            Mathf.Clamp(transform.localPosition.x, -10f, 10f),
            transform.localPosition.y,
            Mathf.Clamp(transform.localPosition.z, -10f, 10f)
        );

        transform.Rotate(Vector3.up, rot * rotationSpeed * Time.deltaTime);

        bool iSeeSeeker = SeekerInMyView();
        bool seekerSeesMe = IsSeenBySeeker();

        if (iSeeSeeker)
        {
            AddReward(rewardWhenSeeingSeeker);
            lastSeenSeekerPos = seekerTransform.position;
        }

        if (seekerSeesMe)
            AddReward(penaltyWhenSeenBySeeker);
        else
            AddReward(rewardForLiving);

        if (CanBeCaught())
        {
            AddReward(penaltyCaught);
            //EndEpisode();
        }
    }

    private bool SeekerInMyView()
    {
        if (seekerTransform == null) return false;

        Vector3 dir = (seekerTransform.position - transform.position).normalized;
        if (Vector3.Angle(transform.forward, dir) > viewAngle / 2f) return false;

        float dist = Vector3.Distance(transform.position, seekerTransform.position);
        if (dist > viewRadius) return false;

        if (!Physics.Raycast(transform.position + Vector3.up * 0.3f, dir, dist, obstacleMask))
            return true;

        return false;
    }

    private bool IsSeenBySeeker()
    {
        if (seekerTransform == null) return false;

        Vector3 toHider = (transform.position - seekerTransform.position);
        float dist = toHider.magnitude;
        if (dist > 10f) return false;

        if (Vector3.Angle(seekerTransform.forward, toHider.normalized) > 45f) return false;

        if (!Physics.Raycast(seekerTransform.position + Vector3.up * 0.3f, toHider.normalized, dist, obstacleMask))
            return true;

        return false;
    }

    private bool CanBeCaught()
    {
        return Vector3.Distance(transform.position, seekerTransform.position) < 2f && IsSeenBySeeker();
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.collider.CompareTag("Wall"))
        {
            AddReward(boundaryPenalty);
        }
    }

    private void OnCollisionStay(Collision collision)
    {
        if (collision.collider.CompareTag("Wall"))
            AddReward(-0.005f);
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(transform.position, viewRadius);

        Vector3 A = AngleToDir(-viewAngle / 2);
        Vector3 B = AngleToDir(viewAngle / 2);

        Gizmos.DrawLine(transform.position, transform.position + A * viewRadius);
        Gizmos.DrawLine(transform.position, transform.position + B * viewRadius);
    }

    private Vector3 AngleToDir(float angle)
    {
        angle += transform.eulerAngles.y;
        return new Vector3(Mathf.Sin(angle * Mathf.Deg2Rad), 0, Mathf.Cos(angle * Mathf.Deg2Rad));
    }
}