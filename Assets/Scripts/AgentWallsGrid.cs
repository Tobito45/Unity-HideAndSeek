using System.Collections.Generic;
using TMPro;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

/// <summary>
/// Seeker agent: compact heatmap-based seeker with two view zones:
/// - SeeZone (wide, updates heatmap)
/// - GetZone  (small, means "caught")
/// Observation size is fixed and computed: gridSize*gridSize + 2 + 2 + 2 + 1 = 232 (for gridSize=15)
/// </summary>
public class AgentWallsGrid : Agent
{
    [Header("References")]
    [SerializeField] private Transform targetTransform; // the Hider (sphere)
    [SerializeField] private MeshRenderer floorMeshRender;
    [SerializeField] private GameObject wallPrefab;

    [Header("Movement")]
    [SerializeField] private float moveSpeed = 4f;
    [SerializeField] private float rotationSpeed = 120f;
    private Rigidbody rb;

    [Header("Grid / Heatmap")]
    [SerializeField] private int gridSize = 15;
    private Dictionary<Vector2Int, CellMemory> gridMemory = new();
    [SerializeField] private float gridWorldSize = 10f; // side length of floor in Unity units (should match floor scale)
    private Vector3 gridOrigin; // bottom-left world point

    [Header("Vision")]
    [SerializeField] private float seeRadius = 10f;
    [SerializeField][Range(0, 360)] private float seeAngle = 90f;
    [SerializeField] private float getRadius = 4f;
    [SerializeField][Range(0, 360)] private float getAngle = 60f;
    [SerializeField] private LayerMask obstacleMask;
    [SerializeField] private int raysForSee = 30;

    [Header("Heatmap dynamics")]
    [SerializeField][Range(0f, 1f)] private float diffusionRate = 0.12f; // how much spreads per step
    [SerializeField] private float visibilityClearEpsilon = 0.001f; // threshold

    [Header("Rewards")]
    [SerializeField] private float timePenalty = -0.001f;
    [SerializeField] private float seeReward = 0.2f;
    [SerializeField] private float catchReward = 2f;
    [SerializeField] private float entropyRewardScale = 0.2f; // reward for reducing entropy (optional)

    [SerializeField] private Transform wallsParent;
    [SerializeField] private int wallCount = 6;

    [SerializeField] private Material loseMat, winMat;
 
    private List<GameObject> walls = new();

    public void ResetWalls()
    {
        // Удаляем старые стены
        for (int i = walls.Count - 1; i >= 0; i--)
            Destroy(walls[i]);
        walls.Clear();

        float floorRadius = (floorMeshRender.transform.localScale.x - 0.5f) / 2f;
        int maxAttempts = 50;

        for (int i = 0; i < wallCount; i++)
        {
            bool placed = false;
            int attempts = 0;

            while (!placed && attempts < maxAttempts)
            {
                attempts++;

                float radius = floorRadius * 0.9f;
                Vector2 rnd = Random.insideUnitCircle * radius;

                Vector3 pos = new Vector3(rnd.x, 0.5f, rnd.y);

                Vector3 scale = new Vector3(
                    Random.Range(2f, 6f),  // длина
                    2f,                    // высота
                    1f                     // толщина
                );

                Quaternion rot = Quaternion.Euler(0, Random.Range(0f, 360f), 0);

                if (IsPositionValidSimple(pos, scale, rot))
                {
                    GameObject wall = Instantiate(wallPrefab, wallsParent);
                    wall.transform.localPosition = pos;
                    wall.transform.localScale = scale;
                    wall.transform.rotation = rot;

                    wall.tag = "LocalWall";
                    walls.Add(wall);

                    placed = true;
                }
            }
        }
    }

    private bool IsPositionValidSimple(Vector3 pos, Vector3 scale, Quaternion rot)
    {
        float minFromAgent = 0.5f;
        float minFromTarget = 0.5f;

        // Агент
        if (Vector3.Distance(pos, transform.localPosition) <
            minFromAgent + (scale.magnitude / 2))
            return false;

        // Цель (Hider)
        if (Vector3.Distance(pos, targetTransform.localPosition) <
            minFromTarget + (scale.magnitude / 2))
            return false;

        // Стены друг к другу
        foreach (GameObject w in walls)
        {
            float minDist = 1f;
            float comb = (scale.magnitude + w.transform.localScale.magnitude) / 2f;

            if (Vector3.Distance(pos, w.transform.localPosition) < minDist + comb)
                return false;
        }

        return true;
    }


    // internal
    private Vector2Int? lastSeenHiderCell = null;
    private bool hiderCurrentlyVisible = false;
    private float previousEntropy = 0f;

    // Observations size constant helper
    public static int ComputeObservationSize(int gridSizeLocal)
    {
        return gridSizeLocal * gridSizeLocal /*probabilities*/ + 2 /*agent cell*/ + 2 /*agent dir*/ + 2 /*lastSeen offset*/ + 1 /*see flag*/;
    }

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        if (floorMeshRender == null)
        {
            Debug.LogError("Assign floorMeshRender in inspector!");
        }
    }

    private void Start()
    {
        // compute grid origin and scale from floor mesh renderer bounds
        Vector3 floorPos = floorMeshRender.transform.position;
        Vector3 floorScale = floorMeshRender.transform.localScale;
        float worldSide = floorScale.x; // assumes square floor and scale represents side length
        gridWorldSize = worldSide;
        gridOrigin = floorPos - new Vector3(worldSide, 0f, worldSide) * 0.5f;

        InitializeGrid();
        previousEntropy = CalculateEntropy();
    }

    private void InitializeGrid()
    {
        gridMemory.Clear();
        int size = gridSize;
        float uniform = 1f / (size * size);
        for (int x = 0; x < size; x++)
        {
            for (int z = 0; z < size; z++)
            {
                var pos = new Vector2Int(x, z);
                gridMemory[pos] = new CellMemory()
                {
                    GridPos = pos,
                    Probability = uniform,
                    IsWall = false,
                    HasBeenSeen = false
                };
            }
        }
        lastSeenHiderCell = null;
        hiderCurrentlyVisible = false;
    }

    public override void OnEpisodeBegin()
    {
        // reset agent and target positions (you can change ranges to your arena)
        transform.localPosition = new Vector3(0f, transform.localPosition.y, 0f);
        transform.localRotation = Quaternion.identity;

        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        // place target somewhere random within floor
        float radius = (gridWorldSize - 0.5f) * 0.5f;
        Vector2 rnd = Random.insideUnitCircle * radius;
        targetTransform.localPosition = new Vector3(rnd.x, targetTransform.localPosition.y, rnd.y);

        ResetWalls();

        InitializeGrid();
        previousEntropy = CalculateEntropy();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 1) grid probabilities (row-major x,z)
        for (int z = 0; z < gridSize; z++)
        {
            for (int x = 0; x < gridSize; x++)
            {
                var key = new Vector2Int(x, z);
                float p = 0f;
                if (gridMemory.TryGetValue(key, out var cell))
                    p = Mathf.Clamp01(cell.Probability);
                sensor.AddObservation(p);
            }
        }

        // 2) agent cell (normalized)
        var agentCell = WorldToGridPos(transform.position);
        sensor.AddObservation(agentCell.x / (float)gridSize);
        sensor.AddObservation(agentCell.y / (float)gridSize);

        // 3) agent orientation (cos, sin)
        float ang = transform.eulerAngles.y * Mathf.Deg2Rad;
        sensor.AddObservation(Mathf.Cos(ang));
        sensor.AddObservation(Mathf.Sin(ang));

        // 4) lastSeen offset normalized (dx, dz) (0,0 if none)
        if (lastSeenHiderCell.HasValue)
        {
            Vector2Int last = lastSeenHiderCell.Value;
            // center of that cell world pos
            Vector3 world = GridToWorldCenter(last);
            Vector3 offset = world - transform.position;
            // normalize by gridWorldSize
            sensor.AddObservation(offset.x / gridWorldSize);
            sensor.AddObservation(offset.z / gridWorldSize);
        }
        else
        {
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
        }

        // 5) do we see Hider right now? (1/0)
        sensor.AddObservation(hiderCurrentlyVisible ? 1f : 0f);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // actions: [0]=moveX (-1..1), [1]=moveZ (-1..1), [2]=rotation (-1..1)
        float moveX = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float moveZ = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float rot = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);

        Vector3 moveWorld = (transform.right * moveX + transform.forward * moveZ).normalized;
        Vector3 newPos = rb.position + moveWorld * moveSpeed * Time.deltaTime;
        rb.MovePosition(newPos);
        rb.MoveRotation(rb.rotation * Quaternion.Euler(0f, rot * rotationSpeed * Time.deltaTime, 0f));

        // small time penalty to encourage speed
        AddReward(timePenalty);

        // update perception and heatmap
        bool saw = UpdateSeeAndMark(); // runs raycasts, updates gridMemory.HasBeenSeen, returns true if target seen
        bool caught = CheckGetZone();

        // update heatmap diffusion if we lost sight
        UpdateHeatmapDynamics();

        // rewards: seeing target
        if (saw && !hiderCurrentlyVisible)
        {
            AddReward(seeReward);
        }
        hiderCurrentlyVisible = saw;

        // reward for entropy reduction (optional, small)
        float entropy = CalculateEntropy();
        if (entropy < previousEntropy)
        {
            AddReward((previousEntropy - entropy) * entropyRewardScale);
        }
        previousEntropy = entropy;

        if (caught)
        {
            AddReward(catchReward);
            // Optionally end episodes for both agents in the scene � caller's responsibility to sync other agents!
            EndEpisode();
            floorMeshRender.material = winMat;
            // If you want to also end hider, find and end it here (example):
            var hider = Object.FindObjectOfType<AgentHider>();
            if (hider != null) hider.WasCaught();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var cont = actionsOut.ContinuousActions;
        float moveX = 0f;
        float moveZ = 0f;
        float rot = 0f;
        if (Input.GetKey(KeyCode.W)) moveZ = 1f;
        if (Input.GetKey(KeyCode.S)) moveZ = -1f;
        if (Input.GetKey(KeyCode.D)) moveX = 1f;
        if (Input.GetKey(KeyCode.A)) moveX = -1f;
        if (Input.GetKey(KeyCode.Q)) rot = -1f;
        if (Input.GetKey(KeyCode.E)) rot = 1f;
        cont[0] = moveX;
        cont[1] = moveZ;
        cont[2] = rot;

        if (Input.GetKey(KeyCode.R)) EndEpisode();
    }

    // -----------------------------
    // Vision & Heatmap logic
    // -----------------------------

    /// <summary>
    /// Performs the See rays, marks cells seen, updates lastSeenHiderCell if target is spotted.
    /// Returns true if target is directly visible now.
    /// </summary>
    private bool UpdateSeeAndMark()
    {
        bool sawTarget = false;
        Vector2Int? targetCell = null;

        float angleStep = seeAngle / Mathf.Max(1, raysForSee - 1);
        float startAngle = -seeAngle / 2f;

        float cellSize = gridWorldSize / gridSize;

        for (int i = 0; i < raysForSee; i++)
        {
            float currentAngle = startAngle + angleStep * i;
            Vector3 dir = Quaternion.Euler(0f, currentAngle, 0f) * transform.forward;
            Vector3 origin = transform.position + Vector3.up * 0.3f;

            if (Physics.Raycast(origin, dir, out RaycastHit hit, seeRadius, obstacleMask))
            {
                // draw debug ray
                Debug.DrawRay(origin, dir * hit.distance, Color.red);

                // mark cells along the ray up to hit.point as seen
                MarkCellsOnRay(origin, hit.point);
                // if hit is target
                if (hit.collider != null && hit.collider.transform == targetTransform)
                {
                    sawTarget = true;
                    targetCell = WorldToGridPos(hit.point);
                }
                else
                {
                    // if hit a LocalWall, mark that cell as wall
                    if (hit.collider != null && hit.collider.CompareTag("LocalWall"))
                    {
                        var c = WorldToGridPos(hit.point);
                        if (gridMemory.ContainsKey(c)) gridMemory[c].IsWall = true;
                    }
                }
            }
            else
            {
                Debug.DrawRay(origin, dir * seeRadius, Color.green);
                Vector3 end = origin + dir * seeRadius;
                MarkCellsOnRay(origin, end);
            }
        }

        if (sawTarget && targetCell.HasValue)
        {
            // set that cell probability to 1 and zero others (or optionally set high weight)
            SetHeatmapToCell(targetCell.Value);
            lastSeenHiderCell = targetCell.Value;
            hiderCurrentlyVisible = true;
        }
        else
        {
            // if not seen this step, and previously seen, mark hiderCurrentlyVisible false (handled in OnActionReceived)
            // we do not clear lastSeenHiderCell here
        }

        return sawTarget;
    }

    private void MarkCellsOnRay(Vector3 start, Vector3 end)
    {
        float cellSize = gridWorldSize / gridSize;
        Vector3 delta = end - start;
        int steps = Mathf.CeilToInt(delta.magnitude / (cellSize * 0.5f));
        for (int i = 0; i <= steps; i++)
        {
            Vector3 pt = Vector3.Lerp(start, end, i / (float)steps);
            Vector2Int cell = WorldToGridPos(pt);
            if (gridMemory.ContainsKey(cell))
            {
                gridMemory[cell].HasBeenSeen = true;
                // since we can see that cell and no hider was there (unless we just saw target), set prob to 0
                gridMemory[cell].Probability = 0f;
            }
        }
    }

    private void SetHeatmapToCell(Vector2Int cell)
    {
        // zero all probabilities then set single cell to 1
        foreach (var kv in gridMemory)
            kv.Value.Probability = 0f;
        if (gridMemory.ContainsKey(cell))
            gridMemory[cell].Probability = 1f;
    }

    private void UpdateHeatmapDynamics()
    {
        // if currently visible, keep heatmap pinned
        if (hiderCurrentlyVisible) return;

        // if we have last seen location, diffuse probability outward
        if (!lastSeenHiderCell.HasValue) return;

        // diffusion step
        Dictionary<Vector2Int, float> newProbs = new Dictionary<Vector2Int, float>();
        foreach (var k in gridMemory.Keys)
            newProbs[k] = 0f;

        foreach (var kv in gridMemory)
        {
            Vector2Int cell = kv.Key;
            float p = kv.Value.Probability;
            if (p <= 0f) continue;

            // keep some in place
            float keep = p * (1f - diffusionRate);
            newProbs[cell] += keep;

            // distribute rest to neighbors (4-directional to simplify)
            var neigh = GetCardinalNeighbors(cell);
            if (neigh.Count == 0) continue;
            float distribute = p * diffusionRate / neigh.Count;
            foreach (var n in neigh)
            {
                if (gridMemory[n].IsWall) continue; // do not diffuse into walls
                newProbs[n] += distribute;
            }
        }

        // write back and normalize
        float total = 0f;
        foreach (var kv in newProbs)
        {
            gridMemory[kv.Key].Probability = kv.Value;
            total += kv.Value;
        }
        if (total > 1e-6f)
        {
            foreach (var kv in gridMemory)
                kv.Value.Probability /= total;
        }
        else
        {
            // if lost all probability, reinitialize uniform but keep walls excluded
            float uniform = 1f / (gridSize * gridSize);
            foreach (var kv in gridMemory)
                kv.Value.Probability = uniform;
        }
    }

    private List<Vector2Int> GetCardinalNeighbors(Vector2Int cell)
    {
        List<Vector2Int> res = new List<Vector2Int>();
        Vector2Int[] dirs = new Vector2Int[] { new Vector2Int(1, 0), new Vector2Int(-1, 0), new Vector2Int(0, 1), new Vector2Int(0, -1) };
        foreach (var d in dirs)
        {
            var n = cell + d;
            if (gridMemory.ContainsKey(n)) res.Add(n);
        }
        return res;
    }

    private bool CheckGetZone()
    {
        // Check Hider within Get cone (narrower and smaller radius)
        Vector3 toTarget = targetTransform.position - transform.position;
        float dist = toTarget.magnitude;
        if (dist > getRadius) return false;
        float angle = Vector3.Angle(transform.forward, toTarget.normalized);
        if (angle > getAngle / 2f) return false;
        // optionally check line of sight
        if (Physics.Raycast(transform.position + Vector3.up * 0.3f, toTarget.normalized, out RaycastHit hit, getRadius, obstacleMask))
        {
            if (hit.collider != null && hit.collider.transform == targetTransform)
                return true;
            else
                return false;
        }
        return false;
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Wall"))
        {
            AddReward(-1000f);

            var hider = FindObjectOfType<AgentHider>();
            //if (hider != null)
            //    hider.AddReward(+0.5f);

            EndEpisode();
            floorMeshRender.material = loseMat;
            if (hider != null)
                hider.EndEpisode();

            return;
        }

        if (other.tag == "Goal")
        {
            AddReward(catchReward);  
            EndEpisode();
            floorMeshRender.material = winMat;

            var hider = FindObjectOfType<AgentHider>();
            if (hider != null)
                hider.WasCaught();

        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.tag == "Goal")
        {
            AddReward(catchReward);
            EndEpisode();
            floorMeshRender.material = winMat;

            var hider = FindObjectOfType<AgentHider>();
            if (hider != null)
                hider.WasCaught();

        }
    }

    // -----------------------------
    // Utilities: grid <-> world
    // -----------------------------
    private Vector2Int WorldToGridPos(Vector3 worldPos)
    {
        float cellSize = gridWorldSize / gridSize;
        int x = Mathf.FloorToInt((worldPos.x - gridOrigin.x) / cellSize);
        int z = Mathf.FloorToInt((worldPos.z - gridOrigin.z) / cellSize);
        x = Mathf.Clamp(x, 0, gridSize - 1);
        z = Mathf.Clamp(z, 0, gridSize - 1);
        return new Vector2Int(x, z);
    }

    private Vector3 GridToWorldCenter(Vector2Int gridPos)
    {
        float cellSize = gridWorldSize / gridSize;
        Vector3 center = gridOrigin + new Vector3((gridPos.x + 0.5f) * cellSize, 0f, (gridPos.y + 0.5f) * cellSize);
        return center;
    }

    private float CalculateEntropy()
    {
        float entropy = 0f;
        foreach (var kv in gridMemory)
        {
            float p = kv.Value.Probability;
            if (p > 1e-6f)
                entropy -= p * Mathf.Log(p);
        }
        return entropy;
    }

    private void OnDrawGizmosSelected()
    {
        // Draw see and get cones
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position, getRadius);
        Vector3 a = DirFromAngle(-getAngle / 2);
        Vector3 b = DirFromAngle(getAngle / 2);
        Gizmos.DrawLine(transform.position, transform.position + a * getRadius);
        Gizmos.DrawLine(transform.position, transform.position + b * getRadius);

        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position, seeRadius);
        Vector3 sa = DirFromAngle(-seeAngle / 2);
        Vector3 sb = DirFromAngle(seeAngle / 2);
        Gizmos.DrawLine(transform.position, transform.position + sa * seeRadius);
        Gizmos.DrawLine(transform.position, transform.position + sb * seeRadius);
    }

    private Vector3 DirFromAngle(float angle)
    {
        float a = angle + transform.eulerAngles.y;
        return new Vector3(Mathf.Sin(a * Mathf.Deg2Rad), 0f, Mathf.Cos(a * Mathf.Deg2Rad));
    }

    // -----------------------------
    // Helper classes
    // -----------------------------
    public class CellMemory
    {
        public Vector2Int GridPos;
        public bool IsWall = false;
        public bool HasBeenSeen = false;
        public float Probability = 0f;
    }
}
