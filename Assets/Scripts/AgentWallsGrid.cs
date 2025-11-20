using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class AgentWallsGrid : Agent
{
    private float previousDistance;

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

    private Dictionary<Vector2Int, CellMemory> gridMemory = new();
    private int gridSize = 15;      // initialized here for simplicity

    // Heatmap variables
    private Vector2Int? lastKnownHiderCell = null;
    private float heatmapDecayRate = 0.15f; // How much probability spreads per step
    private bool hiderCurrentlyVisible = false;

    private void Start()
    {
        meshAreaGet = new Mesh();
        meshAreaGet.name = "View Mesh Get";
        meshFilterAreaGet.mesh = meshAreaGet;

        meshAreaSee = new Mesh();
        meshAreaSee.name = "View Mesh See";
        meshFilterAreaSee.mesh = meshAreaSee;
    }

    private void InitializeMemoryGrid()
    {
        gridMemory.Clear();
        
        // Initialize uniform probability distribution
        float uniformProbability = 1.0f / (gridSize * gridSize);
        
        for (int x = 0; x < 15; x++)
        {
            for (int y = 0; y < 15; y++)
            {
                Vector2Int cellPos = new Vector2Int(x, y);
                if (!gridMemory.ContainsKey(cellPos))
                {
                    gridMemory[cellPos] = new CellMemory() 
                    { 
                        GridPos = cellPos,
                        Probability = uniformProbability
                    };
                }
            }
        }
        
        lastKnownHiderCell = null;
        hiderCurrentlyVisible = false;
    }

    public override void OnEpisodeBegin()
    {
        InitializeMemoryGrid();
        transform.localPosition = new Vector3(0, 0.26f * 3, 0);
        lastSeenTargetPosition = null;
        transform.localRotation = Quaternion.identity;
        targetPosition.localPosition = GetRandomPositionInCircle((floorMeshRender.gameObject.transform.localScale.x - 0.5f) / 2);

        int offset = gridSize / 2;

        Vector2Int gridPos = new Vector2Int(Mathf.FloorToInt(targetPosition.localPosition.x + offset + 0.5f), Mathf.FloorToInt(targetPosition.localPosition.z + offset + 0.5f));

        // if (gridMemory.ContainsKey(gridPos))
        //   gridMemory[gridPos].CellType = CellState.Target;


        previousDistance = Vector3.Distance(transform.localPosition, targetPosition.localPosition);

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
                    2.0f,
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
                    cube.tag = "LocalWall";
                }
            }
        }
    }

    private bool IsPositionValidSimple(Vector3 position, Vector3 scale, Quaternion rotation)
    {
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
        for (int z = 0; z < gridSize; z++)
        {
            for (int x = 0; x < gridSize; x++)
            {
                Vector2Int pos = new Vector2Int(x, z);
                if (gridMemory.TryGetValue(pos, out CellMemory cell))
                {
                    float typeValue = 0f;
                    if (cell.CellType == CellState.Wall) typeValue = 1f;
                    else if (cell.CellType == CellState.Target) typeValue = 2f;

                    float seenValue = cell.HasBeenSeen ? 1f : 0f;
                    
                    // Add heatmap probability as observation
                    float probabilityValue = cell.Probability;

                    sensor.AddObservation(typeValue);
                    sensor.AddObservation(seenValue);
                    sensor.AddObservation(probabilityValue);
                }
                else
                {
                    sensor.AddObservation(0f);
                    sensor.AddObservation(0f);
                    sensor.AddObservation(0f);
                }
            }
        }

        Vector3 floorScale = floorMeshRender.transform.localScale;
        Vector3 floorPos = floorMeshRender.transform.position;
        Vector3 origin = floorPos - new Vector3(floorScale.x, 0, floorScale.z) * 0.5f;
        float cellSize = floorScale.x / gridSize;

        Vector2Int agentCell = WorldToGridPos(transform.position, origin, cellSize);
        sensor.AddObservation(agentCell.x / (float)gridSize);
        sensor.AddObservation(agentCell.y / (float)gridSize);

        float angleRad = transform.eulerAngles.y * Mathf.Deg2Rad;
        sensor.AddObservation(Mathf.Cos(angleRad));
        sensor.AddObservation(Mathf.Sin(angleRad));
    }



    public override void OnActionReceived(ActionBuffers actions)
    {
        // With SetReward you set the reward of a specific step during learning. With AddReward you add a value to the current reward value of that step.

        Vector3 move = new Vector3(actions.ContinuousActions[0], 0, actions.ContinuousActions[1]);
        transform.localPosition += move * Time.deltaTime * moveSpeed;

        float rotationY = actions.ContinuousActions[2];
        transform.Rotate(Vector3.up, rotationY * rotationSpeed * Time.deltaTime);
        
        // Update heatmap every step
        UpdateHeatmap();

        // ===== REWARD STRUCTURE =====

        // 1. CONSTANT TIME PENALTY (small, every step)
        //    Encourages agent to solve episode quickly
        AddReward(-0.001f);

        /*if (!CanSeeTarget() && lastSeenTargetPosition.HasValue)
        {
            float distance = Vector3.Distance(transform.localPosition, targetPosition.localPosition);
            float distDelta = previousDistance - distance;

            // Only reward if agent is moving CLOSER (distDelta > 0)
            if (distDelta > 0)
            {
                AddReward(distDelta * 0.05f);  // Small bonus for progress
            }
            else if (distDelta < 0)
            {
                AddReward(distDelta * 0.02f);  // Small penalty for moving away
            }

            previousDistance = distance;
        }*/

        // 2. EXPLORATION REWARD (when target is NOT in sight)
        //    Small reward for moving into cells with higher probability of target presence
        if (!CanSeeTarget() && lastKnownHiderCell.HasValue)
        {
            Vector3 floorScale = floorMeshRender.transform.localScale;
            Vector3 floorPos = floorMeshRender.transform.position;
            Vector3 origin = floorPos - new Vector3(floorScale.x, 0, floorScale.z) * 0.5f;
            float cellSize = floorScale.x / gridSize;
            Vector2Int agentCell = WorldToGridPos(transform.position, origin, cellSize);
            if (gridMemory.ContainsKey(agentCell))
            {
                float cellProbability = gridMemory[agentCell].Probability;
                AddReward(cellProbability * 0.1f); // Reward proportional to probability
            }
        }

        // 3. VISION REWARD (when target comes into sight)
        //    Significant but not game-ending reward for spotting target
        if (CanSeeTarget())
        {
            lastSeenTargetPosition = targetPosition.localPosition;
            lastSeenTime = Time.time;

            if (!hasGivenSightReward)
            {
                AddReward(50f);  // ? Use AddReward, not SetReward
                hasGivenSightReward = true;
            }
            else
            {
                // Continue rewarding for MAINTAINING sight (small bonus each step)
                AddReward(0.1f);
            }
        }

        // 4. CONTACT/CAPTURE REWARD (terminal state)
        //    Largest reward for actually reaching/capturing target
        if (lastSeenTargetPosition.HasValue && CanGetTarget())
        {
            AddReward(1000f);  // ? Can use either AddReward or SetReward here
                               //   (episode ends immediately after)
            EndEpisode();
            //transform.parent.GetComponentInChildren<AgentHider>().EndEpisode(); uncomment after seeker training
            floorMeshRender.material = winMat;
        }

        /*if (CanSeeTarget())
        {
            lastSeenTargetPosition = targetPosition.localPosition;
            lastSeenTime = Time.time;

            float distance = Vector3.Distance(transform.localPosition, targetPosition.localPosition);
            float distDelta = previousDistance - distance;
            AddReward(distDelta * 0.1f);
            previousDistance = distance;

            // Instead of this add reward as Agent keeps target in sight?
            // AddReward(0.01f);
            if (!hasGivenSightReward)
            {
                SetReward(100f);
                hasGivenSightReward = true;
            }
        }
        else
            AddReward(-0.001f);     // time penalty     

        if (lastSeenTargetPosition.HasValue && CanGetTarget())
        {
            SetReward(1000f);
            EndEpisode();
            floorMeshRender.material = winMat;
        }*/
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
            AddReward(200f);  // Smaller reward than vision-based (100, 50, or 300 � adjust based on testing)
            EndEpisode();
            floorMeshRender.material = winMat;
        }

        if (other.tag == "Wall")    // Boundary wall
        {
            AddReward(-1000f);  // Large penalty for collision failure
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


    void OnDrawGizmos()
    {
        if (floorMeshRender == null) return;

        Vector3 floorPos = floorMeshRender.transform.position;
        Vector3 floorScale = floorMeshRender.transform.localScale;

        float cellSize = floorScale.x / gridSize;
        Vector3 origin = floorPos - new Vector3(floorScale.x, -1, floorScale.z) * 0.5f;

        foreach (var pair in gridMemory)
        {
            int x = pair.Key.x;
            int z = pair.Key.y;
            Vector3 center = origin + new Vector3(x * cellSize + cellSize / 2f, 0.01f, z * cellSize + cellSize / 2f);

            Color color = Color.gray;
            
            // Visualize heatmap probabilities
            float probability = pair.Value.Probability;
            // Use red to visualize probability (darker red = higher probability)
            color = new Color(probability * 5f, 0f, 0f); // Multiply by 5 to make it more visible
            
            // Override with special states
            if (pair.Value.CellType == CellState.Wall) color = Color.black;
            else if (pair.Value.CellType == CellState.Target) color = Color.yellow;
            else if (pair.Value.HasBeenSeen) 
            {
                // Mix green with heatmap red to show explored areas
                color = new Color(probability * 5f, 0.3f, 0f);
            }

            Gizmos.color = new Color(color.r, color.g, color.b, 0.3f);
            Gizmos.DrawCube(center, new Vector3(cellSize * 0.95f, 0.01f, cellSize * 0.95f));
        }

        Gizmos.color = Color.white;
        for (int x = 0; x <= gridSize; x++)
        {
            Vector3 start = origin + new Vector3(x * cellSize, 0.02f, 0);
            Vector3 end = start + new Vector3(0, 0, gridSize * cellSize);
            Gizmos.DrawLine(start, end);
        }

        for (int z = 0; z <= gridSize; z++)
        {
            Vector3 start = origin + new Vector3(0, 0.02f, z * cellSize);
            Vector3 end = start + new Vector3(gridSize * cellSize, 0, 0);
            Gizmos.DrawLine(start, end);
        }
    }

    private bool CanGetTarget()
    {
        Vector3 dirToTarget = (targetPosition.position - transform.position).normalized;
        float dstToTarget = Vector3.Distance(transform.position, targetPosition.position);

        // Debug.Log((Vector3.Angle(transform.forward, dirToTarget) < viewGetAngle / 2) + " " + (dstToTarget <= viewGetRadius));
        if (Vector3.Angle(transform.forward, dirToTarget) < viewGetAngle / 2 && dstToTarget <= viewGetRadius)
            //if (!Physics.Raycast(transform.position, dirToTarget, dstToTarget, obstacleMask))
            return true;

        return false;
    }

    private bool CanSeeTarget()
    {
        int rayCount = 20;
        float angleStep = viewSeeAngle / (rayCount - 1);
        float startAngle = -viewSeeAngle / 2;
        bool sawGoal = false;
        Vector2Int? currentHiderCell = null;

        float floorScale = floorMeshRender.transform.localScale.x;
        Vector3 origin = floorMeshRender.transform.position - new Vector3(floorScale, 0, floorScale) * 0.5f;
        float cellSize = floorScale / gridSize;

        for (int i = 0; i < rayCount; i++)
        {
            float currentAngle = startAngle + angleStep * i;
            Vector3 dir = Quaternion.Euler(0, currentAngle, 0) * transform.forward;

            Vector3 start = transform.position;
            Vector3 end;

            if (Physics.Raycast(start, dir, out RaycastHit hit, viewSeeRadius, obstacleMask))
            {
                Debug.DrawRay(start, dir * hit.distance, Color.red);
                end = hit.point;

                Vector2Int cell = WorldToGridPos(hit.point, origin, cellSize);
                if (gridMemory.ContainsKey(cell))
                {
                    if (hit.collider.CompareTag("LocalWall"))
                        gridMemory[cell].CellType = CellState.Wall;
                    else if (hit.collider.CompareTag("Goal"))
                    {
                        gridMemory[cell].CellType = CellState.Target;
                        sawGoal = true;
                        currentHiderCell = cell;
                    }
                }
            }
            else
            {
                Debug.DrawRay(start, dir * viewSeeRadius, Color.green);
                end = start + dir * viewSeeRadius;
            }

            MarkCellsOnRayPath(start, end, origin, cellSize);
        }
        
        // Update heatmap based on visibility
        if (sawGoal && currentHiderCell.HasValue)
        {
            // Hider is visible - set that cell to probability 1.0
            SetHeatmapToCell(currentHiderCell.Value);
            lastKnownHiderCell = currentHiderCell.Value;
            hiderCurrentlyVisible = true;
        }
        else if (hiderCurrentlyVisible)
        {
            // Just lost sight of hider - start diffusion from last known position
            hiderCurrentlyVisible = false;
        }

        return sawGoal;
    }


    //Digital Differential Analyzer (DDA)
    private void MarkCellsOnRayPath(Vector3 start, Vector3 end, Vector3 origin, float cellSize)
    {
        Vector3 delta = end - start;
        int steps = Mathf.CeilToInt(delta.magnitude / (cellSize * 0.5f));
        for (int i = 0; i <= steps; i++)
        {
            Vector3 point = Vector3.Lerp(start, end, i / (float)steps);
            Vector2Int cell = WorldToGridPos(point, origin, cellSize);
            if (gridMemory.ContainsKey(cell))
            {
                gridMemory[cell].HasBeenSeen = true;
            }
        }
    }

    private Vector2Int WorldToGridPos(Vector3 worldPos, Vector3 origin, float cellSize)
    {
        int x = Mathf.FloorToInt((worldPos.x - origin.x) / cellSize);
        int z = Mathf.FloorToInt((worldPos.z - origin.z) / cellSize);
        return new Vector2Int(x, z);
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
    
    // Heatmap Methods
    
    private void SetHeatmapToCell(Vector2Int cell)
    {
        // Set all probabilities to 0 except the cell where hider is seen
        foreach (var kvp in gridMemory)
        {
            kvp.Value.Probability = 0f;
        }
        
        if (gridMemory.ContainsKey(cell))
        {
            gridMemory[cell].Probability = 1.0f;
        }
    }
    
    private void UpdateHeatmap()
    {
        if (hiderCurrentlyVisible) return;
        if (!lastKnownHiderCell.HasValue) return;
        
        // Calculate entropy (uncertainty) before update
        float entropyBefore = CalculateEntropy();
        
        ClearVisibleCellsFromHeatmap();
        DiffuseHeatmap();
        NormalizeHeatmap();
        
        // Calculate entropy after update
        float entropyAfter = CalculateEntropy();
        
        // Reward for reducing uncertainty (exploring effectively)
        if (entropyAfter < entropyBefore)
        {
            AddReward((entropyBefore - entropyAfter) * 0.5f);
        }
    }
    
    private float CalculateEntropy()
    {
        float entropy = 0f;
        foreach (var cell in gridMemory.Values)
        {
            if (cell.Probability > 0.0001f)
            {
                entropy -= cell.Probability * Mathf.Log(cell.Probability);
            }
        }
        return entropy;
    }

    private void ClearVisibleCellsFromHeatmap()
    {
        // For each cell marked as HasBeenSeen in the current field of view,
        // if we don't see the hider there, set its probability to 0
        // This simulates: "I can see this area and the hider is NOT here"
        
        int rayCount = 20;
        float angleStep = viewSeeAngle / (rayCount - 1);
        float startAngle = -viewSeeAngle / 2;
        
        float floorScale = floorMeshRender.transform.localScale.x;
        Vector3 origin = floorMeshRender.transform.position - new Vector3(floorScale, 0, floorScale) * 0.5f;
        float cellSize = floorScale / gridSize;
        
        HashSet<Vector2Int> visibleCells = new HashSet<Vector2Int>();
        
        for (int i = 0; i < rayCount; i++)
        {
            float currentAngle = startAngle + angleStep * i;
            Vector3 dir = Quaternion.Euler(0, currentAngle, 0) * transform.forward;
            Vector3 start = transform.position;
            Vector3 end;
            
            if (Physics.Raycast(start, dir, out RaycastHit hit, viewSeeRadius, obstacleMask))
            {
                end = hit.point;
            }
            else
            {
                end = start + dir * viewSeeRadius;
            }
            
            // Mark all cells along this ray as visible
            Vector3 delta = end - start;
            int steps = Mathf.CeilToInt(delta.magnitude / (cellSize * 0.5f));
            for (int j = 0; j <= steps; j++)
            {
                Vector3 point = Vector3.Lerp(start, end, j / (float)steps);
                Vector2Int cell = WorldToGridPos(point, origin, cellSize);
                if (gridMemory.ContainsKey(cell))
                {
                    visibleCells.Add(cell);
                }
            }
        }
        
        // Clear probability from visible cells (since we don't see hider there)
        foreach (var cell in visibleCells)
        {
            if (gridMemory.ContainsKey(cell))
            {
                gridMemory[cell].Probability = 0f;
            }
        }
    }
    
    private void DiffuseHeatmap()
    {
        Dictionary<Vector2Int, float> newProbabilities = new Dictionary<Vector2Int, float>();
        
        // Initialize all cells to 0
        foreach (var key in gridMemory.Keys)
        {
            newProbabilities[key] = 0f;
        }
        
        // Diffuse probability from each cell to its neighbors
        foreach (var kvp in gridMemory)
        {
            Vector2Int cell = kvp.Key;
            float probability = kvp.Value.Probability;
            
            if (probability <= 0.0001f) continue; // Skip cells with negligible probability
            
            // Get valid neighbors (8-directional movement)
            List<Vector2Int> neighbors = GetValidNeighbors(cell);
            
            if (neighbors.Count == 0) continue;
            
            // Keep some probability in current cell, distribute rest to neighbors
            float retainRate = 1.0f - heatmapDecayRate;
            float distributeAmount = probability * heatmapDecayRate;
            float perNeighbor = distributeAmount / neighbors.Count;
            
            // Retain probability in current cell
            newProbabilities[cell] += probability * retainRate;
            
            // Distribute to neighbors
            foreach (var neighbor in neighbors)
            {
                if (newProbabilities.ContainsKey(neighbor))
                {
                    newProbabilities[neighbor] += perNeighbor;
                }
            }
        }
        
        // Update gridMemory with new probabilities
        foreach (var kvp in newProbabilities)
        {
            if (gridMemory.ContainsKey(kvp.Key))
            {
                gridMemory[kvp.Key].Probability = kvp.Value;
            }
        }
    }
    
    private List<Vector2Int> GetValidNeighbors(Vector2Int cell)
    {
        List<Vector2Int> neighbors = new List<Vector2Int>();
        
        // 8 directions (including diagonals)
        Vector2Int[] directions = new Vector2Int[]
        {
            new Vector2Int(-1, -1), new Vector2Int(0, -1), new Vector2Int(1, -1),
            new Vector2Int(-1, 0),                          new Vector2Int(1, 0),
            new Vector2Int(-1, 1),  new Vector2Int(0, 1),  new Vector2Int(1, 1)
        };
        
        foreach (var dir in directions)
        {
            Vector2Int neighbor = cell + dir;
            
            // Check if neighbor is within grid bounds and not a wall
            if (gridMemory.ContainsKey(neighbor))
            {
                // Don't diffuse into walls
                if (gridMemory[neighbor].CellType == CellState.Wall)
                    continue;
                    
                neighbors.Add(neighbor);
            }
        }
        
        return neighbors;
    }
    
    private void NormalizeHeatmap()
    {
        float total = 0f;
        foreach (var cell in gridMemory.Values)
        {
            total += cell.Probability;
        }
        
        if (total > 0.0001f)
        {
            // Normalize so total probability = 1.0
            foreach (var cell in gridMemory.Values)
            {
                cell.Probability /= total;
            }
        }
        else
        {
            // If all probabilities are 0 (shouldn't happen), reset to uniform
            float uniformProb = 1.0f / gridMemory.Count;
            foreach (var cell in gridMemory.Values)
            {
                cell.Probability = uniformProb;
            }
        }
    }
}

public class CellMemory
{
    public Vector2Int GridPos { get; set; }
    public CellState CellType { get; set; }
    public bool HasBeenSeen { get; set; }
    public float Probability { get; set; }
}

public enum CellState
{
    Empty,
    Wall,
    Target
}
