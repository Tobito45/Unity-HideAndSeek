//csharp Assets\Scripts\MapGenerator.cs;
using UnityEngine;
using System.Collections.Generic;

public class MapGenerator : MonoBehaviour
{
    [Header("Map")]
    [Tooltip("Side length used in Inspector (agent script expects transform.localScale.x == mapSize).")]
    public float mapSize = 10f;                 // inspector-exposed side length

    [Header("Floor")]
    public Material floorMaterial;
    public float floorYOffset = 0f;             // Y position of the floor

    [Header("Boundary")]
    public GameObject boundaryWallPrefab;       // prefab for boundary (optional)
    public float boundaryHeight = 2.0f;
    public float boundaryThickness = 0.5f;

    [Header("Interior walls")]
    public GameObject innerWallPrefab;          // prefab used to spawn interior walls (optional)
    public int innerWallCount = 3;              // configurable number of interior walls
    public float innerWallMinLength = 1.0f;
    public float innerWallMaxLength = 4.0f;
    public float innerWallHeight = 2.0f;
    public float innerWallThickness = 0.5f;
    [Tooltip("Minimum margin from map edges where interior walls can be placed.")]
    public float placementMargin = 1.0f;

    private GameObject _floorInstance;
    private GameObject _boundaryParent;
    private GameObject _interiorParent;
    private readonly List<GameObject> _spawnedInterior = new List<GameObject>();

    public MeshRenderer FloorMeshRenderer => _floorInstance ? _floorInstance.GetComponent<MeshRenderer>() : null;
    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.T))
        {
            GenerateMap();
        }
    }

    public void GenerateMap()
    {
        ClearMap();
        CreateFloor();
        CreateBoundary();
        CreateInteriorWalls();
    }

    public void ClearMap()
    {
        if (_floorInstance) DestroyImmediate(_floorInstance);
        if (_boundaryParent) DestroyImmediate(_boundaryParent);
        if (_interiorParent) DestroyImmediate(_interiorParent);
        _spawnedInterior.Clear();
        _floorInstance = null;
        _boundaryParent = null;
        _interiorParent = null;
    }

    private void CreateFloor()
    {
        // Use a Plane primitive. We'll set transform.localScale.x to equal mapSize
        _floorInstance = GameObject.CreatePrimitive(PrimitiveType.Plane);
        _floorInstance.name = "ProceduralFloor";
        _floorInstance.transform.parent = transform;
        _floorInstance.transform.localPosition = new Vector3(0f, floorYOffset, 0f);

        // Set localScale.x so other scripts that read localScale.x get mapSize directly.
        // Note: default plane size is 10 units; we store mapSize in localScale.x for compatibility.
        _floorInstance.transform.localScale = new Vector3(mapSize, 1f, mapSize);

        var mr = _floorInstance.GetComponent<MeshRenderer>();
        if (floorMaterial != null)
            mr.material = floorMaterial;

        // Keep the collider (Plane has MeshCollider by default). If desired, you can replace/adjust it.
    }

    private void CreateBoundary()
    {
        _boundaryParent = new GameObject("ProceduralBoundary");
        _boundaryParent.transform.parent = transform;
        _boundaryParent.transform.localPosition = Vector3.zero;

        float half = mapSize * 0.5f;
        // Create four sides using either the provided prefab or a cube primitive
        // Two walls along X (north/south) and two along Z (east/west)
        CreateBoundaryWall(new Vector3(0f, boundaryHeight * 0.5f + floorYOffset, half + boundaryThickness * 0.5f),
                           new Vector3(mapSize + boundaryThickness * 2f, boundaryHeight, boundaryThickness), "North");
        CreateBoundaryWall(new Vector3(0f, boundaryHeight * 0.5f + floorYOffset, -half - boundaryThickness * 0.5f),
                           new Vector3(mapSize + boundaryThickness * 2f, boundaryHeight, boundaryThickness), "South");
        CreateBoundaryWall(new Vector3(half + boundaryThickness * 0.5f, boundaryHeight * 0.5f + floorYOffset, 0f),
                           new Vector3(boundaryThickness, boundaryHeight, mapSize + boundaryThickness * 2f), "East");
        CreateBoundaryWall(new Vector3(-half - boundaryThickness * 0.5f, boundaryHeight * 0.5f + floorYOffset, 0f),
                           new Vector3(boundaryThickness, boundaryHeight, mapSize + boundaryThickness * 2f), "West");
    }

    private void CreateBoundaryWall(Vector3 localPos, Vector3 localScale, string name)
    {
        GameObject wall;
        if (boundaryWallPrefab != null)
            wall = Instantiate(boundaryWallPrefab, _boundaryParent.transform);
        else
            wall = GameObject.CreatePrimitive(PrimitiveType.Cube);

        wall.name = $"Boundary_{name}";
        wall.transform.parent = _boundaryParent.transform;
        wall.transform.localPosition = localPos;
        wall.transform.localRotation = Quaternion.identity;
        wall.transform.localScale = localScale;

        wall.tag = "Wall";
        if (wall.GetComponent<Collider>() == null)
            wall.AddComponent<BoxCollider>();
    }

    private void CreateInteriorWalls()
    {
        _interiorParent = new GameObject("InteriorWalls");
        _interiorParent.transform.parent = transform;
        _interiorParent.transform.localPosition = Vector3.zero;

        float half = mapSize * 0.5f;
        int attemptsLimit = Mathf.Max(20, innerWallCount * 10);

        for (int i = 0; i < innerWallCount; i++)
        {
            bool placed = false;
            int attempts = 0;

            while (!placed && attempts < attemptsLimit)
            {
                attempts++;
                float length = Random.Range(innerWallMinLength, innerWallMaxLength);
                float thickness = innerWallThickness;
                float height = innerWallHeight;

                // Randomly orient along X or Z (0 or 90 degrees), optionally allow arbitrary rotation by using angle
                bool alignX = Random.value > 0.5f;
                float angle = alignX ? 0f : 90f;

                // compute extents for placement margin
                float extX = alignX ? length * 0.5f : thickness * 0.5f;
                float extZ = alignX ? thickness * 0.5f : length * 0.5f;

                float minX = -half + placementMargin + extX;
                float maxX = half - placementMargin - extX;
                float minZ = -half + placementMargin + extZ;
                float maxZ = half - placementMargin - extZ;

                if (minX > maxX || minZ > maxZ)
                    break; // map too small for this wall configuration

                float posX = Random.Range(minX, maxX);
                float posZ = Random.Range(minZ, maxZ);
                Vector3 pos = new Vector3(posX, height * 0.5f + floorYOffset, posZ);

                // Simple overlap check with existing interior walls and center area
                Bounds newBounds = new Bounds(pos, new Vector3(alignX ? length : thickness, height, alignX ? thickness : length));
                bool overlap = false;

                // Avoid placement too close to center (agent start) - reserve center radius
                float centerReserve = 1.0f + placementMargin;
                if (Mathf.Abs(posX) < centerReserve && Mathf.Abs(posZ) < centerReserve)
                    overlap = true;

                foreach (var existing in _spawnedInterior)
                {
                    if (existing == null) continue;
                    Bounds b = existing.GetComponent<Renderer>()?.bounds ?? new Bounds(existing.transform.position, existing.transform.localScale);
                    if (b.Intersects(newBounds))
                    {
                        overlap = true;
                        break;
                    }
                }

                if (overlap) continue;

                GameObject wall;
                if (innerWallPrefab != null)
                    wall = Instantiate(innerWallPrefab, _interiorParent.transform);
                else
                    wall = GameObject.CreatePrimitive(PrimitiveType.Cube);

                wall.name = $"InnerWall_{i}";
                wall.transform.parent = _interiorParent.transform;
                wall.transform.localPosition = pos;
                wall.transform.localRotation = Quaternion.Euler(0f, angle, 0f);
                wall.transform.localScale = new Vector3(alignX ? length : thickness, height, alignX ? thickness : length);

                wall.tag = "LocalWall";
                if (wall.GetComponent<Collider>() == null)
                    wall.AddComponent<BoxCollider>();

                _spawnedInterior.Add(wall);
                placed = true;
            }
        }
    }

    
}