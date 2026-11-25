using UnityEngine;
using System.Collections.Generic;

public class ClothSurfaceSimulation : MonoBehaviour
{
    const int rows = 7;
    const int columns = 7;
    const float spacing = 0.1f;

    [Header("Table Contact")]
    public BoxCollider tableCollider;

    [Min(0.0001f)]
    public float contactOffset = 0.005f;

    [Header("Table Friction")]
    [Min(0f)]
    public float staticFriction = 0.6f;

    [Min(0f)]
    public float slidingFriction = 0.4f;

    [Header("Physics - restart Play after changing compliance")]
    [Min(0f)]
    public float stretchCompliance = 0.0001f;

    [Min(0f)]
    public float bendingCompliance = 1000f;

    [Min(1)]
    public int solverIterations = 10;

    [Min(1)]
    public int substeps = 5;

    [Header("Rendering")]
    public Material clothMaterial;
    public bool showGizmos = true;

    SheetParticle[] particles;
    int[] triangles;

    Vector3[] contactNormals;
    bool[] touchingTable;

    Mesh clothMesh;
    Vector3[] meshVertices;

    readonly List<SheetStretchConstraint> stretchConstraints =
        new List<SheetStretchConstraint>();

    readonly List<SheetBendingConstraint> bendingConstraints =
        new List<SheetBendingConstraint>();

    void Start()
    {
        Time.timeScale = 0.25f; // Quarter speed
        if (tableCollider == null)
        {
            Debug.LogError(
                "Assign the table's BoxCollider on ClothSurfaceSimulation."
            );

            enabled = false;
            return;
        }

        CreateParticles();
        CreateTriangles();
        AssignMasses();

        CreateStretchConstraints();
        CreateBendingConstraints();

        contactNormals = new Vector3[particles.Length];
        touchingTable = new bool[particles.Length];

        CreateClothMesh();

        Debug.Log(
            $"Particles: {particles.Length} | " +
            $"Triangles: {triangles.Length / 3} | " +
            $"Stretch constraints: {stretchConstraints.Count} | " +
            $"Bending constraints: {bendingConstraints.Count}"
        );
    }

    void CreateParticles()
    {
        particles = new SheetParticle[rows * columns];

        for (int row = 0; row < rows; row++)
        {
            for (int column = 0; column < columns; column++)
            {
                int index = row * columns + column;

                SheetParticle particle = new SheetParticle();

                // Horizontal cloth centered at world X = 0, Z = 0.
                particle.position = new Vector3(
                    (column - (columns - 1) * 0.5f) * spacing,
                    0.5f,
                    (row - (rows - 1) * 0.5f) * spacing
                );

                particle.predictedPosition = particle.position;
                particle.velocity = Vector3.zero;

                // All particles are free for the table-drop demo.
                particle.isFixed = false;

                particles[index] = particle;
            }
        }
    }

    void CreateTriangles()
    {
        int triangleCount = (rows - 1) * (columns - 1) * 2;
        triangles = new int[triangleCount * 3];

        int next = 0;

        for (int row = 0; row < rows - 1; row++)
        {
            for (int column = 0; column < columns - 1; column++)
            {
                int a = row * columns + column;
                int b = a + 1;
                int c = a + columns;
                int d = c + 1;

                triangles[next++] = a;
                triangles[next++] = b;
                triangles[next++] = c;

                triangles[next++] = b;
                triangles[next++] = d;
                triangles[next++] = c;
            }
        }
    }

    void AssignMasses()
    {
        const float massPerArea = 0.2f;

        foreach (SheetParticle particle in particles)
            particle.mass = 0f;

        for (int t = 0; t < triangles.Length; t += 3)
        {
            int a = triangles[t];
            int b = triangles[t + 1];
            int c = triangles[t + 2];

            Vector3 edge1 =
                particles[b].position - particles[a].position;

            Vector3 edge2 =
                particles[c].position - particles[a].position;

            float area =
                Vector3.Cross(edge1, edge2).magnitude * 0.5f;

            float massShare = massPerArea * area / 3f;

            particles[a].mass += massShare;
            particles[b].mass += massShare;
            particles[c].mass += massShare;
        }

        foreach (SheetParticle particle in particles)
        {
            particle.inverseMass =
                particle.isFixed || particle.mass <= 0f
                ? 0f
                : 1f / particle.mass;
        }
    }

    void CreateStretchConstraints()
    {
        stretchConstraints.Clear();

        for (int row = 0; row < rows; row++)
        {
            for (int column = 0; column < columns; column++)
            {
                int a = row * columns + column;

                if (column < columns - 1)
                    AddStretch(a, a + 1);

                if (row < rows - 1)
                    AddStretch(a, a + columns);

                if (column < columns - 1 && row < rows - 1)
                    AddStretch(a + 1, a + columns);
            }
        }
    }

    void AddStretch(int a, int b)
    {
        float restLength = Vector3.Distance(
            particles[a].position,
            particles[b].position
        );

        stretchConstraints.Add(
            new SheetStretchConstraint(
                a,
                b,
                restLength,
                Mathf.Max(0f, stretchCompliance)
            )
        );
    }

    void CreateBendingConstraints()
    {
        bendingConstraints.Clear();

        Dictionary<Vector2Int, Vector3Int> edges =
            new Dictionary<Vector2Int, Vector3Int>();

        for (int t = 0; t < triangles.Length; t += 3)
        {
            int a = triangles[t];
            int b = triangles[t + 1];
            int c = triangles[t + 2];

            RegisterBendingEdge(edges, a, b, c);
            RegisterBendingEdge(edges, b, c, a);
            RegisterBendingEdge(edges, c, a, b);
        }
    }

    void RegisterBendingEdge(
        Dictionary<Vector2Int, Vector3Int> edges,
        int a,
        int b,
        int opposite)
    {
        Vector2Int key = new Vector2Int(
            Mathf.Min(a, b),
            Mathf.Max(a, b)
        );

        if (edges.TryGetValue(key, out Vector3Int first))
        {
            bendingConstraints.Add(
                new SheetBendingConstraint(
                    first.x,
                    first.y,
                    first.z,
                    opposite,
                    particles,
                    Mathf.Max(0f, bendingCompliance)
                )
            );
        }
        else
        {
            edges.Add(key, new Vector3Int(a, b, opposite));
        }
    }

    void FixedUpdate()
    {
        if (particles == null || tableCollider == null)
            return;

        int steps = Mathf.Max(1, substeps);
        int iterations = Mathf.Max(1, solverIterations);

        float dt = Time.fixedDeltaTime / steps;

        for (int step = 0; step < steps; step++)
        {
            // 1. Reset XPBD multipliers for this substep.
            foreach (SheetStretchConstraint constraint in stretchConstraints)
                constraint.ResetLambda();

            foreach (SheetBendingConstraint constraint in bendingConstraints)
                constraint.ResetLambda();

            // 2. Apply gravity and predict positions.
            for (int i = 0; i < particles.Length; i++)
            {
                SheetParticle particle = particles[i];

                touchingTable[i] = false;
                contactNormals[i] = Vector3.zero;

                if (particle.inverseMass <= 0f)
                {
                    particle.velocity = Vector3.zero;
                    particle.predictedPosition = particle.position;
                    continue;
                }

                particle.velocity += Physics.gravity * dt;

                particle.predictedPosition =
                    particle.position + particle.velocity * dt;
            }

            // 3. Solve material constraints, contact, and friction.
            for (int iteration = 0; iteration < iterations; iteration++)
            {
                foreach (SheetStretchConstraint constraint in stretchConstraints)
                    constraint.Solve(particles, dt);

                foreach (SheetBendingConstraint constraint in bendingConstraints)
                    constraint.Solve(particles, dt);

                for (int i = 0; i < particles.Length; i++)
                    SolveTableContact(i);
            }

            // 4. Reconstruct velocities and accept positions.
            for (int i = 0; i < particles.Length; i++)
            {
                SheetParticle particle = particles[i];

                if (particle.inverseMass <= 0f)
                    continue;

                particle.velocity =
                    (particle.predictedPosition - particle.position) / dt;

                if (touchingTable[i])
                {
                    Vector3 normal = contactNormals[i];

                    float normalSpeed =
                        Vector3.Dot(particle.velocity, normal);

                    // No bounce: remove remaining inward velocity.
                    // Friction was already applied to predicted positions.
                    if (normalSpeed < 0f)
                        particle.velocity -= normalSpeed * normal;
                }

                particle.position = particle.predictedPosition;
            }
        }
    }

    void SolveTableContact(int index)
    {
        SheetParticle particle = particles[index];

        if (particle.inverseMass <= 0f)
            return;

        // Refresh contact state for this solver pass.
        touchingTable[index] = false;
        contactNormals[index] = Vector3.zero;

        Transform tableTransform = tableCollider.transform;

        Vector3 localPosition =
            tableTransform.InverseTransformPoint(
                particle.predictedPosition
            ) - tableCollider.center;

        Vector3 scale = tableTransform.lossyScale;

        scale = new Vector3(
            Mathf.Abs(scale.x),
            Mathf.Abs(scale.y),
            Mathf.Abs(scale.z)
        );

        if (scale.x < 0.000001f ||
            scale.y < 0.000001f ||
            scale.z < 0.000001f)
        {
            return;
        }

        Vector3 halfSize = tableCollider.size * 0.5f;
        float gap = Mathf.Max(0.0001f, contactOffset);

        Vector3 expandedHalfSize = halfSize + new Vector3(
            gap / scale.x,
            gap / scale.y,
            gap / scale.z
        );

        // Outside the expanded box: no penetration.
        if (Mathf.Abs(localPosition.x) > expandedHalfSize.x ||
            Mathf.Abs(localPosition.y) > expandedHalfSize.y ||
            Mathf.Abs(localPosition.z) > expandedHalfSize.z)
        {
            return;
        }

        // Find the nearest face in world-distance units.
        float distanceX =
            (expandedHalfSize.x - Mathf.Abs(localPosition.x)) * scale.x;

        float distanceY =
            (expandedHalfSize.y - Mathf.Abs(localPosition.y)) * scale.y;

        float distanceZ =
            (expandedHalfSize.z - Mathf.Abs(localPosition.z)) * scale.z;

        Vector3 localNormal;

        if (distanceX <= distanceY && distanceX <= distanceZ)
        {
            float sign = localPosition.x >= 0f ? 1f : -1f;

            localPosition.x = sign * expandedHalfSize.x;
            localNormal = new Vector3(sign, 0f, 0f);
        }
        else if (distanceY <= distanceZ)
        {
            float sign = localPosition.y >= 0f ? 1f : -1f;

            localPosition.y = sign * expandedHalfSize.y;
            localNormal = new Vector3(0f, sign, 0f);
        }
        else
        {
            float sign = localPosition.z >= 0f ? 1f : -1f;

            localPosition.z = sign * expandedHalfSize.z;
            localNormal = new Vector3(0f, 0f, sign);
        }

        Vector3 beforeContact = particle.predictedPosition;

        Vector3 correctedPosition =
            tableTransform.TransformPoint(
                localPosition + tableCollider.center
            );

        Vector3 normal =
            tableTransform.TransformDirection(localNormal).normalized;

        float normalCorrection = Mathf.Max(
            0f,
            Vector3.Dot(correctedPosition - beforeContact, normal)
        );

        // Push the particle out of the table.
        particle.predictedPosition = correctedPosition;

        // Tangential movement since the start of this substep.
        Vector3 displacement =
            particle.predictedPosition - particle.position;

        Vector3 tangentDisplacement =
            displacement - Vector3.Dot(displacement, normal) * normal;

        float tangentDistance = tangentDisplacement.magnitude;

        if (tangentDistance > 0.0000001f && normalCorrection > 0f)
        {
            float muStatic = Mathf.Max(0f, staticFriction);
            float muSliding = Mathf.Clamp(
                slidingFriction, 0f, muStatic
            );

            if (tangentDistance <= muStatic * normalCorrection)
            {
                // Static friction: cancel tangential movement.
                particle.predictedPosition -= tangentDisplacement;
            }
            else
            {
                // Sliding friction: oppose movement without reversing it.
                float correctionDistance = Mathf.Min(
                    muSliding * normalCorrection,
                    tangentDistance
                );

                particle.predictedPosition -=
                    tangentDisplacement / tangentDistance
                    * correctionDistance;
            }
        }

        contactNormals[index] = normal;
        touchingTable[index] = true;
    }

    void CreateClothMesh()
    {
        MeshFilter filter = GetComponent<MeshFilter>();

        if (filter == null)
            filter = gameObject.AddComponent<MeshFilter>();

        MeshRenderer meshRenderer = GetComponent<MeshRenderer>();

        if (meshRenderer == null)
            meshRenderer = gameObject.AddComponent<MeshRenderer>();

        clothMesh = new Mesh();
        clothMesh.name = "Simulated Cloth";
        clothMesh.MarkDynamic();

        int count = particles.Length;

        // Separate vertices give each side its own normals.
        meshVertices = new Vector3[count * 2];
        int[] renderTriangles = new int[triangles.Length * 2];

        for (int t = 0; t < triangles.Length; t += 3)
        {
            renderTriangles[t] = triangles[t];
            renderTriangles[t + 1] = triangles[t + 1];
            renderTriangles[t + 2] = triangles[t + 2];

            int back = triangles.Length + t;

            renderTriangles[back] = triangles[t] + count;
            renderTriangles[back + 1] = triangles[t + 2] + count;
            renderTriangles[back + 2] = triangles[t + 1] + count;
        }

        CopyParticlePositions();

        clothMesh.vertices = meshVertices;
        clothMesh.triangles = renderTriangles;
        clothMesh.RecalculateNormals();
        clothMesh.RecalculateBounds();

        filter.sharedMesh = clothMesh;

        if (clothMaterial != null)
        {
            meshRenderer.sharedMaterial = clothMaterial;
        }
        else
        {
            Debug.LogWarning(
                "Assign Cloth Material on ClothSurfaceSimulation."
            );
        }
    }

    void CopyParticlePositions()
    {
        int count = particles.Length;

        for (int i = 0; i < count; i++)
        {
            Vector3 localPosition =
                transform.InverseTransformPoint(particles[i].position);

            meshVertices[i] = localPosition;
            meshVertices[i + count] = localPosition;
        }
    }

    void LateUpdate()
    {
        if (clothMesh == null)
            return;

        CopyParticlePositions();

        clothMesh.vertices = meshVertices;
        clothMesh.RecalculateNormals();
        clothMesh.RecalculateBounds();
    }

    void OnDrawGizmos()
    {
        if (!showGizmos || particles == null || triangles == null)
            return;

        Gizmos.color = Color.white;

        for (int t = 0; t < triangles.Length; t += 3)
        {
            Vector3 a = particles[triangles[t]].position;
            Vector3 b = particles[triangles[t + 1]].position;
            Vector3 c = particles[triangles[t + 2]].position;

            Gizmos.DrawLine(a, b);
            Gizmos.DrawLine(b, c);
            Gizmos.DrawLine(c, a);
        }

        for (int i = 0; i < particles.Length; i++)
        {
            if (particles[i].inverseMass <= 0f)
            {
                Gizmos.color = Color.red;
            }
            else if (touchingTable != null && touchingTable[i])
            {
                Gizmos.color = Color.yellow;
            }
            else
            {
                Gizmos.color = Color.green;
            }

            Gizmos.DrawSphere(particles[i].position, 0.012f);
        }
    }

    void OnDestroy()
    {
        Time.timeScale = 1f;
        if (clothMesh != null)
            Destroy(clothMesh);
    }
}