using UnityEngine;
using System.Collections.Generic;

public class BendingSimulation : MonoBehaviour
{
    const int rows = 7;
    const int columns = 7;

    [Header("Rendering")]
    public Material clothMaterial;
    public bool showGizmos = true;

    [Header("Forces — total force per patch, in newtons")]
    public Vector3 upperForce = new Vector3(0f, 0f, 0.001f);
    public Vector3 lowerForce = new Vector3(0f, 0f, -0.001f);

    [Min(0f)]
    public float forceDuration = 2f;

    [Header("Physics — restart Play after changing compliance")]
    [Min(0f)]
    public float stretchCompliance = 0.0001f;

    [Min(0f)]
    public float bendingCompliance = 1000f;

    [Min(1)]
    public int solverIterations = 10;

    [Header("Debug")]
    public bool logDisplacement = false;

    float spacing = 0.1f;
    float elapsedTime = 0f;
    int measurementSteps = 0;

    SheetParticle[] particles;
    int[] triangles;

    Mesh clothMesh;
    Vector3[] meshVertices;

    List<SheetStretchConstraint> stretchConstraints =
        new List<SheetStretchConstraint>();

    List<SheetBendingConstraint> bendingConstraints =
        new List<SheetBendingConstraint>();

    void Start()
    {
        CreateParticles();
        CreateTriangles();
        AssignMasses();

        CreateStretchConstraints();
        CreateBendingConstraints();

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

                particle.position = new Vector3(
                    column * spacing,
                    (row - (rows - 1) * 0.5f) * spacing,
                    0f
                );

                particle.predictedPosition = particle.position;
                particle.velocity = Vector3.zero;

                // Fix the first two columns.
                particle.isFixed = column < 2;

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
        float massPerArea = 0.2f;

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
            particle.inverseMass = particle.isFixed
                ? 0f
                : 1f / particle.mass;
        }
    }

    void CreateStretchConstraints()
    {
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

    bool IsUpperPatch(int row, int column)
    {
        return column == columns - 1 &&
               (row == 4 || row == 5);
    }

    bool IsLowerPatch(int row, int column)
    {
        return column == columns - 1 &&
               (row == 1 || row == 2);
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        if (particles == null || dt <= 0f)
            return;

        // Reset once per timestep, not per solver iteration.
        foreach (SheetStretchConstraint constraint in stretchConstraints)
            constraint.ResetLambda();

        foreach (SheetBendingConstraint constraint in bendingConstraints)
            constraint.ResetLambda();

        // Apply external forces and predict positions.
        for (int i = 0; i < particles.Length; i++)
        {
            SheetParticle particle = particles[i];

            if (particle.isFixed)
            {
                particle.predictedPosition = particle.position;
                particle.velocity = Vector3.zero;
                continue;
            }

            int row = i / columns;
            int column = i % columns;

            Vector3 force = Vector3.zero;

            if (elapsedTime < forceDuration)
            {
                // Each patch contains two particles.
                if (IsUpperPatch(row, column))
                    force += upperForce / 2f;

                if (IsLowerPatch(row, column))
                    force += lowerForce / 2f;
            }

            particle.velocity +=
                force * particle.inverseMass * dt;

            particle.predictedPosition =
                particle.position + particle.velocity * dt;
        }

        // Solve both constraint types on predicted positions.
        int iterations = Mathf.Max(1, solverIterations);

        for (int iteration = 0; iteration < iterations; iteration++)
        {
            foreach (SheetStretchConstraint constraint in stretchConstraints)
                constraint.Solve(particles, dt);

            foreach (SheetBendingConstraint constraint in bendingConstraints)
                constraint.Solve(particles, dt);
        }

        // Reconstruct velocities and accept corrected positions.
        foreach (SheetParticle particle in particles)
        {
            if (particle.isFixed)
                continue;

            particle.velocity =
                (particle.predictedPosition - particle.position) / dt;

            particle.position = particle.predictedPosition;
        }

        elapsedTime += dt;
        measurementSteps++;

        if (logDisplacement && measurementSteps % 25 == 0)
        {
            int upper = 4 * columns + columns - 1;
            int lower = 2 * columns + columns - 1;

            Debug.Log(
                $"Upper Z: {particles[upper].position.z:F6} m | " +
                $"Lower Z: {particles[lower].position.z:F6} m"
            );
        }
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

        // Separate rendering vertices for the front and back.
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
                "Assign Cloth Material on BendingSimulation " +
                "to display the surface correctly."
            );
        }
    }

    void CopyParticlePositions()
    {
        int count = particles.Length;

        for (int i = 0; i < count; i++)
        {
            // Simulation positions are world-space.
            // Mesh vertices are local to this GameObject.
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
            int row = i / columns;
            int column = i % columns;

            if (particles[i].isFixed)
                Gizmos.color = Color.red;
            else if (IsUpperPatch(row, column))
                Gizmos.color = Color.yellow;
            else if (IsLowerPatch(row, column))
                Gizmos.color = Color.cyan;
            else
                Gizmos.color = Color.green;

            Gizmos.DrawSphere(particles[i].position, 0.012f);
        }
    }

    void OnDestroy()
    {
        if (clothMesh != null)
            Destroy(clothMesh);
    }
}

public class SheetParticle
{
    public Vector3 position;
    public Vector3 predictedPosition;
    public Vector3 velocity;

    public float mass;
    public float inverseMass;
    public bool isFixed;
}

public class SheetStretchConstraint
{
    int a, b;

    float restLength;
    float compliance;
    float lambda;

    public SheetStretchConstraint(
        int a,
        int b,
        float restLength,
        float compliance)
    {
        this.a = a;
        this.b = b;
        this.restLength = restLength;
        this.compliance = compliance;
    }

    public void ResetLambda()
    {
        lambda = 0f;
    }

    public void Solve(SheetParticle[] particles, float dt)
    {
        Vector3 difference =
            particles[a].predictedPosition -
            particles[b].predictedPosition;

        float length = difference.magnitude;

        if (length < 0.000001f)
            return;

        float wa = particles[a].inverseMass;
        float wb = particles[b].inverseMass;

        if (wa + wb == 0f)
            return;

        Vector3 direction = difference / length;
        float C = length - restLength;

        float alphaTilde = compliance / (dt * dt);

        float deltaLambda =
            (-C - alphaTilde * lambda) /
            (wa + wb + alphaTilde);

        lambda += deltaLambda;

        particles[a].predictedPosition +=
            wa * deltaLambda * direction;

        particles[b].predictedPosition -=
            wb * deltaLambda * direction;
    }
}

public class SheetBendingConstraint
{
    int a, b, c, d;

    float restAngle;
    float compliance;
    float lambda;

    public SheetBendingConstraint(
        int a,
        int b,
        int c,
        int d,
        SheetParticle[] particles,
        float compliance)
    {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
        this.compliance = compliance;

        restAngle = CalculateAngle(
            particles[a].position,
            particles[b].position,
            particles[c].position,
            particles[d].position
        );
    }

    public void ResetLambda()
    {
        lambda = 0f;
    }

    static float CalculateAngle(
        Vector3 p0,
        Vector3 p1,
        Vector3 p2,
        Vector3 p3)
    {
        Vector3 edge = p1 - p0;

        Vector3 n1 =
            Vector3.Cross(edge, p2 - p0).normalized;

        Vector3 n2 =
            Vector3.Cross(p3 - p0, edge).normalized;

        return Mathf.Atan2(
            Vector3.Dot(edge.normalized, Vector3.Cross(n1, n2)),
            Vector3.Dot(n1, n2)
        );
    }

    public void Solve(SheetParticle[] particles, float dt)
    {
        Vector3 p0 = particles[a].predictedPosition;
        Vector3 p1 = particles[b].predictedPosition;
        Vector3 p2 = particles[c].predictedPosition;
        Vector3 p3 = particles[d].predictedPosition;

        Vector3 edge = p1 - p0;
        float edgeSquared = edge.sqrMagnitude;

        Vector3 normal1 = Vector3.Cross(edge, p2 - p0);
        Vector3 normal2 = Vector3.Cross(p3 - p0, edge);

        // Skip nearly collapsed edges or triangles.
        if (edgeSquared < 1e-12f ||
            normal1.sqrMagnitude < 1e-16f ||
            normal2.sqrMagnitude < 1e-16f)
        {
            return;
        }

        float edgeLength = Mathf.Sqrt(edgeSquared);

        float angle = CalculateAngle(p0, p1, p2, p3);
        float C = angle - restAngle;

        // Wrap the angle error into [-PI, PI].
        C = Mathf.Atan2(Mathf.Sin(C), Mathf.Cos(C));

        Vector3 g2 =
            -edgeLength * normal1 / normal1.sqrMagnitude;

        Vector3 g3 =
            -edgeLength * normal2 / normal2.sqrMagnitude;

        float t2 = Vector3.Dot(p2 - p0, edge) / edgeSquared;
        float t3 = Vector3.Dot(p3 - p0, edge) / edgeSquared;

        Vector3 g0 = -(1f - t2) * g2 - (1f - t3) * g3;
        Vector3 g1 = -t2 * g2 - t3 * g3;

        float w0 = particles[a].inverseMass;
        float w1 = particles[b].inverseMass;
        float w2 = particles[c].inverseMass;
        float w3 = particles[d].inverseMass;

        if (w0 + w1 + w2 + w3 == 0f)
            return;

        float alphaTilde = compliance / (dt * dt);

        float denominator =
            w0 * g0.sqrMagnitude +
            w1 * g1.sqrMagnitude +
            w2 * g2.sqrMagnitude +
            w3 * g3.sqrMagnitude +
            alphaTilde;

        if (denominator <= 0f)
            return;

        float deltaLambda =
            (-C - alphaTilde * lambda) / denominator;

        lambda += deltaLambda;

        particles[a].predictedPosition += w0 * deltaLambda * g0;
        particles[b].predictedPosition += w1 * deltaLambda * g1;
        particles[c].predictedPosition += w2 * deltaLambda * g2;
        particles[d].predictedPosition += w3 * deltaLambda * g3;
    }
}