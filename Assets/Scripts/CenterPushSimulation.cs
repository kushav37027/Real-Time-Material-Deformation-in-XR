using UnityEngine;
using System.Collections.Generic;

public class CenterPushSimulation : MonoBehaviour
{
    const int rows = 7;
    const int columns = 7;

    public float pushForce = 0.01f;
    public float pushDuration = 5f;

    public float stretchCompliance = 0.0001f;
    public float bendingCompliance = 1000f;

    float spacing = 0.1f;
    float elapsedTime = 0f;

    int solverIterations = 20;

    SheetParticle[] particles;
    int[] triangles;

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

        Debug.Log(
            $"Particles: {particles.Length}, " +
            $"Triangles: {triangles.Length / 3}, " +
            $"Stretch: {stretchConstraints.Count}, " +
            $"Bending: {bendingConstraints.Count}"
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

                // Centre the sheet around the world origin.
                particle.position = new Vector3(
                    (column - (columns - 1) * 0.5f) * spacing,
                    (row - (rows - 1) * 0.5f) * spacing,
                    0f
                );

                particle.predictedPosition = particle.position;
                particle.velocity = Vector3.zero;

                // Fix every particle on the four borders.
                particle.isFixed =
                    row == 0 ||
                    row == rows - 1 ||
                    column == 0 ||
                    column == columns - 1;

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

            Vector3 edge1 = particles[b].position - particles[a].position;
            Vector3 edge2 = particles[c].position - particles[a].position;

            float area = Vector3.Cross(edge1, edge2).magnitude * 0.5f;
            float massShare = massPerArea * area / 3f;

            particles[a].mass += massShare;
            particles[b].mass += massShare;
            particles[c].mass += massShare;
        }

        foreach (SheetParticle particle in particles)
        {
            particle.inverseMass =
                particle.isFixed ? 0f : 1f / particle.mass;
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
                a, b, restLength, stretchCompliance
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

            RegisterEdge(edges, a, b, c);
            RegisterEdge(edges, b, c, a);
            RegisterEdge(edges, c, a, b);
        }
    }

    void RegisterEdge(
        Dictionary<Vector2Int, Vector3Int> edges,
        int a, int b, int opposite)
    {
        Vector2Int key = new Vector2Int(
            Mathf.Min(a, b),
            Mathf.Max(a, b)
        );

        if (edges.TryGetValue(key, out Vector3Int first))
        {
            bendingConstraints.Add(
                new SheetBendingConstraint(
                    first.x, first.y, first.z, opposite,
                    particles, bendingCompliance
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
        float dt = Time.fixedDeltaTime;

        foreach (SheetStretchConstraint constraint in stretchConstraints)
            constraint.ResetLambda();

        foreach (SheetBendingConstraint constraint in bendingConstraints)
            constraint.ResetLambda();

        // Apply force and predict positions.
        for (int i = 0; i < particles.Length; i++)
        {
            SheetParticle particle = particles[i];

            if (particle.isFixed)
            {
                particle.velocity = Vector3.zero;
                particle.predictedPosition = particle.position;
                continue;
            }

            int row = i / columns;
            int column = i % columns;

            // A 3x3 patch around the centre.
            bool isPushed =
                Mathf.Abs(row - rows / 2) <= 1 &&
                Mathf.Abs(column - columns / 2) <= 1;

            Vector3 force = Vector3.zero;

            if (isPushed && elapsedTime < pushDuration)
            {
                // Divide the total force among nine particles.
                force = Vector3.forward * (pushForce / 9f);
            }

            particle.velocity += force * particle.inverseMass * dt;

            particle.predictedPosition =
                particle.position + particle.velocity * dt;
        }

        // Correct the predicted positions.
        for (int iteration = 0; iteration < solverIterations; iteration++)
        {
            foreach (SheetStretchConstraint constraint in stretchConstraints)
                constraint.Solve(particles, dt);

            foreach (SheetBendingConstraint constraint in bendingConstraints)
                constraint.Solve(particles, dt);
        }

        // Accept the corrected positions and reconstruct velocities.
        foreach (SheetParticle particle in particles)
        {
            if (particle.isFixed)
                continue;

            particle.velocity =
                (particle.predictedPosition - particle.position) / dt;

            particle.position = particle.predictedPosition;
        }

        elapsedTime += dt;
    }

    void OnDrawGizmos()
    {
        if (particles == null || triangles == null)
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

            bool isPushed =
                Mathf.Abs(row - rows / 2) <= 1 &&
                Mathf.Abs(column - columns / 2) <= 1;

            Gizmos.color = particles[i].isFixed
                ? Color.red
                : isPushed ? Color.yellow : Color.green;

            Gizmos.DrawSphere(particles[i].position, 0.008f);
        }
    }
}