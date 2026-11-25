using UnityEngine;

// Static analytic floor and sphere, using particle contacts (not cloth self-collision).
public class ClothContact
{
    public bool floorEnabled = true;
    public float floorHeight = 0f;
    public float thickness = 0.005f;
    public float staticFriction = 0.5f;
    public float slidingFriction = 0.3f;
    public bool sphereEnabled;
    public Vector3 sphereCenter;
    public float sphereRadius = 0.3f;

    private readonly float[,] normalLambda;
    private readonly Vector3[,] normals;
    private readonly bool[] swept;
    private readonly Vector3[] hitPoints;
    private readonly Vector3[] hitNormals;

    public ClothContact(int particleCount)
    {
        normalLambda = new float[particleCount, 2];
        normals = new Vector3[particleCount, 2];
        swept = new bool[particleCount];
        hitPoints = new Vector3[particleCount];
        hitNormals = new Vector3[particleCount];
    }

    public void BeginSubstep(ClothParticle[] particles)
    {
        System.Array.Clear(normalLambda, 0, normalLambda.Length);
        System.Array.Clear(swept, 0, swept.Length);
        if (!sphereEnabled) return;
        float radius = sphereRadius + thickness;
        for (int i = 0; i < particles.Length; i++)
        {
            Vector3 p = particles[i].position - sphereCenter;
            Vector3 movement = particles[i].predictedPosition - particles[i].position;
            float a = movement.sqrMagnitude;
            float b = Vector3.Dot(p, movement);
            float c = p.sqrMagnitude - radius * radius;
            if (c <= 0f || a < 1e-14f || b >= 0f) continue;
            float discriminant = b * b - a * c;
            if (discriminant < 0f) continue;
            float t = (-b - Mathf.Sqrt(discriminant)) / a;
            if (t < 0f || t > 1f) continue;
            swept[i] = true;
            hitPoints[i] = particles[i].position + t * movement;
            hitNormals[i] = (hitPoints[i] - sphereCenter).normalized;
        }
    }

    private void Project(ClothParticle particle, int i, int slot, float distance, Vector3 normal)
    {
        float w = particle.InverseMass;
        if (w <= 0f) return;
        float nextLambda = Mathf.Max(0f, normalLambda[i, slot] - distance / w);
        particle.predictedPosition += w * (nextLambda - normalLambda[i, slot]) * normal;
        normalLambda[i, slot] = nextLambda;
        normals[i, slot] = normal;
    }

    public void Solve(ClothParticle[] particles)
    {
        for (int i = 0; i < particles.Length; i++)
        {
            ClothParticle p = particles[i];
            if (floorEnabled)
                Project(p, i, 0, p.predictedPosition.y - floorHeight - thickness, Vector3.up);
            if (!sphereEnabled) continue;
            if (swept[i])
            {
                // Conservative impact tangent plane, retained only for the current substep.
                float distance = Vector3.Dot(p.predictedPosition - hitPoints[i], hitNormals[i]);
                Project(p, i, 1, distance, hitNormals[i]);
            }
            else
            {
                Vector3 delta = p.predictedPosition - sphereCenter;
                float length = delta.magnitude;
                Vector3 normal = length > 1e-8f ? delta / length : Vector3.up;
                Project(p, i, 1, length - sphereRadius - thickness, normal);
            }
        }
    }

    public void ApplyFriction(ClothParticle[] particles)
    {
        float muStatic = Mathf.Max(0f, staticFriction);
        float muSliding = Mathf.Clamp(slidingFriction, 0f, muStatic);
        for (int i = 0; i < particles.Length; i++)
        {
            ClothParticle p = particles[i];
            for (int slot = 0; slot < 2; slot++)
            {
                if (normalLambda[i, slot] <= 0f || p.InverseMass <= 0f) continue;
                Vector3 normal = normals[i, slot];
                Vector3 movement = p.predictedPosition - p.position;
                Vector3 tangent = movement - Vector3.Dot(movement, normal) * normal;
                float length = tangent.magnitude;
                if (length < 1e-9f) continue;
                float budget = p.InverseMass * normalLambda[i, slot];
                float correction = length <= muStatic * budget ? length : Mathf.Min(length, muSliding * budget);
                p.predictedPosition -= tangent * (correction / length);
            }
        }
    }
}
