using UnityEngine;

public class TetraSimulation : MonoBehaviour
{
    Particle[] particles;

    public GameObject particleVisualPrefab;
    GameObject[] particleVisuals;

    VolumeConstraint volumeConstraint;
    StretchConstraint[] stretchConstraints;

    int solverIterations = 50, substeps = 5;
    float initialVolume;
    int measurementSteps = 0;

    void Start()
    {
        Debug.Log("Gravity: " + Physics.gravity);

        if (particleVisualPrefab == null)
        {
            Debug.LogError("Assign Particle Visual Prefab in the Inspector.");
            enabled = false;
            return;
        }

        particles = new Particle[4];

        particles[0] = new Particle();
        particles[0].position = new Vector3(0, 1, 0);
        particles[0].inverseMass = 0.0f;

        particles[1] = new Particle();
        particles[1].position = new Vector3(-1f, 0f, -0.5773503f);
        particles[1].inverseMass = 1.0f;

        particles[2] = new Particle();
        particles[2].position = new Vector3(1f, 0f, -0.5773503f);
        particles[2].inverseMass = 1.0f;

        particles[3] = new Particle();
        particles[3].position = new Vector3(0f, 0f, 1.1547006f);
        particles[3].inverseMass = 1.0f;

        for (int i = 0; i < particles.Length; i++)
        {
            particles[i].predictedPosition = particles[i].position;
            particles[i].velocity = Vector3.zero;
        }

        // Zero compliance requests volume preservation.
        volumeConstraint = new VolumeConstraint(0, 1, 2, 3, 0.0f);
        volumeConstraint.InitializeRestVolume(particles);

        stretchConstraints = new StretchConstraint[6];

        int edge = 0;

        for (int i = 0; i < particles.Length; i++)
        {
            for (int j = i + 1; j < particles.Length; j++)
            {
                float restLength =
                    (particles[i].position - particles[j].position).magnitude;

                stretchConstraints[edge] =
                    new StretchConstraint(i, j, restLength, 0.01f);

                edge++;
            }
        }

        particleVisuals = new GameObject[particles.Length];

        for (int i = 0; i < particles.Length; i++)
        {
            particleVisuals[i] = Instantiate(
                particleVisualPrefab,
                particles[i].position,
                Quaternion.identity
            );
        }

        initialVolume = CalculateVolume();
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime/substeps;

        for (int substep = 0; substep<substeps; substep++){
            // 1. Reset accumulated multipliers for this timestep.
            volumeConstraint.ResetLambda();

            for (int i = 0; i < stretchConstraints.Length; i++)
            {
                stretchConstraints[i].ResetLambda();
            }

            // 2. Apply gravity and predict positions.
            for (int i = 0; i < particles.Length; i++)
            {
                if (particles[i].inverseMass > 0.0f)
                {
                    particles[i].velocity += Physics.gravity * dt;

                    particles[i].predictedPosition =
                        particles[i].position + particles[i].velocity * dt;
                }
                else
                {
                    particles[i].predictedPosition = particles[i].position;
                    particles[i].velocity = Vector3.zero;
                }
            }

            // 3. Solve stretch and volume constraints.
            for (int iteration = 0; iteration < solverIterations; iteration++)
            {
                for (int i = stretchConstraints.Length - 1; i >= 0; i--)
                {
                    stretchConstraints[i].Solve(particles, dt);
                }

                volumeConstraint.Solve(particles, dt);
            }

            // 4. Update velocities, positions, and visuals.
            for (int i = 0; i < particles.Length; i++)
            {
                if (particles[i].inverseMass > 0.0f)
                {
                    particles[i].velocity =
                        (particles[i].predictedPosition - particles[i].position) / dt;

                    particles[i].position = particles[i].predictedPosition;
                }

                particleVisuals[i].transform.position = particles[i].position;
            }
        }
        // 5. Measure the accepted configuration.
        measurementSteps++;

        if (measurementSteps % 25 == 0)
        {
            float currentVolume = CalculateVolume();

            float errorPercent =
                Mathf.Abs(currentVolume - initialVolume)
                / Mathf.Abs(initialVolume) * 100.0f;

            float averageY =
                (particles[1].position.y +
                particles[2].position.y +
                particles[3].position.y) / 3.0f;

            float maxSpeed = Mathf.Max(
                particles[1].velocity.magnitude,
                Mathf.Max(
                    particles[2].velocity.magnitude,
                    particles[3].velocity.magnitude
                )
            );

            Debug.Log(
                $"Volume: {currentVolume:F6} | " +
                $"Error: {errorPercent:F8}% | " +
                $"Average height: {averageY:F5} | " +
                $"Max speed: {maxSpeed:F5}"
            );
        }
    }

    float CalculateVolume()
    {
        Vector3 p0 = particles[0].position;
        Vector3 p1 = particles[1].position;
        Vector3 p2 = particles[2].position;
        Vector3 p3 = particles[3].position;

        return Vector3.Dot(
            Vector3.Cross(p1 - p0, p2 - p0),
            p3 - p0
        ) / 6.0f;
    }
}

public class Particle
{
    public Vector3 position;
    public Vector3 predictedPosition;
    public Vector3 velocity;
    public float inverseMass;
}

public class StretchConstraint
{
    int particle1;
    int particle2;

    float restLength;
    float compliance;
    float lambda;

    public StretchConstraint(
        int particle1,
        int particle2,
        float restLength,
        float compliance
    )
    {
        this.particle1 = particle1;
        this.particle2 = particle2;
        this.restLength = restLength;
        this.compliance = compliance;

        lambda = 0.0f;
    }

    public void ResetLambda()
    {
        lambda = 0.0f;
    }

    public void Solve(Particle[] particles, float dt)
    {
        Vector3 p1 = particles[particle1].predictedPosition;
        Vector3 p2 = particles[particle2].predictedPosition;

        Vector3 difference = p1 - p2;
        float length = difference.magnitude;

        if (length < 0.000001f)
        {
            return;
        }

        Vector3 n = difference / length;

        float C = length - restLength;

        float w1 = particles[particle1].inverseMass;
        float w2 = particles[particle2].inverseMass;

        float alpha = compliance / (dt * dt);
        float denominator = w1 + w2 + alpha;

        if (denominator <= 0.0f)
        {
            return;
        }

        float deltaLambda = (-C - alpha * lambda) / denominator;

        lambda += deltaLambda;

        particles[particle1].predictedPosition += w1 * deltaLambda * n;
        particles[particle2].predictedPosition -= w2 * deltaLambda * n;
    }
}

public class VolumeConstraint
{
    int particle1;
    int particle2;
    int particle3;
    int particle4;

    float restVolume;
    float compliance;
    float lambda;

    public VolumeConstraint(
        int particle1,
        int particle2,
        int particle3,
        int particle4,
        float compliance
    )
    {
        this.particle1 = particle1;
        this.particle2 = particle2;
        this.particle3 = particle3;
        this.particle4 = particle4;
        this.compliance = compliance;

        lambda = 0.0f;
    }

    public void InitializeRestVolume(Particle[] particles)
    {
        Vector3 p1 = particles[particle1].position;
        Vector3 p2 = particles[particle2].position;
        Vector3 p3 = particles[particle3].position;
        Vector3 p4 = particles[particle4].position;

        restVolume = Vector3.Dot(
            Vector3.Cross(p2 - p1, p3 - p1),
            p4 - p1
        ) / 6.0f;

        Debug.Log("Rest volume: " + restVolume);
    }

    public void ResetLambda()
    {
        lambda = 0.0f;
    }

    public void Solve(Particle[] particles, float dt)
    {
        Vector3 p1 = particles[particle1].predictedPosition;
        Vector3 p2 = particles[particle2].predictedPosition;
        Vector3 p3 = particles[particle3].predictedPosition;
        Vector3 p4 = particles[particle4].predictedPosition;

        float volume = Vector3.Dot(
            Vector3.Cross(p2 - p1, p3 - p1),
            p4 - p1
        ) / 6.0f;

        float C = volume - restVolume;

        Vector3 gradient1 =
            -Vector3.Cross(p3 - p2, p4 - p2) / 6.0f;

        Vector3 gradient2 =
            -Vector3.Cross(p4 - p1, p3 - p1) / 6.0f;

        Vector3 gradient3 =
            -Vector3.Cross(p2 - p1, p4 - p1) / 6.0f;

        Vector3 gradient4 =
            -Vector3.Cross(p3 - p1, p2 - p1) / 6.0f;

        float w1 = particles[particle1].inverseMass;
        float w2 = particles[particle2].inverseMass;
        float w3 = particles[particle3].inverseMass;
        float w4 = particles[particle4].inverseMass;

        float alpha = compliance / (dt * dt);

        float denominator =
            w1 * gradient1.sqrMagnitude +
            w2 * gradient2.sqrMagnitude +
            w3 * gradient3.sqrMagnitude +
            w4 * gradient4.sqrMagnitude +
            alpha;

        if (denominator <= 0.0f)
        {
            return;
        }

        float deltaLambda = (-C - alpha * lambda) / denominator;

        lambda += deltaLambda;

        particles[particle1].predictedPosition +=
            w1 * deltaLambda * gradient1;

        particles[particle2].predictedPosition +=
            w2 * deltaLambda * gradient2;

        particles[particle3].predictedPosition +=
            w3 * deltaLambda * gradient3;

        particles[particle4].predictedPosition +=
            w4 * deltaLambda * gradient4;
    }
}