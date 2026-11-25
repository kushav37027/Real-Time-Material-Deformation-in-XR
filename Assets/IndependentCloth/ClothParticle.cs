using UnityEngine;

public class ClothParticle
{
    public Vector3 position, predictedPosition, velocity;
    public readonly Vector3 restPosition;
    public Vector3 pinTarget;
    public bool isPinned;
    private float mass, inverseMass;
    public float Mass { get { return mass; } }
    public float InverseMass { get { return isPinned ? 0f : inverseMass; } }

    public ClothParticle(Vector3 startPosition, float mass, bool isPinned = false)
    {
        restPosition = startPosition;
        pinTarget = startPosition;
        this.isPinned = isPinned;
        SetMass(mass);
        Reset();
    }
    public void SetMass(float value)
    {
        if (float.IsNaN(value) || float.IsInfinity(value) || value <= 0f)
            throw new System.ArgumentOutOfRangeException(nameof(value));
        mass = value;
        inverseMass = 1f / value;
    }
    public void Reset()
    {
        position = predictedPosition = restPosition;
        pinTarget = restPosition;
        velocity = Vector3.zero;
    }
}
