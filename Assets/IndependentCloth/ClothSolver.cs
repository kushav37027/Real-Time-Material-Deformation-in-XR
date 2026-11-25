using System;
using System.Collections.Generic;
using UnityEngine;

// Simulation engine; has no dependency on mouse input, rendering or UI.
public class ClothSolver
{
    public ClothGeometry Geometry { get; private set; }
    public ClothParticle[] Particles { get; private set; }
    public Vector3[] PreviousPositions { get; private set; }
    private float lastDensity;
    // Updated on reset/rebuild to invalidate old display history.
    public int DisplayRevision { get; private set; }
    public ClothContact Contact { get; private set; }
    public int substeps = 6;
    public int solverIterations = 10;
    public Vector3 gravity = new Vector3(0f, -9.81f, 0f);
    public float grabCompliance = 0.00002f;
    public Vector3[] ExternalForces { get; private set; }
    public int HingeCount { get { return bending.Count; } }
    private readonly List<ClothShellConstraint> edgesOnly = new List<ClothShellConstraint>();
    private readonly List<ClothShellConstraint> areas = new List<ClothShellConstraint>();
    // Two independent grab slots: 0 = right hand, 1 = left hand.
    public bool IsGrabbing { get { return grabs[0].triangleIndex >= 0 || grabs[1].triangleIndex >= 0; } }
    public bool IsGrabbingWith(int grabId)
    {
        return grabId >= 0 && grabId < grabs.Length && grabs[grabId].triangleIndex >= 0;
    }

    private readonly ClothMembraneConstraint[] membrane;
    private readonly List<ClothBendingConstraint> bending = new List<ClothBendingConstraint>();

    private class GrabState
    {
        public int triangleIndex = -1;
        public Vector3 weights;
        public Vector3 target;
        public Vector3 lambda;
    }

    private readonly GrabState[] grabs = { new GrabState(), new GrabState() };

    private class Edge
    {
        public int a, b, opposite, face;
        public bool paired;
    }

    public ClothSolver(ClothGeometry geometry, ClothMaterial material, bool pinTopCorners = true)
    {
        if (geometry == null) throw new ArgumentNullException(nameof(geometry));
        Geometry = geometry;
        Particles = geometry.CreateParticles(material, pinTopCorners);
        lastDensity = material.arealDensity;
        PreviousPositions = new Vector3[Particles.Length];
        for(int i=0;i<Particles.Length;i++) PreviousPositions[i]=Particles[i].position;
        Contact = new ClothContact(Particles.Length);
        ExternalForces = new Vector3[Particles.Length];
        membrane = new ClothMembraneConstraint[geometry.triangles.Length / 3];
        var edges = new Dictionary<long, Edge>();
        for (int t = 0; t < geometry.triangles.Length; t += 3)
        {
            membrane[t / 3] = new ClothMembraneConstraint(geometry, t);
            int a = geometry.triangles[t], b = geometry.triangles[t + 1], c = geometry.triangles[t + 2];
            areas.Add(new ClothShellConstraint(a,b,c,geometry.restPositions));
            RegisterEdge(edges, a, b, c, t);
            RegisterEdge(edges, b, c, a, t);
            RegisterEdge(edges, c, a, b, t);
        }
    }

    private void RegisterEdge(Dictionary<long, Edge> edges, int a, int b, int opposite, int face)
    {
        long key = ((long)Math.Min(a, b) << 32) | (uint)Math.Max(a, b);
        Edge first;
        if (!edges.TryGetValue(key, out first))
        {
            edges.Add(key, new Edge { a = a, b = b, opposite = opposite, face=face });
            edgesOnly.Add(new ClothShellConstraint(a,b,Geometry.restPositions));
            return;
        }
        if (first.paired || first.a != b || first.b != a)
            throw new ArgumentException("Mesh must have consistently oriented manifold triangles.");
        first.paired = true;
        Vector3 direction=(Geometry.restPositions[b]-Geometry.restPositions[a]).normalized;
        Vector3 weights=0.5f*(DirectionWeights(first.face,direction)+DirectionWeights(face,direction));
        bending.Add(new ClothBendingConstraint(first.a, first.b, first.opposite, opposite, Geometry.restPositions, weights));
    }

    private Vector3 DirectionWeights(int face, Vector3 edge)
    {
        Vector3 u,v; Geometry.Frame(face,out u,out v);
        // Across-hinge direction: a 90-degree rotation of the edge in the rest face.
        float angle=(float)System.Math.Atan2(Mathf.Abs(Vector3.Dot(edge,u)),Mathf.Abs(Vector3.Dot(edge,v)));
        float t=angle/((float)System.Math.PI*0.25f);
        return t<=1f ? new Vector3(1f-t,t,0f) : new Vector3(0f,2f-t,t-1f);
    }
    public void SetPin(int index,bool pinned,Vector3 target)
    {
        if(index<0||index>=Particles.Length||!ClothMath.IsFinite(target)) throw new ArgumentException("Invalid pin.");
        Particles[index].pinTarget=target; Particles[index].isPinned=pinned;
        if(pinned) { PreviousPositions[index]=target; Particles[index].position=Particles[index].predictedPosition=target; Particles[index].velocity=Vector3.zero; }
    }
    public void SetRestAngle(int hinge,float angle) { bending[hinge].SetRestAngle(angle); }
    public void CaptureCrease(int hinge) { bending[hinge].CaptureCrease(Particles); }
    public void ResetCreases() { foreach(var h in bending) h.ResetRestAngle(); }

    // Internal ELASTIC forces only, from the same energy used by the constraints.
    public double EvaluateElastic(ClothMaterial material,Vector3[] forces)
    {
        material.Validate();
        if(forces==null||forces.Length!=Particles.Length) throw new ArgumentException("Force array size mismatch.");
        Array.Clear(forces,0,forces.Length); double energy=0.0;
        if(material.membraneModel==ClothMembraneModel.Orthotropic)
            foreach(var c in membrane) energy+=c.AccumulateForces(Particles,material,forces);
        else foreach(var c in edgesOnly) energy+=c.AccumulateForces(Particles,material.edgeStiffness,forces);
        foreach(var c in areas) energy+=c.AccumulateForces(Particles,material.areaStiffness,forces);
        foreach(var c in bending) energy+=c.AccumulateForces(Particles,material,forces);
        return energy;
    }

    public void Reset()
    {
        EndGrab();
        foreach (ClothParticle particle in Particles) particle.Reset();
        for(int i=0;i<Particles.Length;i++) PreviousPositions[i]=Particles[i].position;
        DisplayRevision++;
    }

    // Backwards-compatible single-grab API used by desktop interaction (slot 0).
    public bool BeginGrab(int triangleIndex, Vector3 weights, Vector3 target)
    {
        return BeginGrab(0, triangleIndex, weights, target);
    }

    // Begin a grab in an independent slot. Use slot 0 for the right hand and slot 1 for the left.
    public bool BeginGrab(int grabId, int triangleIndex, Vector3 weights, Vector3 target)
    {
        if (grabId < 0 || grabId >= grabs.Length) return false;
        if (triangleIndex < 0 || triangleIndex >= membrane.Length ||
            !ClothMath.IsFinite(weights) || !ClothMath.IsFinite(target)) return false;
        if (weights.x < 0f || weights.y < 0f || weights.z < 0f ||
            Mathf.Abs(weights.x + weights.y + weights.z - 1f) > 0.001f) return false;

        int t = triangleIndex * 3;
        float movable = Particles[Geometry.triangles[t]].InverseMass * weights.x * weights.x
            + Particles[Geometry.triangles[t + 1]].InverseMass * weights.y * weights.y
            + Particles[Geometry.triangles[t + 2]].InverseMass * weights.z * weights.z;
        if (movable <= 0f) return false;

        GrabState grab = grabs[grabId];
        grab.triangleIndex = triangleIndex;
        grab.weights = weights;
        grab.target = target;
        grab.lambda = Vector3.zero;
        return true;
    }

    // Backwards-compatible movement API for desktop interaction (slot 0).
    public void MoveGrab(Vector3 target)
    {
        MoveGrab(0, target);
    }

    public void MoveGrab(int grabId, Vector3 target)
    {
        if (grabId < 0 || grabId >= grabs.Length || !ClothMath.IsFinite(target)) return;
        if (grabs[grabId].triangleIndex >= 0) grabs[grabId].target = target;
    }

    // End only one hand's grab.
    public void EndGrab(int grabId)
    {
        if (grabId < 0 || grabId >= grabs.Length) return;
        grabs[grabId].triangleIndex = -1;
        grabs[grabId].lambda = Vector3.zero;
    }

    // End all grabs. Kept for reset, pause, disable and existing desktop code.
    public void EndGrab()
    {
        EndGrab(0);
        EndGrab(1);
    }

    private void SolveGrab(float dt)
    {
        for (int grabId = 0; grabId < grabs.Length; grabId++)
        {
            GrabState grab = grabs[grabId];
            if (grab.triangleIndex < 0) continue;

            int t = grab.triangleIndex * 3;
            ClothParticle a = Particles[Geometry.triangles[t]];
            ClothParticle b = Particles[Geometry.triangles[t + 1]];
            ClothParticle c = Particles[Geometry.triangles[t + 2]];
            Vector3 w = grab.weights;
            Vector3 point = w.x * a.predictedPosition + w.y * b.predictedPosition + w.z * c.predictedPosition;
            float inverseMass = w.x * w.x * a.InverseMass + w.y * w.y * b.InverseMass + w.z * w.z * c.InverseMass;
            float alpha = Mathf.Max(0f, grabCompliance) / (dt * dt);
            if (inverseMass + alpha <= 0f) continue;

            Vector3 change = (grab.target - point - alpha * grab.lambda) / (inverseMass + alpha);
            grab.lambda += change;
            a.predictedPosition += a.InverseMass * w.x * change;
            b.predictedPosition += b.InverseMass * w.y * change;
            c.predictedPosition += c.InverseMass * w.z * change;
        }
    }

    public void Step(float timestep, ClothMaterial material)
    {
        if (timestep <= 0f || float.IsNaN(timestep) || float.IsInfinity(timestep))
            throw new ArgumentOutOfRangeException(nameof(timestep));
        if (material == null) throw new ArgumentNullException(nameof(material));
        material.Validate();
        if (!ClothMath.IsFinite(gravity) || float.IsNaN(grabCompliance) || float.IsInfinity(grabCompliance) || grabCompliance<0f)
            throw new ArgumentException("Invalid gravity/grab compliance.");
        foreach(Vector3 force in ExternalForces) if(!ClothMath.IsFinite(force)) throw new ArgumentException("Invalid external force.");

        // Live density changes update mass without resetting positions or velocities.
        if (lastDensity != material.arealDensity)
        {
            for (int i = 0; i < Particles.Length; i++)
                Particles[i].SetMass(material.arealDensity * Geometry.vertexAreas[i]);
            lastDensity=material.arealDensity;
        }
        for(int i=0;i<Particles.Length;i++) PreviousPositions[i]=Particles[i].position;
        bool orthotropic=material.membraneModel==ClothMembraneModel.Orthotropic;
        bool useArea=material.areaStiffness>0f;
        bool useBend=material.bending>0f || (material.directionalBending && (material.bending45>0f || material.bending90>0f));
        bool useDamping=material.bendDamping>0f;
        bool useContact=Contact.floorEnabled || Contact.sphereEnabled;

        int steps = Mathf.Clamp(substeps, 1, 32);
        int iterations = Mathf.Clamp(solverIterations, 1, 40);
        float dt = timestep / steps;
        float dragFactor = Mathf.Exp(-material.particleDrag * dt);
        for (int s = 0; s < steps; s++)
        {
            if(orthotropic) foreach (var constraint in membrane) constraint.BeginSubstep(material, dt);
            else foreach(var constraint in edgesOnly) constraint.BeginSubstep(material.edgeStiffness,dt);
            if(useBend || useDamping) foreach (var constraint in bending) constraint.BeginSubstep(material, dt);
            if(useArea) foreach(var constraint in areas) constraint.BeginSubstep(material.areaStiffness,dt);
            for (int grabId = 0; grabId < grabs.Length; grabId++)
                grabs[grabId].lambda = Vector3.zero;

            for (int particleIndex=0; particleIndex<Particles.Length; particleIndex++)
            {
                ClothParticle particle=Particles[particleIndex];
                if (particle.isPinned)
                {
                    particle.position = particle.pinTarget;
                    particle.predictedPosition = particle.pinTarget;
                    particle.velocity = Vector3.zero;
                    continue;
                }
                particle.velocity *= dragFactor; // Add this line
                particle.velocity += (gravity + particle.InverseMass * ExternalForces[particleIndex]) * dt;
                particle.predictedPosition = particle.position + particle.velocity * dt;
                particle.velocity += (gravity + particle.InverseMass*ExternalForces[particleIndex]) * dt;
                particle.predictedPosition = particle.position + particle.velocity * dt;
            }

            if(useContact) Contact.BeginSubstep(Particles);
            for (int iteration = 0; iteration < iterations; iteration++)
            {
                // Alternate traversal to reduce a persistent directional ordering bias.
                if(orthotropic)
                    for (int j = 0; j < membrane.Length; j++)
                        membrane[iteration % 2 == 0 ? j : membrane.Length - 1 - j].Solve(Particles);
                else foreach(var constraint in edgesOnly) constraint.Solve(Particles);
                if(useArea) foreach(var constraint in areas) constraint.Solve(Particles);
                if(useBend) for (int j = 0; j < bending.Count; j++)
                    bending[iteration % 2 == 0 ? j : bending.Count - 1 - j].Solve(Particles);
                SolveGrab(dt);
                if(useContact) Contact.Solve(Particles);
            }

            if(useContact) { Contact.ApplyFriction(Particles); Contact.Solve(Particles); }
            foreach (ClothParticle particle in Particles)
            {
                particle.velocity = (particle.predictedPosition - particle.position) / dt;
                particle.position = particle.predictedPosition;
            }
            if(useDamping) foreach (ClothBendingConstraint constraint in bending) constraint.ApplyDamping(Particles, dt);
            foreach (ClothParticle particle in Particles)
            {
                if (!ClothMath.IsFinite(particle.position) || !ClothMath.IsFinite(particle.velocity) || particle.position.sqrMagnitude > 1e8f)
                {
                    Reset();
                    throw new InvalidOperationException("Cloth became unstable. Reset, reduce stiffness or increase substeps.");
                }
            }
        }
    }
}
