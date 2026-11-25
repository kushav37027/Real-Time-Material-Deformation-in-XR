using UnityEngine;

public class ClothBendingConstraint
{
    private readonly int[] indices;
    private readonly Vector3[] gradients=new Vector3[4];
    private readonly float originalAngle, length, width, geometricWeight;
    private readonly Vector3 directionWeights;
    private float restAngle, lambda, timestep, damping, activeStiffness, linearScale;
    private ClothMaterial material;
    public float RestAngle { get { return restAngle; } }

    public ClothBendingConstraint(int a,int b,int c,int d,Vector3[] restPositions)
        : this(a,b,c,d,restPositions,new Vector3(1f,0f,0f)) { }
    public ClothBendingConstraint(int a,int b,int c,int d,Vector3[] restPositions,Vector3 weights)
    {
        indices=new int[]{a,b,c,d}; directionWeights=weights;
        Vector3 edge=restPositions[b]-restPositions[a]; length=edge.magnitude;
        float area1=0.5f*Vector3.Cross(edge,restPositions[c]-restPositions[a]).magnitude;
        float area2=0.5f*Vector3.Cross(restPositions[d]-restPositions[a],edge).magnitude;
        if(length<=1e-8f || area1<=1e-12f || area2<=1e-12f) throw new System.ArgumentException("Degenerate hinge.");
        width=(area1+area2)/(3f*length); geometricWeight=length/width;
        originalAngle=restAngle=ClothMath.SignedAngle(restPositions[a],restPositions[b],restPositions[c],restPositions[d]);
    }
    public void SetRestAngle(float angle)
    {
        if(float.IsNaN(angle)||float.IsInfinity(angle)) throw new System.ArgumentException("Invalid rest angle.");
        restAngle=ClothMath.WrapAngle(angle); lambda=0f;
    }
    public void ResetRestAngle() { restAngle=originalAngle; lambda=0f; }
    public void CaptureCrease(ClothParticle[] p)
    { SetRestAngle(ClothMath.SignedAngle(p[indices[0]].position,p[indices[1]].position,p[indices[2]].position,p[indices[3]].position)); }
    public void BeginSubstep(ClothMaterial m,float dt)
    {
        material=m; timestep=dt; lambda=0f; damping=m.bendDamping;
        activeStiffness=Stiffness(m);
        linearScale=(float)System.Math.Sqrt((double)(length*width*activeStiffness));
    }
    private float Stiffness(ClothMaterial m)
    { return m.directionalBending ? directionWeights.x*m.bending+directionWeights.y*m.bending45+directionWeights.z*m.bending90 : m.bending; }
    private float Evaluate(ClothParticle[] p,bool predicted)
    {
        Vector3 a=predicted?p[indices[0]].predictedPosition:p[indices[0]].position;
        Vector3 b=predicted?p[indices[1]].predictedPosition:p[indices[1]].position;
        Vector3 c=predicted?p[indices[2]].predictedPosition:p[indices[2]].position;
        Vector3 d=predicted?p[indices[3]].predictedPosition:p[indices[3]].position;
        if(!ClothMath.AngleGradients(a,b,c,d,gradients))
        { System.Array.Clear(gradients,0,4); return 0f; }
        return ClothMath.WrapAngle(ClothMath.SignedAngle(a,b,c,d)-restAngle)/width;
    }
    public void Solve(ClothParticle[] p)
    {
        float stiffness=activeStiffness; if(stiffness<=0f) return;
        float q=Evaluate(p,true), value, derivative;
        if (material.nonlinearBending)
            material.extraBending.Constraint(q,length*width*stiffness,1.0,out value,out derivative);
        else { value=linearScale*q; derivative=linearScale; }
        derivative/=width;
        float alpha=1f/(timestep*timestep), denominator=alpha;
        for(int i=0;i<4;i++) denominator+=p[indices[i]].InverseMass*derivative*derivative*gradients[i].sqrMagnitude;
        float change=(-value-alpha*lambda)/denominator; lambda+=change;
        for(int i=0;i<4;i++) p[indices[i]].predictedPosition+=p[indices[i]].InverseMass*change*derivative*gradients[i];
    }
    public double AccumulateForces(ClothParticle[] p,ClothMaterial m,Vector3[] forces)
    {
        float q=Evaluate(p,false), stiffness=Stiffness(m);
        double energy=0.0, stress=0.0;
        if(m.nonlinearBending) m.extraBending.Evaluate(q,out energy,out stress);
        float torque=(float)(length*stiffness*(q+stress));
        for(int i=0;i<4;i++) forces[indices[i]]-=torque*gradients[i];
        return length*width*stiffness*(0.5*q*q+energy);
    }
    // Implicit local velocity damping: damps changing bend angle, not rigid translation.
    public void ApplyDamping(ClothParticle[] particles, float dt)
    {
        if (damping <= 0f) return;
        if (!ClothMath.AngleGradients(
            particles[indices[0]].position, particles[indices[1]].position,
            particles[indices[2]].position, particles[indices[3]].position, gradients)) return;
        float rate = 0f, effectiveMass = 0f;
        for (int i = 0; i < 4; i++)
        {
            rate += Vector3.Dot(gradients[i], particles[indices[i]].velocity);
            effectiveMass += particles[indices[i]].InverseMass * gradients[i].sqrMagnitude;
        }
        float amount = dt * damping * geometricWeight;
        float impulse = -amount * rate / (1f + amount * effectiveMass);
        for (int i = 0; i < 4; i++)
            particles[indices[i]].velocity += particles[indices[i]].InverseMass * impulse * gradients[i];
    }
}
