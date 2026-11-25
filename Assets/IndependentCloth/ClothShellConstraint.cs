using UnityEngine;

// Edge or area energy, using the original shell paper's coefficient convention.
public class ClothShellConstraint
{
    private readonly int[] indices;
    private readonly Vector3[] gradients;
    private readonly bool isArea;
    private readonly float measure;
    private float lambda, alpha, stiffness;

    public ClothShellConstraint(int a,int b,Vector3[] rest)
    {
        indices=new int[]{a,b}; gradients=new Vector3[2];
        measure=(rest[b]-rest[a]).magnitude;
        if(measure<=1e-8f) throw new System.ArgumentException("Degenerate edge.");
    }
    public ClothShellConstraint(int a,int b,int c,Vector3[] rest)
    {
        indices=new int[]{a,b,c}; gradients=new Vector3[3]; isArea=true;
        measure=0.5f*Vector3.Cross(rest[b]-rest[a],rest[c]-rest[a]).magnitude;
        if(measure<=1e-12f) throw new System.ArgumentException("Degenerate area.");
    }
    public void BeginSubstep(float k,float h)
    { stiffness=k; lambda=0f; alpha=k>0f ? 1f/(2f*k*measure*h*h) : 0f; }
    private float Evaluate(ClothParticle[] particles,bool predicted)
    {
        Vector3 a=predicted?particles[indices[0]].predictedPosition:particles[indices[0]].position;
        Vector3 b=predicted?particles[indices[1]].predictedPosition:particles[indices[1]].position;
        if(!isArea)
        {
            Vector3 edge=b-a; float length=edge.magnitude;
            gradients[1]=length>1e-10f?edge/(length*measure):Vector3.zero;
            gradients[0]=-gradients[1]; return length/measure-1f;
        }
        Vector3 c=predicted?particles[indices[2]].predictedPosition:particles[indices[2]].position;
        Vector3 cross=Vector3.Cross(b-a,c-a); float twiceArea=cross.magnitude;
        Vector3 n=twiceArea>1e-12f?cross/twiceArea:Vector3.zero;
        gradients[0]=0.5f/measure*Vector3.Cross(b-c,n);
        gradients[1]=0.5f/measure*Vector3.Cross(c-a,n);
        gradients[2]=0.5f/measure*Vector3.Cross(a-b,n);
        return twiceArea/(2f*measure)-1f;
    }
    public void Solve(ClothParticle[] particles)
    {
        if(stiffness<=0f) return;
        float value=Evaluate(particles,true), denominator=alpha;
        for(int i=0;i<indices.Length;i++) denominator+=particles[indices[i]].InverseMass*gradients[i].sqrMagnitude;
        float change=(-value-alpha*lambda)/denominator; lambda+=change;
        for(int i=0;i<indices.Length;i++) particles[indices[i]].predictedPosition+=particles[indices[i]].InverseMass*change*gradients[i];
    }
    public double AccumulateForces(ClothParticle[] particles,float k,Vector3[] forces)
    {
        float value=Evaluate(particles,false);
        for(int i=0;i<indices.Length;i++) forces[indices[i]]-=2f*k*measure*value*gradients[i];
        return k*measure*value*value;
    }
}
