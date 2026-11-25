using UnityEngine;

// One coupled stretch-U / stretch-V / engineering-shear block per triangle.
public class ClothMembraneConstraint
{
    private readonly int[] indices;
    private readonly float[] p = new float[3];
    private readonly float[] q = new float[3];
    private readonly Vector3[] gu = new Vector3[3];
    private readonly Vector3[] gv = new Vector3[3];
    private readonly Vector3[] gs = new Vector3[3];
    private readonly float area;
    private float alphaU, alphaV, alphaCoupling, alphaShear;
    private Vector3 lambda;
    private ClothMaterial material;
    private float timestep;
    private float cachedU = -1f, cachedV, cachedShear, cachedCoupling, cachedDt;
    private readonly float[] nonlinearLambda = new float[3];

    public ClothMembraneConstraint(ClothGeometry geometry, int triangleOffset)
    {
        indices = new int[] {
            geometry.triangles[triangleOffset],
            geometry.triangles[triangleOffset + 1],
            geometry.triangles[triangleOffset + 2]
        };
        Vector3 axisU, axisV;
        geometry.Frame(triangleOffset, out axisU, out axisV);
        Vector3 edge1 = geometry.restPositions[indices[1]] - geometry.restPositions[indices[0]];
        Vector3 edge2 = geometry.restPositions[indices[2]] - geometry.restPositions[indices[0]];
        Vector2 e1 = new Vector2(Vector3.Dot(edge1,axisU), Vector3.Dot(edge1,axisV));
        Vector2 e2 = new Vector2(Vector3.Dot(edge2,axisU), Vector3.Dot(edge2,axisV));
        float determinant = e1.x * e2.y - e2.x * e1.y;
        if (Mathf.Abs(determinant) < 1e-12f)
            throw new System.ArgumentException("Degenerate material-coordinate triangle.");

        // Columns of Dm^-1 supply coefficients for the deformation-gradient columns.
        p[1] = e2.y / determinant;
        p[2] = -e1.y / determinant;
        p[0] = -p[1] - p[2];
        q[1] = -e2.x / determinant;
        q[2] = e1.x / determinant;
        q[0] = -q[1] - q[2];

        Vector3 r1 = geometry.restPositions[indices[1]] - geometry.restPositions[indices[0]];
        Vector3 r2 = geometry.restPositions[indices[2]] - geometry.restPositions[indices[0]];
        area = 0.5f * Vector3.Cross(r1, r2).magnitude;
        if (area < 1e-12f) throw new System.ArgumentException("Degenerate reference triangle.");
    }

    // Called once per substep, never once per solver iteration.
    public void BeginSubstep(ClothMaterial material, float dt)
    {
        this.material = material;
        timestep = dt;
        System.Array.Clear(nonlinearLambda, 0, 3);
        lambda = Vector3.zero;
        if (cachedU == material.stretchU && cachedV == material.stretchV &&
            cachedShear == material.shear && cachedCoupling == material.coupling && cachedDt == dt) return;
        cachedU = material.stretchU; cachedV = material.stretchV;
        cachedShear = material.shear; cachedCoupling = material.coupling; cachedDt = dt;
        double coupling = material.CoupledStretch;
        double determinant = (double)material.stretchU * material.stretchV - coupling * coupling;
        if (determinant <= 0) throw new System.ArgumentException("Invalid material stiffness matrix.");
        double scale = 1.0 / ((double)area * dt * dt);
        // alphaTilde = inverse(area * H) / dt^2.
        alphaU = (float)(scale * material.stretchV / determinant);
        alphaV = (float)(scale * material.stretchU / determinant);
        alphaCoupling = (float)(-scale * coupling / determinant);
        alphaShear = (float)(scale / material.shear);
    }

    public void Solve(ClothParticle[] particles)
    {
        Vector3 x0 = particles[indices[0]].predictedPosition;
        Vector3 e1 = particles[indices[1]].predictedPosition - x0;
        Vector3 e2 = particles[indices[2]].predictedPosition - x0;
        Vector3 u = p[1] * e1 + p[2] * e2;
        Vector3 v = q[1] * e1 + q[2] * e2;
        Vector3 strain = new Vector3(
            (u.sqrMagnitude - 1f) * 0.5f,
            (v.sqrMagnitude - 1f) * 0.5f,
            Vector3.Dot(u, v));

        double a = alphaU, b = alphaCoupling, c = 0;
        double d = alphaV, e = 0, f = alphaShear;
        for (int i = 0; i < 3; i++)
        {
            gu[i] = p[i] * u;
            gv[i] = q[i] * v;
            gs[i] = p[i] * v + q[i] * u;
            float w = particles[indices[i]].InverseMass;
            a += w * Vector3.Dot(gu[i], gu[i]);
            b += w * Vector3.Dot(gu[i], gv[i]);
            c += w * Vector3.Dot(gu[i], gs[i]);
            d += w * Vector3.Dot(gv[i], gv[i]);
            e += w * Vector3.Dot(gv[i], gs[i]);
            f += w * Vector3.Dot(gs[i], gs[i]);
        }

        Vector3 alphaLambda = new Vector3(
            alphaU * lambda.x + alphaCoupling * lambda.y,
            alphaCoupling * lambda.x + alphaV * lambda.y,
            alphaShear * lambda.z);
        Vector3 change = ClothMath.SolveSymmetric(a, b, c, d, e, f, -strain - alphaLambda);
        lambda += change;

        for (int i = 0; i < 3; i++)
        {
            ClothParticle particle = particles[indices[i]];
            particle.predictedPosition += particle.InverseMass *
                (gu[i] * change.x + gv[i] * change.y + gs[i] * change.z);
        }
        if (material.nonlinearMembrane)
            for (int mode=0; mode<3; mode++) SolveNonlinear(particles, mode);
    }

    private Vector3 Strain(ClothParticle[] particles, bool predicted)
    {
        Vector3 a = predicted ? particles[indices[0]].predictedPosition : particles[indices[0]].position;
        Vector3 b = predicted ? particles[indices[1]].predictedPosition : particles[indices[1]].position;
        Vector3 c = predicted ? particles[indices[2]].predictedPosition : particles[indices[2]].position;
        Vector3 u=p[1]*(b-a)+p[2]*(c-a), v=q[1]*(b-a)+q[2]*(c-a);
        for(int i=0;i<3;i++) { gu[i]=p[i]*u; gv[i]=q[i]*v; gs[i]=p[i]*v+q[i]*u; }
        return new Vector3((u.sqrMagnitude-1f)*0.5f,(v.sqrMagnitude-1f)*0.5f,Vector3.Dot(u,v));
    }
    private ClothResponseCurve Curve(ClothMaterial m,int mode)
    { return mode==0 ? m.extraStretchU : mode==1 ? m.extraStretchV : m.extraShear; }
    private void SolveNonlinear(ClothParticle[] particles,int mode)
    {
        Vector3 e=Strain(particles,true);
        float strain=mode==0 ? e.x : mode==1 ? e.y : e.z;
        float value, derivative;
        Curve(material,mode).Constraint(strain,area,0.0,out value,out derivative);
        if(derivative==0f) return;
        Vector3[] g=mode==0 ? gu : mode==1 ? gv : gs;
        float alpha=1f/(timestep*timestep), denominator=alpha;
        for(int i=0;i<3;i++) denominator+=particles[indices[i]].InverseMass*derivative*derivative*g[i].sqrMagnitude;
        float change=(-value-alpha*nonlinearLambda[mode])/denominator;
        nonlinearLambda[mode]+=change;
        for(int i=0;i<3;i++) particles[indices[i]].predictedPosition+=particles[indices[i]].InverseMass*change*derivative*g[i];
    }
    public double AccumulateForces(ClothParticle[] particles,ClothMaterial m,Vector3[] forces)
    {
        Vector3 e=Strain(particles,false);
        Vector3 stress=new Vector3(m.stretchU*e.x+m.CoupledStretch*e.y,m.CoupledStretch*e.x+m.stretchV*e.y,m.shear*e.z);
        double energy=0.5*area*Vector3.Dot(e,stress);
        if(m.nonlinearMembrane)
        {
            double u,s;
            m.extraStretchU.Evaluate(e.x,out u,out s); energy+=area*u; stress.x+=(float)s;
            m.extraStretchV.Evaluate(e.y,out u,out s); energy+=area*u; stress.y+=(float)s;
            m.extraShear.Evaluate(e.z,out u,out s); energy+=area*u; stress.z+=(float)s;
        }
        for(int i=0;i<3;i++) forces[indices[i]]-=area*(stress.x*gu[i]+stress.y*gv[i]+stress.z*gs[i]);
        return energy;
    }
}
