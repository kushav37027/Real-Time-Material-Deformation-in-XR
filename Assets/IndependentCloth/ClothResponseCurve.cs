using System;

// Nonnegative secant-stiffness knots; even response in tension/compression.
[Serializable]
public class ClothResponseCurve
{
    public float[] strain = new float[] { 0f, 0.1f, 0.3f };
    public float[] stiffness = new float[] { 0f, 0f, 0f };

    public void Validate()
    {
        if (strain == null || stiffness == null || strain.Length < 2 || strain.Length != stiffness.Length || strain[0] != 0f)
            throw new ArgumentException("Curve needs matching arrays, at least two knots, starting at zero.");
        for (int i = 0; i < strain.Length; i++)
            if (!Finite(strain[i]) || !Finite(stiffness[i]) || stiffness[i] < 0f || (i > 0 && strain[i] <= strain[i-1]))
                throw new ArgumentException("Curve knots must increase; stiffness must be finite and nonnegative.");
    }
    private static bool Finite(float x) { return !float.IsNaN(x) && !float.IsInfinity(x); }

    // Returns Phi(e) and dPhi/de. Exact integral of the Hermite secant curve times strain.
    public void Evaluate(float e, out double energy, out double stress)
    {
        double x = Math.Abs(e), integral = 0.0, k = stiffness[stiffness.Length-1];
        for (int i = 0; i < strain.Length-1; i++)
        {
            double a = strain[i], d = strain[i+1]-a;
            double z = Math.Min(1.0, Math.Max(0.0, (x-a)/d));
            double z2=z*z, z3=z2*z, z4=z3*z, z5=z4*z;
            double k0=stiffness[i], dk=stiffness[i+1]-k0;
            integral += k0*(a*d*z+d*d*z2/2.0)
                + dk*(a*d*(z3-z4/2.0)+d*d*(3.0*z4/4.0-2.0*z5/5.0));
            if (x <= strain[i+1])
            {
                k = k0+dk*(3.0*z2-2.0*z3);
                energy = Math.Max(0.0, integral); stress = k*e; return;
            }
        }
        double last = strain[strain.Length-1];
        integral += k*(x*x-last*last)/2.0;
        energy = Math.Max(0.0, integral); stress=k*e;
    }

    // C^2/2 = scale * [baseStiffness*e^2/2 + Phi(e)].
    public void Constraint(float e, double scale, double baseStiffness, out float value, out float derivative)
    {
        double energy, stress;
        Evaluate(e, out energy, out stress);
        energy = scale*(energy+0.5*baseStiffness*e*e);
        stress = scale*(stress+baseStiffness*e);
        if (Math.Abs(e) < 1e-8)
        {
            derivative=(float)Math.Sqrt(scale*(baseStiffness+stiffness[0]));
            value=derivative*e;
        }
        else if (energy <= 0.0) { value=derivative=0f; }
        else
        {
            double c=Math.Sign(e)*Math.Sqrt(2.0*energy);
            value=(float)c; derivative=(float)(stress/c);
        }
    }
}
