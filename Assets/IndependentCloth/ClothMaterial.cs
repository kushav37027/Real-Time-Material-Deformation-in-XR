using System;
using UnityEngine;

public enum ClothMembraneModel { Orthotropic, EdgeAreaShell }

[Serializable]
public class ClothMaterial
{
    public ClothMembraneModel membraneModel = ClothMembraneModel.Orthotropic;
    public float stretchU = 180f, stretchV = 90f, shear = 35f;
    [Range(-0.99f, 0.99f)] public float coupling = 0.2f;
    public bool nonlinearMembrane;
    public ClothResponseCurve extraStretchU = new ClothResponseCurve();
    public ClothResponseCurve extraStretchV = new ClothResponseCurve();
    public ClothResponseCurve extraShear = new ClothResponseCurve();
    // Paper convention: energy = k_L*l0*C_L^2 + k_A*A0*C_A^2.
    public float edgeStiffness = 5f, areaStiffness = 0f;
    // D = 2*k_B in the shell paper's convention. Direction is across the hinge.
    public float bending = 0.0002f;
    public bool directionalBending;
    public float bending45 = 0.0002f, bending90 = 0.0002f;
    public bool nonlinearBending;
    public ClothResponseCurve extraBending = new ClothResponseCurve
    { strain = new float[] { 0f, 5f, 20f }, stiffness = new float[] { 0f, 0f, 0f } };
    [Min(0f)] public float particleDrag = 1f;
    public float bendDamping = 0.00002f;
    public float arealDensity = 0.2f;
    public float CoupledStretch { get { return coupling * Mathf.Sqrt(stretchU*stretchV); } }

    public void Validate()
    {
        Positive(stretchU); Positive(stretchV); Positive(shear); Positive(arealDensity);
        Nonnegative(particleDrag); Nonnegative(edgeStiffness); Nonnegative(areaStiffness); Nonnegative(bending);
        Nonnegative(bending45); Nonnegative(bending90); Nonnegative(bendDamping);
        if (float.IsNaN(coupling) || Mathf.Abs(coupling) >= 1f) throw new ArgumentException("Coupling magnitude must be below 1.");
        if (extraStretchU == null || extraStretchV == null || extraShear == null || extraBending == null)
            throw new ArgumentException("Response curves cannot be null.");
        extraStretchU.Validate(); extraStretchV.Validate(); extraShear.Validate(); extraBending.Validate();
    }
    private static void Positive(float x) { Nonnegative(x); if(x==0f) throw new ArgumentException("Positive material value required."); }
    private static void Nonnegative(float x)
    { if(float.IsNaN(x)||float.IsInfinity(x)||x<0f) throw new ArgumentException("Finite nonnegative material value required."); }
}
