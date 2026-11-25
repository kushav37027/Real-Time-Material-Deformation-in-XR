using System;

// Small offline fitting helper. The caller supplies measured-data objective and equilibrium solves.
// Run outside the per-frame update. This is coordinate search, not the paper's optimizer.
public static class ClothCalibration
{
    public static double[] FitPositive(double[] initial, double[] lower, double[] upper,
        Func<double[], double> objective, int sweeps = 20, double logStep = 0.5)
    {
        if(initial==null||lower==null||upper==null||objective==null||initial.Length==0
            ||initial.Length!=lower.Length||initial.Length!=upper.Length||sweeps<1||!Finite(logStep)||logStep<=0)
            throw new ArgumentException("Invalid calibration inputs.");
        double[] p=(double[])initial.Clone();
        for(int i=0;i<p.Length;i++)
            if(!Finite(lower[i])||!Finite(upper[i])||!Finite(p[i])||lower[i]<=0||upper[i]<lower[i]||p[i]<lower[i]||p[i]>upper[i])
                throw new ArgumentException("Calibration requires bounded positive parameters.");
        double best=Score(objective,p);
        for(int sweep=0;sweep<sweeps;sweep++)
        {
            bool improved=false;
            for(int i=0;i<p.Length;i++)
            {
                double original=p[i], selected=original;
                for(int sign=-1;sign<=1;sign+=2)
                {
                    p[i]=Math.Max(lower[i],Math.Min(upper[i],original*Math.Exp(sign*logStep)));
                    double score=Score(objective,p);
                    if(score<best) { best=score;selected=p[i];improved=true; }
                }
                p[i]=selected;
            }
            if(!improved) logStep*=0.5;
        }
        // Restore caller-owned model state to the winning parameters through its callback.
        Score(objective,p);
        return p;
    }
    private static bool Finite(double x) { return !double.IsNaN(x)&&!double.IsInfinity(x); }
    private static double Score(Func<double[],double> objective,double[] p)
    {
        double score=objective((double[])p.Clone());
        if(!Finite(score)||score<0) throw new ArgumentException("Objective must return finite nonnegative error; reject unconverged samples explicitly.");
        return score;
    }
}
