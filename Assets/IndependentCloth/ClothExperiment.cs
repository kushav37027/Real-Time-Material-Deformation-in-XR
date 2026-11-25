using System;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

// Optional component for controlled loads and recording. Values are world-space SI units.
[RequireComponent(typeof(ClothSimulation))]
public class ClothExperiment : MonoBehaviour
{
    public int[] loadedVertices = new int[0];
    public Vector3 forcePerVertex = Vector3.zero;
    private ClothSimulation simulation;
    private ClothSolver appliedSolver;
    private int[] previous = new int[0];
    private Vector3 previousForce;
    private void Awake() { simulation=GetComponent<ClothSimulation>(); }
    private void Update()
    {
        if(simulation==null||simulation.Solver==null) return;
        RemoveLoads();
        appliedSolver=simulation.Solver;
        if(!ClothMath.IsFinite(forcePerVertex)) return;
        previous=loadedVertices==null?new int[0]:(int[])loadedVertices.Clone();
        previousForce=forcePerVertex;
        foreach(int i in previous)
            if(i>=0&&i<appliedSolver.Particles.Length) appliedSolver.ExternalForces[i]+=previousForce;
    }
    private void RemoveLoads()
    {
        if(appliedSolver!=null) foreach(int i in previous)
            if(i>=0&&i<appliedSolver.Particles.Length) appliedSolver.ExternalForces[i]-=previousForce;
        previous=new int[0]; appliedSolver=null;
    }
    private void OnDisable() { RemoveLoads(); }

    // Use these methods from a test script to reproduce measured grip boundary conditions.
    public void SetGrip(int vertex, Vector3 target)
    { if(simulation.Solver!=null) simulation.Solver.SetPin(vertex,true,target); }
    public void ReleaseGrip(int vertex)
    { if(simulation.Solver!=null) simulation.Solver.SetPin(vertex,false,simulation.Solver.Particles[vertex].position); }

    public double PositionRMSE(Vector3[] measured)
    {
        var solver=simulation.Solver;
        if(solver==null||measured==null||measured.Length!=solver.Particles.Length) throw new ArgumentException("Measurements need matching vertex correspondence.");
        double error=0;
        for(int i=0;i<measured.Length;i++)
        {
            if(!ClothMath.IsFinite(measured[i])) throw new ArgumentException("Nonfinite measurement.");
            error+=(solver.Particles[i].position-measured[i]).sqrMagnitude;
        }
        return Math.Sqrt(error/measured.Length);
    }

    [ContextMenu("Export Current Measurement CSV")]
    public void ExportSnapshot()
    {
        if(simulation==null) simulation=GetComponent<ClothSimulation>();
        var solver=simulation.Solver;
        if(solver==null) { Debug.LogWarning("Enter Play Mode before exporting.",this); return; }
        var elastic=new Vector3[solver.Particles.Length];
        double energy=solver.EvaluateElastic(simulation.material,elastic);
        var csv=new StringBuilder("vertex,x_m,y_m,z_m,speed_m_s,load_x_N,load_y_N,load_z_N,elastic_x_N,elastic_y_N,elastic_z_N,pinned,contact_free_support_x_N,contact_free_support_y_N,contact_free_support_z_N\n");
        double residual=0;
        for(int i=0;i<solver.Particles.Length;i++)
        {
            var p=solver.Particles[i]; Vector3 load=solver.ExternalForces[i];
            Vector3 net=elastic[i]+load+p.Mass*solver.gravity;
            Vector3 support=p.isPinned?-net:Vector3.zero;
            if(!p.isPinned) residual+=net.sqrMagnitude;
            csv.Append(i).Append(','); Append(csv,p.position); csv.Append(',').Append(p.velocity.magnitude.ToString("R",CultureInfo.InvariantCulture)).Append(',');
            Append(csv,load);csv.Append(',');Append(csv,elastic[i]);csv.Append(',').Append(p.isPinned?1:0).Append(',');Append(csv,support);csv.Append('\n');
        }
        string path=Path.Combine(Application.persistentDataPath,"cloth_"+DateTime.UtcNow.ToString("yyyyMMdd_HHmmss_fff")+".csv");
        File.WriteAllText(path,csv.ToString());
        Debug.Log("Saved "+path+"; elastic energy="+energy+" J; contact-free free-vertex residual norm="+Math.Sqrt(residual)+" N. Support estimates exclude contact, friction and grabbing.",this);
    }
    private static void Append(StringBuilder s,Vector3 v)
    {
        s.Append(v.x.ToString("R",CultureInfo.InvariantCulture)).Append(',')
         .Append(v.y.ToString("R",CultureInfo.InvariantCulture)).Append(',')
         .Append(v.z.ToString("R",CultureInfo.InvariantCulture));
    }
}
