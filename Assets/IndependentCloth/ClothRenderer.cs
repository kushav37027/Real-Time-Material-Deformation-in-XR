using UnityEngine;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class ClothRenderer : MonoBehaviour
{
    public Material surfaceMaterial;
    private Mesh mesh;
    private Vector3[] vertices;
    private ClothParticle[] particles;
    private ClothSimulation simulation;
    public double LastMeshMilliseconds { get; private set; }

    public void Initialize(ClothGeometry geometry, ClothParticle[] state)
    {
        if (geometry == null || state == null || state.Length != geometry.restPositions.Length)
            throw new System.ArgumentException("Geometry and particle counts must match.");
        particles = state;
        simulation = GetComponent<ClothSimulation>();
        if (mesh != null) Destroy(mesh);
        mesh = new Mesh { name = "Independent Cloth" };
        mesh.MarkDynamic();
        int n = particles.Length;
        vertices = new Vector3[2*n];
        if (vertices.Length > 65535) mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        int[] source = geometry.triangles;
        int[] indices = new int[2*source.Length];
        Vector2[] uv = new Vector2[2*n];
        for (int i=0; i<n; i++) uv[i] = uv[i+n] = geometry.materialCoordinates[i];
        for (int t=0; t<source.Length; t+=3)
        {
            indices[t]=source[t]; indices[t+1]=source[t+1]; indices[t+2]=source[t+2];
            int back=t+source.Length;
            indices[back]=source[t]+n; indices[back+1]=source[t+2]+n; indices[back+2]=source[t+1]+n;
        }
        CopyPositions(); mesh.vertices=vertices; mesh.triangles=indices; mesh.uv=uv;
        mesh.RecalculateNormals(); mesh.RecalculateBounds();
        GetComponent<MeshFilter>().sharedMesh=mesh;
        if(surfaceMaterial!=null) GetComponent<MeshRenderer>().sharedMaterial=surfaceMaterial;
    }
    private bool CopyPositions()
    {
        bool changed=false;
        ClothSolver solver=simulation!=null ? simulation.Solver : null;
        bool smooth=simulation!=null && simulation.smoothDisplay && !simulation.paused &&
            solver!=null && object.ReferenceEquals(solver.Particles,particles) && simulation.HasStepped;
        float blend=smooth ? Mathf.Clamp((Time.time-Time.fixedTime)/Time.fixedDeltaTime,0f,1f) : 1f;
        for(int i=0;i<particles.Length;i++)
        {
            Vector3 position=particles[i].position;
            if(smooth) position=(1f-blend)*solver.PreviousPositions[i]+blend*position;
            Vector3 local=transform.InverseTransformPoint(position);
            if((vertices[i]-local).sqrMagnitude>0f) changed=true;
            vertices[i]=vertices[i+particles.Length]=local;
        }
        return changed;
    }
    private void LateUpdate()
    {
        if(mesh==null||particles==null) return;
        long start=System.Diagnostics.Stopwatch.GetTimestamp();
        if(CopyPositions()) { mesh.vertices=vertices; mesh.RecalculateNormals(); mesh.RecalculateBounds(); }
        LastMeshMilliseconds=(System.Diagnostics.Stopwatch.GetTimestamp()-start)*1000.0/System.Diagnostics.Stopwatch.Frequency;
    }
    private void OnDestroy() { if(mesh!=null) Destroy(mesh); }
}
