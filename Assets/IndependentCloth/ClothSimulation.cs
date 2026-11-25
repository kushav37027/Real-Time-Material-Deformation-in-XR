using UnityEngine;

// Attach this component to ONE empty GameObject. Other data/solver classes are not components.
[RequireComponent(typeof(ClothRenderer))]
public class ClothSimulation : MonoBehaviour
{
    [Header("Geometry - press Rebuild after changes")]
    [Range(2, 64)] public int rows = 12;
    [Range(2, 64)] public int columns = 12;
    [Min(0.05f)] public float width = 0.6f;
    [Min(0.05f)] public float height = 0.6f;
    public Vector3 origin = new Vector3(-0.3f, 1f, 0f);
    public bool pinTopCorners = true;
    [Tooltip("Optional welded, consistently wound triangle mesh. UVs define fabric orientation. Read/Write must be enabled.")]
    public Mesh restMesh;
    public int[] pinnedVertices = new int[0];

    [Header("Physical material - live")]
    public ClothMaterial material = new ClothMaterial();

    [Header("Solver")]
    [Range(1, 32)] public int substeps = 6;
    [Range(1, 40)] public int solverIterations = 10;
    public Vector3 gravity = new Vector3(0f, -9.81f, 0f);
    public bool paused;
    [Tooltip("Display-only interpolation. Adds one fixed tick of visual latency; does not speed up physics.")]
    public bool smoothDisplay = false;
    [Min(0f)] public float grabCompliance = 0.00002f;
    public bool HasStepped { get; private set; }
    public double AverageStepMilliseconds { get; private set; }
    public double PeakStepMilliseconds { get; private set; }
    public double AverageFrameMilliseconds { get; private set; }
    public double PeakFrameMilliseconds { get; private set; }
    public int PhysicsStepsPerWindow { get; private set; }
    private double sampleSeconds, stepTotal, stepPeak, framePeak;
    private int sampleFrames, sampleSteps;
    public float MaximumSpeed => displayedMaxSpeed;
    public double MeshMilliseconds => clothRenderer != null ? clothRenderer.LastMeshMilliseconds : 0;
    public string Status => status;
    private float displayedMaxSpeed;
    private ClothRenderer clothRenderer;

    private void Update()
    {
        double dt=Time.unscaledDeltaTime;
        sampleSeconds+=dt; sampleFrames++; framePeak=System.Math.Max(framePeak,dt*1000.0);
        if(sampleSeconds<0.5) return;
        float maxSquared=0f;
        if(Solver!=null) foreach(var particle in Solver.Particles) maxSquared=Mathf.Max(maxSquared,particle.velocity.sqrMagnitude);
        displayedMaxSpeed=Mathf.Sqrt(maxSquared);
        AverageFrameMilliseconds=sampleSeconds*1000.0/sampleFrames;
        PeakFrameMilliseconds=framePeak;
        AverageStepMilliseconds=sampleSteps>0 ? stepTotal/sampleSteps : 0.0;
        PeakStepMilliseconds=stepPeak; PhysicsStepsPerWindow=sampleSteps;
        sampleSeconds=stepTotal=stepPeak=framePeak=0.0; sampleFrames=sampleSteps=0;
    }

    [Header("Static contact")]
    public bool floorEnabled = true;
    public float floorHeight = 0f;
    [Min(0f)] public float thickness = 0.005f;
    [Min(0f)] public float staticFriction = 0.5f;
    [Min(0f)] public float slidingFriction = 0.3f;
    public SphereCollider sphereObstacle;

    [Header("Desktop controls")]
    public bool showControls = true;
    public ClothSolver Solver { get; private set; }
    public float PanelWidth { get { return Mathf.Min(310f, Screen.width * 0.4f); } }
    private Vector2 scroll;
    private string status = "Left-drag cloth. Scroll while grabbing to move it in depth.";

    private void Start()
    {
        Rebuild();
    }

    [ContextMenu("Rebuild Cloth")]
    public void Rebuild()
    {
        // Avoid creating runtime meshes accidentally in Edit Mode.
        if (!Application.isPlaying) return;
        try
        {
            if (material == null) material = new ClothMaterial();
            rows = Mathf.Clamp(rows, 2, 64);
            columns = Mathf.Clamp(columns, 2, 64);
            if (!ClothMath.IsFinite(origin)) throw new System.ArgumentException("Origin must be finite.");
            ClothGeometry geometry;
            if (restMesh == null) geometry = new ClothGeometry(rows, columns, width, height, origin);
            else
            {
                Vector3[] vertices = restMesh.vertices;
                for (int i=0;i<vertices.Length;i++) vertices[i]=transform.TransformPoint(vertices[i])+origin;
                geometry = new ClothGeometry(vertices, restMesh.triangles, restMesh.uv);
            }
            ClothSolver next = new ClothSolver(geometry, material, pinTopCorners);
            if(pinnedVertices!=null) foreach(int index in pinnedVertices)
            {
                if(index<0||index>=next.Particles.Length) throw new System.ArgumentException("Pinned vertex index outside mesh.");
                next.SetPin(index,true,next.Particles[index].restPosition);
            }
            GetComponent<ClothRenderer>().Initialize(geometry, next.Particles);
            if (Solver != null) Solver.EndGrab();
            Solver = next;
            clothRenderer=GetComponent<ClothRenderer>();
            HasStepped=false;
            paused = false;
            status = "Geometry rebuilt. Material settings retained.";
        }
        catch (System.Exception exception)
        {
            paused = true;
            status = exception.Message;
            Debug.LogException(exception, this);
        }
    }

    public void ResetCloth()
    {
        if (Solver == null) return;
        Solver.ResetCreases();
        Solver.Reset();
        HasStepped=false;
        paused = false;
        status = "Cloth reset. Material settings retained.";
    }

    private void FixedUpdate()
    {
        if (Solver == null || paused) return;
        try
        {
            Solver.substeps = substeps;
            Solver.solverIterations = solverIterations;
            Solver.gravity = gravity;
            Solver.grabCompliance=grabCompliance;
            ClothContact contact = Solver.Contact;
            contact.floorEnabled = floorEnabled;
            contact.floorHeight = floorHeight;
            contact.thickness = Mathf.Max(0f, thickness);
            contact.staticFriction = staticFriction;
            contact.slidingFriction = slidingFriction;
            contact.sphereEnabled = sphereObstacle != null && sphereObstacle.enabled && sphereObstacle.gameObject.activeInHierarchy;
            if (contact.sphereEnabled)
            {
                contact.sphereCenter = sphereObstacle.transform.TransformPoint(sphereObstacle.center);
                Vector3 scale = sphereObstacle.transform.lossyScale;
                float largestScale = Mathf.Max(Mathf.Abs(scale.x), Mathf.Max(Mathf.Abs(scale.y), Mathf.Abs(scale.z)));
                contact.sphereRadius = sphereObstacle.radius * largestScale;
            }
            long start=System.Diagnostics.Stopwatch.GetTimestamp();
            Solver.Step(Time.fixedDeltaTime, material);
            double elapsed=(System.Diagnostics.Stopwatch.GetTimestamp()-start)*1000.0/System.Diagnostics.Stopwatch.Frequency;
            stepTotal+=elapsed; stepPeak=System.Math.Max(stepPeak,elapsed); sampleSteps++;
            HasStepped=true;
        }
        catch (System.Exception exception)
        {
            paused = true;
            Solver.EndGrab();
            status = exception.Message;
            Debug.LogException(exception, this);
        }
    }

    public bool PointerOverControls(Vector2 screenPosition)
    {
        return showControls && screenPosition.x <= PanelWidth + 20f;
    }

    private float LogSlider(string label, float value, float minimum, float maximum)
    {
        GUILayout.Label(label + ": " + value.ToString("G3"));
        float exponent = GUILayout.HorizontalSlider(
            Mathf.Log10(Mathf.Max(value, minimum)), Mathf.Log10(minimum), Mathf.Log10(maximum));
        return Mathf.Pow(10f, exponent);
    }

    private void OnGUI()
    {
        if (!showControls || material == null) return;
        GUILayout.BeginArea(new Rect(10, 10, PanelWidth, Mathf.Max(100, Screen.height - 20)), GUI.skin.box);
        scroll = GUILayout.BeginScrollView(scroll);
        GUILayout.Label("CLOTH WORKSPACE");
        if(Solver!=null) GUILayout.Label(Solver.Particles.Length+" particles; "+substeps+" substeps x "+solverIterations+" iterations");
        GUILayout.Label("Frame avg/peak: "+AverageFrameMilliseconds.ToString("F1")+" / "+PeakFrameMilliseconds.ToString("F1")+" ms");
        GUILayout.Label("Physics tick avg/peak: "+AverageStepMilliseconds.ToString("F2")+" / "+PeakStepMilliseconds.ToString("F2")+" ms");
        if(clothRenderer!=null) GUILayout.Label("Mesh last frame: "+clothRenderer.LastMeshMilliseconds.ToString("F2")+" ms");
        smoothDisplay=GUILayout.Toggle(smoothDisplay,"Smooth display (+1 physics tick latency)");
        GUILayout.Label("Physical material - changes apply while running");
        GUILayout.Label("Advanced curves, directions and membrane model: Inspector.");
        material.stretchU = LogSlider("Stretch U (N/m)", material.stretchU, 1f, 10000f);
        material.stretchV = LogSlider("Stretch V (N/m)", material.stretchV, 1f, 10000f);
        material.shear = LogSlider("Shear (N/m)", material.shear, 0.1f, 5000f);
        GUILayout.Label("Coupling: " + material.coupling.ToString("F2"));
        material.coupling = GUILayout.HorizontalSlider(material.coupling, -0.9f, 0.9f);
        bool bendingEnabled = GUILayout.Toggle(material.bending > 0f, "Bending resistance");
        if (bendingEnabled) material.bending = LogSlider("Bending (N m)", material.bending, 0.000001f, 0.1f);
        else material.bending = 0f;
        bool dampingEnabled = GUILayout.Toggle(material.bendDamping > 0f, "Bending damping");
        if (dampingEnabled) material.bendDamping = LogSlider("Damping (N m s)", material.bendDamping, 0.0000001f, 0.01f);
        else material.bendDamping = 0f;
        material.arealDensity = LogSlider("Density (kg/m2)", material.arealDensity, 0.02f, 2f);
        GUILayout.Space(8);
        GUILayout.Label("Geometry - changes need Rebuild");
        GUILayout.Label("Rows: " + rows);
        rows = Mathf.RoundToInt(GUILayout.HorizontalSlider(rows, 2, 32));
        GUILayout.Label("Columns: " + columns);
        columns = Mathf.RoundToInt(GUILayout.HorizontalSlider(columns, 2, 32));
        width = LogSlider("Width (m)", width, 0.1f, 2f);
        height = LogSlider("Height (m)", height, 0.1f, 2f);
        pinTopCorners = GUILayout.Toggle(pinTopCorners, "Pin top corners");
        if (GUILayout.Button("Rebuild geometry")) Rebuild();
        GUILayout.BeginHorizontal();
        if (GUILayout.Button(paused ? "Resume" : "Pause"))
        {
            paused = !paused;
            if (Solver != null) Solver.EndGrab();
        }
        if (GUILayout.Button("Reset")) ResetCloth();
        GUILayout.EndHorizontal();
        if(Solver!=null)
        {
            GUILayout.Label("Maximum speed: "+displayedMaxSpeed.ToString("G3")+" m/s");
        }
        GUILayout.Label(status, GUI.skin.label);
        GUILayout.EndScrollView();
        GUILayout.EndArea();
    }

    private void OnDisable()
    {
        if (Solver != null) Solver.EndGrab();
    }
}
