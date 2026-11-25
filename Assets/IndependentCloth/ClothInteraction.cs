using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

// Desktop adapter. A later VR adapter can call the same solver grab methods.
[RequireComponent(typeof(ClothSimulation))]
public class ClothInteraction : MonoBehaviour
{
    public Camera interactionCamera;
    private ClothSimulation simulation;
    private float grabDepth;

    private void Awake()
    {
        simulation = GetComponent<ClothSimulation>();
        if (interactionCamera == null) interactionCamera = Camera.main;
    }

    private void Update()
    {
        if (simulation.Solver == null || interactionCamera == null) return;
        if (!simulation.enabled || simulation.paused)
        {
            simulation.Solver.EndGrab();
            return;
        }

#if ENABLE_INPUT_SYSTEM || ENABLE_LEGACY_INPUT_MANAGER
        Vector2 pointer;
        bool pressed, held;
        float scroll;
#if ENABLE_INPUT_SYSTEM
        Mouse mouse = Mouse.current;
        if (mouse == null) { simulation.Solver.EndGrab(); return; }
        pointer = mouse.position.ReadValue();
        pressed = mouse.leftButton.wasPressedThisFrame;
        held = mouse.leftButton.isPressed;
        scroll = mouse.scroll.ReadValue().y / 120f;
#else
        pointer = Input.mousePosition;
        pressed = Input.GetMouseButtonDown(0);
        held = Input.GetMouseButton(0);
        scroll = Input.mouseScrollDelta.y;
#endif
        ClothSolver solver = simulation.Solver;
        if (!held) solver.EndGrab();
        Ray ray = interactionCamera.ScreenPointToRay(pointer);
        if (pressed && !simulation.PointerOverControls(pointer))
        {
            float nearest = float.PositiveInfinity;
            int selected = -1;
            Vector3 selectedWeights = Vector3.zero;
            int[] triangles = solver.Geometry.triangles;
            for (int t = 0; t < triangles.Length; t += 3)
            {
                float distance;
                Vector3 weights;
                if (IntersectTriangle(ray,
                    solver.Particles[triangles[t]].position,
                    solver.Particles[triangles[t + 1]].position,
                    solver.Particles[triangles[t + 2]].position,
                    out distance, out weights) && distance < nearest)
                {
                    nearest = distance;
                    selected = t / 3;
                    selectedWeights = weights;
                }
            }
            if (selected >= 0)
            {
                Vector3 hit = ray.GetPoint(nearest);
                if (solver.BeginGrab(selected, selectedWeights, hit))
                    grabDepth = Vector3.Dot(hit - interactionCamera.transform.position, interactionCamera.transform.forward);
            }
        }
        if (solver.IsGrabbing && held)
        {
            // A grab started on cloth stays responsive over the panel.
            // Do not let panel scrolling alter grab depth.
            if (simulation.PointerOverControls(pointer)) scroll=0f;
            grabDepth = Mathf.Clamp(grabDepth - scroll * 0.05f, 0.15f, 20f);
            Vector3 forward = interactionCamera.transform.forward;
            float denominator = Vector3.Dot(ray.direction, forward);
            float originDepth = Vector3.Dot(ray.origin - interactionCamera.transform.position, forward);
            if (denominator > 0.0001f)
                solver.MoveGrab(ray.GetPoint((grabDepth - originDepth) / denominator));
        }
#endif
    }

    // Ray/triangle intersection returns barycentric weights, independent of MeshCollider.
    public static bool IntersectTriangle(Ray ray, Vector3 a, Vector3 b, Vector3 c,
        out float distance, out Vector3 weights)
    {
        distance = 0f;
        weights = Vector3.zero;
        Vector3 e1 = b - a, e2 = c - a;
        Vector3 p = Vector3.Cross(ray.direction, e2);
        float determinant = Vector3.Dot(e1, p);
        if (Mathf.Abs(determinant) < 1e-9f) return false;
        float inverse = 1f / determinant;
        Vector3 t = ray.origin - a;
        float u = Vector3.Dot(t, p) * inverse;
        if (u < 0f || u > 1f) return false;
        Vector3 q = Vector3.Cross(t, e1);
        float v = Vector3.Dot(ray.direction, q) * inverse;
        if (v < 0f || u + v > 1f) return false;
        distance = Vector3.Dot(e2, q) * inverse;
        weights = new Vector3(1f - u - v, u, v);
        return distance > 0f;
    }

    private void OnApplicationFocus(bool hasFocus)
    {
        if (!hasFocus && simulation != null && simulation.Solver != null) simulation.Solver.EndGrab();
    }

    private void OnDisable()
    {
        if (simulation != null && simulation.Solver != null) simulation.Solver.EndGrab();
    }
}
