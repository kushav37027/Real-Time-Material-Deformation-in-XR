
using UnityEngine;

public class ClothHandGrab : MonoBehaviour
{
    [Header("Cloth")]
    public ClothSimulation simulation;

    [Header("Meta Hand Tracking")]
    public OVRHand hand;
    public OVRSkeleton skeleton;

    [Header("Interaction")]
    [Tooltip("0 = right hand, 1 = left hand. Each hand must use a different slot.")]
    [Range(0, 1)] public int grabId = 0;
    public float maxGrabDistance = 0.08f;

    private bool ownsGrab;
    private bool touchingPanel;
    private bool previousIndexPinch;
    private bool menuGestureUsed;
    private float middlePinchTime;
    private bool pointerDrag;

    private void UpdateMenuGesture(bool indexPinch)
    {
        bool held = !indexPinch && !ownsGrab && hand.GetFingerIsPinching(OVRHand.HandFinger.Middle);
        if (!held)
        {
            middlePinchTime = 0;
            menuGestureUsed = false;
            return;
        }
        middlePinchTime += Time.unscaledDeltaTime;
        if (!menuGestureUsed && middlePinchTime >= 0.45f)
        {
            panel.ToggleMenu();
            menuGestureUsed = true;
        }
    }
    private ClothHeadsetPanel panel;

    private void Awake()
    {
        if (simulation == null)
            simulation = GetComponent<ClothSimulation>();
        if (simulation != null) panel = simulation.GetComponent<ClothHeadsetPanel>();
    }

    private void Update()
    {
        if (simulation == null || simulation.Solver == null)
            return;

        ClothSolver solver = simulation.Solver;

        if (!simulation.enabled)
        {
            ReleaseGrab();
            return;
        }

        if (hand == null || !hand.IsTracked)
        {
            ReleaseGrab();
            panel?.ClearAim(grabId);
            middlePinchTime = 0;
            menuGestureUsed = false;
            previousIndexPinch = false;
            return;
        }

        bool pinching = hand.GetFingerIsPinching(OVRHand.HandFinger.Index);
        bool began = pinching && !previousIndexPinch;
        previousIndexPinch = pinching;
        if (panel != null) UpdateMenuGesture(pinching);
        if (ownsGrab && !solver.IsGrabbingWith(grabId)) ownsGrab = false;
        if (!ownsGrab && !pointerDrag && panel != null && panel.Visible &&
            skeleton != null && skeleton.IsInitialized && skeleton.IsDataValid)
        {
            Transform fingertip = FindBone(
                OVRSkeleton.BoneId.Hand_IndexTip,
                OVRSkeleton.BoneId.XRHand_IndexTip
            );

            if (fingertip != null &&
                panel.TouchPanel(grabId, fingertip.position, ref touchingPanel))
                return;
        }
        Vector3 hit = default;
        bool pointing = panel != null && panel.Aim(grabId,
            !ownsGrab && hand.IsPointerPoseValid ? hand.PointerPose : null, out hit);
        if (!pinching)
        {
            ReleaseGrab();
            return;
        }
        if (pointerDrag)
        {
            if (pointing && panel.Captured(grabId)) panel.Slide(grabId, hit);
            return;
        }
        if (!ownsGrab && began && pointing && panel.Slide(grabId, hit))
        {
            pointerDrag = true;
            return;
        }
        if (skeleton == null || !skeleton.IsInitialized || !skeleton.IsDataValid)
        {
            ReleaseGrab();
            return;
        }

        Transform thumbTip = FindBone(
            OVRSkeleton.BoneId.Hand_ThumbTip,
            OVRSkeleton.BoneId.XRHand_ThumbTip
        );

        Transform indexTip = FindBone(
            OVRSkeleton.BoneId.Hand_IndexTip,
            OVRSkeleton.BoneId.XRHand_IndexTip
        );

        if (thumbTip == null || indexTip == null)
        {
            ReleaseGrab();
            return;
        }

        // Midpoint between thumb and index fingertips.
        Vector3 pinchPoint =
            (thumbTip.position + indexTip.position) * 0.5f;

        if (!ownsGrab && panel != null && panel.Slide(grabId, pinchPoint)) return;

        if (simulation.paused) return;

        // Find and grab a cloth triangle when pinch begins.
        if (!ownsGrab)
        {
            int triangle;
            Vector3 weights;
            Vector3 surfacePoint;

            if (FindClosestTriangle(
                pinchPoint,
                out triangle,
                out weights,
                out surfacePoint))
            {
                ownsGrab = solver.BeginGrab(
                    grabId,
                    triangle,
                    weights,
                    surfacePoint
                );
            }
        }

        // Move the existing XPBD grab constraint.
        if (ownsGrab)
            solver.MoveGrab(grabId, pinchPoint);
    }

    private Transform FindBone(
        OVRSkeleton.BoneId legacyId,
        OVRSkeleton.BoneId openXRId)
    {
        foreach (OVRBone bone in skeleton.Bones)
        {
            if (bone.Id == legacyId || bone.Id == openXRId)
                return bone.Transform;
        }

        return null;
    }

    private bool FindClosestTriangle(
        Vector3 point,
        out int selectedTriangle,
        out Vector3 selectedWeights,
        out Vector3 selectedPoint)
    {
        selectedTriangle = -1;
        selectedWeights = Vector3.zero;
        selectedPoint = Vector3.zero;

        ClothSolver solver = simulation.Solver;

        float nearestDistance = maxGrabDistance * maxGrabDistance;

        int[] triangles = solver.Geometry.triangles;
        ClothParticle[] particles = solver.Particles;

        for (int t = 0; t < triangles.Length; t += 3)
        {
            int ia = triangles[t];
            int ib = triangles[t + 1];
            int ic = triangles[t + 2];

            Vector3 a = particles[ia].position;
            Vector3 b = particles[ib].position;
            Vector3 c = particles[ic].position;

            Vector3 weights;
            Vector3 closest = ClosestPointOnTriangle(
                point, a, b, c, out weights
            );

            // Don't select a completely pinned triangle.
            float movable =
                particles[ia].InverseMass * weights.x * weights.x +
                particles[ib].InverseMass * weights.y * weights.y +
                particles[ic].InverseMass * weights.z * weights.z;

            if (movable <= 0f)
                continue;

            float distance = (point - closest).sqrMagnitude;

            if (distance < nearestDistance)
            {
                nearestDistance = distance;
                selectedTriangle = t / 3;
                selectedWeights = weights;
                selectedPoint = closest;
            }
        }

        return selectedTriangle >= 0;
    }

    // Returns the closest point and barycentric coordinates.
    private Vector3 ClosestPointOnTriangle(
        Vector3 p,
        Vector3 a,
        Vector3 b,
        Vector3 c,
        out Vector3 weights)
    {
        Vector3 ab = b - a;
        Vector3 ac = c - a;
        Vector3 ap = p - a;

        float d1 = Vector3.Dot(ab, ap);
        float d2 = Vector3.Dot(ac, ap);

        if (d1 <= 0f && d2 <= 0f)
        {
            weights = new Vector3(1, 0, 0);
            return a;
        }

        Vector3 bp = p - b;
        float d3 = Vector3.Dot(ab, bp);
        float d4 = Vector3.Dot(ac, bp);

        if (d3 >= 0f && d4 <= d3)
        {
            weights = new Vector3(0, 1, 0);
            return b;
        }

        float vc = d1 * d4 - d3 * d2;

        if (vc <= 0f && d1 >= 0f && d3 <= 0f)
        {
            float v = d1 / (d1 - d3);
            weights = new Vector3(1 - v, v, 0);
            return a + v * ab;
        }

        Vector3 cp = p - c;
        float d5 = Vector3.Dot(ab, cp);
        float d6 = Vector3.Dot(ac, cp);

        if (d6 >= 0f && d5 <= d6)
        {
            weights = new Vector3(0, 0, 1);
            return c;
        }

        float vb = d5 * d2 - d1 * d6;

        if (vb <= 0f && d2 >= 0f && d6 <= 0f)
        {
            float w = d2 / (d2 - d6);
            weights = new Vector3(1 - w, 0, w);
            return a + w * ac;
        }

        float va = d3 * d6 - d5 * d4;

        if (va <= 0f && (d4 - d3) >= 0f &&
            (d5 - d6) >= 0f)
        {
            float w = (d4 - d3) /
                      ((d4 - d3) + (d5 - d6));

            weights = new Vector3(0, 1 - w, w);
            return b + w * (c - b);
        }

        float denominator = 1f / (va + vb + vc);
        float vFace = vb * denominator;
        float wFace = vc * denominator;
        float uFace = 1f - vFace - wFace;

        weights = new Vector3(uFace, vFace, wFace);

        return uFace * a + vFace * b + wFace * c;
    }

    private void ReleaseGrab()
    {
        touchingPanel = false;
        pointerDrag = false;
        if (panel != null) panel.Release(grabId);
        if (!ownsGrab)
            return;

        if (simulation != null && simulation.Solver != null)
            simulation.Solver.EndGrab(grabId);

        ownsGrab = false;
    }

    private void OnDisable()
    {
        panel?.ClearAim(grabId);
        previousIndexPinch = menuGestureUsed = false;
        middlePinchTime = 0;
        ReleaseGrab();
    }
}
