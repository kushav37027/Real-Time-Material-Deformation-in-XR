using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

[RequireComponent(typeof(ClothSimulation))]
public class ClothHeadsetPanel : MonoBehaviour
{ 
    public Transform eye;
    private ClothSimulation sim;
    private RectTransform face;
    private Font font;
    private readonly RectTransform[] cursors = new RectTransform[2];
    public bool Visible => isActiveAndEnabled && face != null && face.gameObject.activeSelf;

    public void ToggleMenu()
    {
        if (face == null || !isActiveAndEnabled) return;
        owner = selected = -1;
        foreach (var cursor in cursors) cursor.gameObject.SetActive(false);
        if (Visible)
        {
            face.gameObject.SetActive(false);
            return;
        }
        if (eye == null && Camera.main != null) eye = Camera.main.transform;
        if (eye == null) return;
        face.position = eye.position + eye.forward * 0.60f;
        face.rotation = Quaternion.LookRotation(eye.forward, eye.up);
        face.gameObject.SetActive(true);
    }

    public bool Aim(int id, Transform pointer, out Vector3 hit)
    {
        hit = default;
        if (!Visible) return false;
        bool captured = owner == id;
        if (pointer != null)
        {
            var ray = new Ray(pointer.position, pointer.forward);
            var plane = new Plane(face.forward, face.position);
            if (plane.Raycast(ray, out float distance))
            {
                hit = ray.GetPoint(distance);
                Vector3 p = face.InverseTransformPoint(hit);
                if (captured || face.rect.Contains(new Vector2(p.x, p.y)))
                {
                    cursors[id].anchoredPosition = new Vector2(Mathf.Clamp(p.x,-245,245), Mathf.Clamp(p.y,-306,306));
                    cursors[id].gameObject.SetActive(true);
                    return true;
                }
            }
        }
        cursors[id].gameObject.SetActive(false);
        return false;
    }

    public bool TouchPanel(int handId, Vector3 fingertip, ref bool touching)
{
    if (!Visible)
    {
        touching = false;
        return false;
    }

    Vector3 local = face.InverseTransformPoint(fingertip);
    float depth = Vector3.Dot(fingertip - face.position, face.forward);

    if (touching)
    {
        if (!Captured(handId) || depth < -0.025f || depth > 0.04f)
        {
            Release(handId);
            touching = false;
            ClearAim(handId);
            return false;
        }
    }
    else
    {
        if (owner >= 0 || Mathf.Abs(depth) > 0.008f ||
            !face.rect.Contains(new Vector2(local.x, local.y)))
            return false;
    }

    Vector3 surfacePoint = fingertip - depth * face.forward;
    touching = Slide(handId, surfacePoint) && Captured(handId);

    if (touching)
    {
        cursors[handId].anchoredPosition = new Vector2(
            Mathf.Clamp(local.x, -245f, 245f),
            Mathf.Clamp(local.y, -306f, 306f)
        );
        cursors[handId].gameObject.SetActive(true);
    }

    return touching;
}

    public bool Captured(int id) => Visible && owner == id;
    public void ClearAim(int id)
    {
        if (face != null && cursors[id] != null) cursors[id].gameObject.SetActive(false);
    }
    private readonly List<Row> rows = new List<Row>();
    private int owner = -1, selected = -1;
    private static readonly Color Ink = new Color32(232, 241, 248, 255);
    private static readonly Color Mint = new Color32(93, 228, 194, 255);

    private enum Page { Material, Geometry, Simulation }
    private Page page;
    private RectTransform parent;
    private readonly RectTransform[] pages = new RectTransform[3];
    private readonly Text[] tabs = new Text[3];
    private readonly List<PanelButton> buttons = new List<PanelButton>();
    private Text pauseLabel, smoothLabel, pinLabel, stats, status;
    private float nextStats;

    private class PanelButton
    {
        public RectTransform rect;
        public Action click;
    }

    private class Row
    {
        public Page page;
        public float y, min, max, previous = float.NaN;
        public bool logarithmic, allowZero;
        public Func<float> read;
        public Action<float> write;
        public Text number;
        public RectTransform fill, knob;
    }

    private void Start()
    {
        sim = GetComponent<ClothSimulation>();
        font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        face = new GameObject("Cloth controls", typeof(RectTransform), typeof(Canvas)).GetComponent<RectTransform>();
        face.SetParent(transform, false);
        face.sizeDelta = new Vector2(500, 622);
        face.localScale = Vector3.one * 0.0008f;
        face.GetComponent<Canvas>().renderMode = RenderMode.WorldSpace;
        parent = face;
        Box("Background", 0, 0, 500, 622, new Color32(20, 33, 47, 252));
        Label("Cloth controls", 0, 275, 452, 40, 29, Ink, TextAnchor.MiddleLeft);
        foreach (Page id in Enum.GetValues(typeof(Page)))
        {
            Page target = id;
            int i = (int)id;
            tabs[i] = Button(id.ToString(), (i - 1) * 154, 236, 146, () => Show(target));
            pages[i] = Rect(id.ToString(), 0, 0, 500, 622);
        }
        parent = pages[(int)Page.Material];
        Group("MOTION", 206);
        Add("Particle drag", 169, 0, 6, () => sim.material.particleDrag, v => sim.material.particleDrag = v);
        Add("Bend damping", 110, 1e-7f, .01f, () => sim.material.bendDamping, v => sim.material.bendDamping = v, true, true);
        Group("FABRIC", 51);
        Add("Stretch U", 16, 1, 10000, () => sim.material.stretchU, v => sim.material.stretchU = v, true);
        Add("Stretch V", -43, 1, 10000, () => sim.material.stretchV, v => sim.material.stretchV = v, true);
        Add("Shear", -102, .1f, 5000, () => sim.material.shear, v => sim.material.shear = v, true);
        Add("Bending", -161, 1e-6f, .1f, () => sim.material.bending, v => sim.material.bending = v, true, true);
        Add("Coupling", -220, -.9f, .9f, () => sim.material.coupling, v => sim.material.coupling = v);
        Add("Areal density", -279, .02f, 2, () => sim.material.arealDensity, v => sim.material.arealDensity = v, true);
        page = Page.Geometry;
        parent = pages[(int)page];
        Group("SHEET SIZE", 190);
        Add("Width (m)", 140, .1f, 2, () => sim.width, v => sim.width = v, true);
        Add("Height (m)", 76, .1f, 2, () => sim.height, v => sim.height = v, true);
        Group("GRID", 12);
        Add("Rows", -35, 2, 32, () => sim.rows, v => sim.rows = Mathf.RoundToInt(v));
        Add("Columns", -99, 2, 32, () => sim.columns, v => sim.columns = Mathf.RoundToInt(v));
        pinLabel = Button("Pin top corners", 0, -169, 448, () => sim.pinTopCorners = !sim.pinTopCorners);
        Button("Rebuild geometry", 0, -229, 448, sim.Rebuild);
        Label("Rebuild applies size, grid and pin changes.", 0, -278, 452, 38, 18, Ink, TextAnchor.MiddleLeft);

        page = Page.Simulation;
        parent = pages[(int)page];
        pauseLabel = Button("Pause", -115, 175, 218, () => {
            sim.paused = !sim.paused;
            sim.Solver?.EndGrab();
        });
        Button("Reset cloth", 115, 175, 218, sim.ResetCloth);
        smoothLabel = Button("Smooth display", 0, 118, 448, () => sim.smoothDisplay = !sim.smoothDisplay);
        Group("LIVE READOUTS", 56);
        stats = Label("", 0, -48, 452, 175, 20, Ink, TextAnchor.UpperLeft);
        status = Label("", 0, -223, 452, 108, 19, Mint, TextAnchor.UpperLeft);
        parent = face;
        for (int i = 0; i < cursors.Length; i++)
        {
            cursors[i] = Box("Hand cursor", 0, 0, 12, 12, new Color32(255, 194, 86, 255));
            cursors[i].gameObject.SetActive(false);
        }
        Show(Page.Material);
        face.gameObject.SetActive(false);
    }

    private void LateUpdate()
    {
        if (!Visible) return;
        if (page == Page.Geometry) pinLabel.text = "Pin top corners   " + (sim.pinTopCorners ? "ON" : "OFF");
        if (page == Page.Simulation)
        {
            pauseLabel.text = sim.paused ? "Resume" : "Pause";
            smoothLabel.text = "Smooth display   " + (sim.smoothDisplay ? "ON" : "OFF");
            if (Time.unscaledTime >= nextStats)
            {
                nextStats = Time.unscaledTime + .25f;
                int count = sim.Solver != null ? sim.Solver.Particles.Length : 0;
                stats.text = $"{count} particles   ·   {sim.substeps} substeps × {sim.solverIterations} iterations\n" +
                    $"Maximum speed   {sim.MaximumSpeed:G3} m/s\n" +
                    $"Frame avg / peak   {sim.AverageFrameMilliseconds:F1} / {sim.PeakFrameMilliseconds:F1} ms\n" +
                    $"Physics avg / peak   {sim.AverageStepMilliseconds:F2} / {sim.PeakStepMilliseconds:F2} ms\n" +
                    $"Mesh update   {sim.MeshMilliseconds:F2} ms";
                status.text = (sim.paused ? "Paused" : "Running") + "\n" + sim.Status;
            }
        }
        foreach (Row row in rows)
        {
            if (row.page != page) continue;
            float value = row.read();
            if (value == row.previous) continue;
            row.previous = value;
            float t = row.logarithmic
                ? Mathf.InverseLerp(Mathf.Log(row.min), Mathf.Log(row.max), Mathf.Log(Mathf.Max(row.min, value)))
                : Mathf.InverseLerp(row.min, row.max, value);
            if (row.allowZero) t = value <= 0 ? 0 : .03f + t * .97f;
            row.number.text = value.ToString("G3");
            row.fill.sizeDelta = new Vector2(448 * t, 6);
            row.fill.anchoredPosition = new Vector2(-224 + 224 * t, row.y);
            row.knob.anchoredPosition = new Vector2(-224 + 448 * t, row.y);
        }
    }

    public bool Slide(int hand, Vector3 worldPoint)
    {
        if (!isActiveAndEnabled || face == null || !face.gameObject.activeSelf) return false;
        Vector3 p = face.InverseTransformPoint(worldPoint);
        if (owner != hand)
        {
            if (Mathf.Abs(p.z) > 55 || Mathf.Abs(p.x) > 242 || Mathf.Abs(p.y) > 311) return false;
            if (owner >= 0) return true;
            owner = hand;
            selected = -1;
            foreach (PanelButton button in buttons)
            {
                if (!button.rect.gameObject.activeInHierarchy) continue;
                Vector3 local = button.rect.InverseTransformPoint(worldPoint);
                if (!button.rect.rect.Contains(new Vector2(local.x, local.y))) continue;
                button.click();
                return true;
            }
            selected = rows.FindIndex(r => r.page == page && Mathf.Abs(p.y - r.y) <= 23);
        }
        if (selected < 0) return true;
        Row row = rows[selected];
        float t = Mathf.InverseLerp(-224, 224, p.x);
        float fraction = row.allowZero ? Mathf.InverseLerp(.03f, 1, t) : t;
        float value = row.logarithmic ? Mathf.Exp(Mathf.Lerp(Mathf.Log(row.min), Mathf.Log(row.max), fraction)) : Mathf.Lerp(row.min, row.max, fraction);
        row.write(row.allowZero && t < .03f ? 0 : value);
        return true;
    }

    public void Release(int hand) { if (owner == hand) owner = -1; }
    private void OnDisable() { owner = -1; if (face != null) face.gameObject.SetActive(false); }

    private void Show(Page next)
    {
        page = next;
        nextStats = 0;
        for (int i = 0; i < pages.Length; i++)
        {
            pages[i].gameObject.SetActive(i == (int)page);
            tabs[i].color = i == (int)page ? Mint : Ink;
        }
    }

    private Text Button(string title, float x, float y, float width, Action click)
    {
        var rect = Box(title, x, y, width, 41, new Color32(47, 67, 86, 255));
        buttons.Add(new PanelButton { rect = rect, click = click });
        return Label(title, x, y, width, 40, 20, Ink, TextAnchor.MiddleCenter);
    }

    private void Group(string name, float y)
    {
        Label(name, 0, y, 452, 24, 16, Mint, TextAnchor.MiddleLeft);
        Box("Divider", 44, y, 360, 1, new Color32(66, 85, 102, 255));
    }

    private void Add(string name, float y, float min, float max, Func<float> read, Action<float> write, bool log = false, bool zero = false)
    {
        Label(name, -45, y + 10, 362, 30, 21, Ink, TextAnchor.MiddleLeft);
        var row = new Row { page = page, y = y - 12, min = min, max = max, read = read, write = write, logarithmic = log, allowZero = zero };
        row.number = Label("", 171, y + 10, 110, 30, 19, Mint, TextAnchor.MiddleRight);
        Box("Track", 0, row.y, 448, 6, new Color32(69, 90, 108, 255));
        row.fill = Box("Fill", 0, row.y, 0, 6, Mint);
        row.knob = Box("Handle", 0, row.y, 16, 18, Ink);
        rows.Add(row);
    }

    private RectTransform Rect(string name, float x, float y, float width, float height)
    {
        var rect = new GameObject(name, typeof(RectTransform)).GetComponent<RectTransform>();
        rect.SetParent(parent, false);
        rect.sizeDelta = new Vector2(width, height);
        rect.anchoredPosition = new Vector2(x, y);
        return rect;
    }
    private RectTransform Box(string name, float x, float y, float w, float h, Color color)
    {
        var rect = Rect(name, x, y, w, h);
        rect.gameObject.AddComponent<Image>().color = color;
        return rect;
    }
    private Text Label(string name, float x, float y, float w, float h, int size, Color color, TextAnchor align)
    {
        var label = Rect(name, x, y, w, h).gameObject.AddComponent<Text>();
        label.font = font;
        label.fontSize = size;
        label.color = color;
        label.alignment = align;
        label.text = name;
        return label;
    }
}
