using System;
using UnityEngine;

// Numerical helpers shared by the membrane and bending constraints.
public static class ClothMath
{
    public static bool IsFinite(Vector3 value)
    {
        return !(float.IsNaN(value.x) || float.IsNaN(value.y) || float.IsNaN(value.z) ||
                 float.IsInfinity(value.x) || float.IsInfinity(value.y) || float.IsInfinity(value.z));
    }

    public static float SignedAngle(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
    {
        Vector3 edge = p1 - p0;
        Vector3 n1 = Vector3.Cross(edge, p2 - p0).normalized;
        Vector3 n2 = Vector3.Cross(p3 - p0, edge).normalized;
        return Mathf.Atan2(Vector3.Dot(edge.normalized, Vector3.Cross(n1, n2)), Vector3.Dot(n1, n2));
    }

    public static float WrapAngle(float angle)
    {
        return Mathf.Atan2(Mathf.Sin(angle), Mathf.Cos(angle));
    }

    // g has four entries: one angle gradient for each vertex of a hinge.
    public static bool AngleGradients(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, Vector3[] g)
    {
        Vector3 edge = p1 - p0;
        Vector3 n1 = Vector3.Cross(edge, p2 - p0);
        Vector3 n2 = Vector3.Cross(p3 - p0, edge);
        if (edge.sqrMagnitude < 1e-12f || n1.sqrMagnitude < 1e-16f || n2.sqrMagnitude < 1e-16f)
            return false;

        float length = edge.magnitude;
        float t2 = Vector3.Dot(p2 - p0, edge) / edge.sqrMagnitude;
        float t3 = Vector3.Dot(p3 - p0, edge) / edge.sqrMagnitude;
        g[2] = -length * n1 / n1.sqrMagnitude;
        g[3] = -length * n2 / n2.sqrMagnitude;
        g[0] = -(1f - t2) * g[2] - (1f - t3) * g[3];
        g[1] = -t2 * g[2] - t3 * g[3];
        return true;
    }

    // Solve A*x = rhs for symmetric positive-definite A.
    // Arguments are A00, A01, A02, A11, A12, A22 (only the upper triangle).
    public static Vector3 SolveSymmetric(
        double a, double b, double c, double d, double e, double f, Vector3 rhs)
    {
        if (a <= 0) throw new InvalidOperationException("Invalid constraint matrix.");
        double l00 = Math.Sqrt(a);
        double l10 = b / l00;
        double l20 = c / l00;
        double pivot = d - l10 * l10;
        if (pivot <= 0) throw new InvalidOperationException("Constraint matrix lost positive definiteness.");
        double l11 = Math.Sqrt(pivot);
        double l21 = (e - l20 * l10) / l11;
        pivot = f - l20 * l20 - l21 * l21;
        if (pivot <= 0) throw new InvalidOperationException("Constraint matrix lost positive definiteness.");
        double l22 = Math.Sqrt(pivot);

        double y0 = rhs.x / l00;
        double y1 = (rhs.y - l10 * y0) / l11;
        double y2 = (rhs.z - l20 * y0 - l21 * y1) / l22;
        double z = y2 / l22;
        double y = (y1 - l21 * z) / l11;
        double x = (y0 - l10 * y - l20 * z) / l00;
        return new Vector3((float)x, (float)y, (float)z);
    }
}
