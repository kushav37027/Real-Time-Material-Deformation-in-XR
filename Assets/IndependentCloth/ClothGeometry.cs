using UnityEngine;
public class ClothGeometry
{
    public readonly int rows;
    public readonly int columns;
    public readonly Vector3[] restPositions;
    public readonly Vector2[] materialCoordinates;
    public readonly int[] triangles;
    public readonly float[] vertexAreas;
    public ClothGeometry(int rows, int columns, float width, float height, Vector3 origin)
    {
        if (rows < 2 || columns < 2)
            throw new System.ArgumentException("The cloth needs at least two rows and two columns.");
        RequirePositive(width, nameof(width));
        RequirePositive(height, nameof(height));
        this.rows = rows;
        this.columns = columns;
        int vertexCount = rows * columns;
        int triangleCount = 2 * (rows - 1) * (columns - 1);
        restPositions = new Vector3[vertexCount];
        materialCoordinates = new Vector2[vertexCount];
        triangles = new int[triangleCount * 3];
        vertexAreas = new float[vertexCount];
        CreateVertices(width, height, origin);
        CreateTriangles();
        CalculateVertexAreas();
    }
    public ClothGeometry(Vector3[] vertices, int[] indices, Vector2[] uv)
    {
        if (vertices == null || indices == null || uv == null || vertices.Length < 3 || uv.Length != vertices.Length || indices.Length == 0 || indices.Length % 3 != 0)
            throw new System.ArgumentException("A rest mesh needs vertices, triangle indices, and one UV per vertex.");
        rows = columns = 0;
        restPositions = (Vector3[])vertices.Clone();
        triangles = (int[])indices.Clone();
        materialCoordinates = (Vector2[])uv.Clone();
        vertexAreas = new float[vertices.Length];
        foreach (Vector3 p in restPositions) if (!ClothMath.IsFinite(p)) throw new System.ArgumentException("Invalid rest vertex.");
        foreach (int i in triangles) if (i < 0 || i >= vertices.Length) throw new System.ArgumentException("Invalid triangle index.");
        CalculateVertexAreas();
        for(int i=0;i<vertexAreas.Length;i++) if(vertexAreas[i]<=1e-12f) throw new System.ArgumentException("Unused/degenerate vertex in rest mesh.");
        for(int t=0;t<triangles.Length;t+=3) { Vector3 u,v; Frame(t,out u,out v); }
    }

    public void Frame(int t, out Vector3 u, out Vector3 v)
    {
        int a=triangles[t], b=triangles[t+1], c=triangles[t+2];
        Vector3 e1=restPositions[b]-restPositions[a], e2=restPositions[c]-restPositions[a];
        Vector2 uv1=materialCoordinates[b]-materialCoordinates[a], uv2=materialCoordinates[c]-materialCoordinates[a];
        float det=uv1.x*uv2.y-uv1.y*uv2.x;
        Vector3 normal=Vector3.Cross(e1,e2);
        if (Mathf.Abs(det)<1e-12f || normal.sqrMagnitude<1e-20f)
            throw new System.ArgumentException("Degenerate rest face or UV face.");
        u=((e1*uv2.y-e2*uv1.y)/det).normalized;
        v=Vector3.Cross(normal.normalized,u).normalized;
        if(!ClothMath.IsFinite(u)||!ClothMath.IsFinite(v)) throw new System.ArgumentException("Nonfinite rest frame.");
    }

    private void CreateVertices(float width, float height, Vector3 origin)
    {
        float spacingX = width / (columns - 1);
        float spacingY = height / (rows - 1);
        for (int row = 0; row < rows; row++)
            for (int column = 0; column < columns; column++)
            {
                int index = row * columns + column;
                float u = column * spacingX;
                float v = row * spacingY;
                materialCoordinates[index] = new Vector2(u, v);
                restPositions[index] = origin + new Vector3(u, v, 0f);
            }
    }
    private void CreateTriangles()
    {
        int next = 0;
        for (int row = 0; row < rows - 1; row++)
            for (int column = 0; column < columns - 1; column++)
            {
                int a = row * columns + column;
                int b = a + 1;
                int c = a + columns;
                int d = c + 1;
                triangles[next++] = a;
                triangles[next++] = b;
                triangles[next++] = c;
                triangles[next++] = b;
                triangles[next++] = d;
                triangles[next++] = c;
            }
    }
    private void CalculateVertexAreas()
    {
        for (int t = 0; t < triangles.Length; t += 3)
        {
            int a = triangles[t], b = triangles[t + 1], c = triangles[t + 2];
            Vector3 edge1 = restPositions[b] - restPositions[a];
            Vector3 edge2 = restPositions[c] - restPositions[a];
            float area = 0.5f * Vector3.Cross(edge1, edge2).magnitude;
            if (area <= 1e-12f) throw new System.ArgumentException("Degenerate rest triangle.");
            float share = area / 3f;
            vertexAreas[a] += share;
            vertexAreas[b] += share;
            vertexAreas[c] += share;
        }
    }
    public ClothParticle[] CreateParticles(ClothMaterial material, bool pinTopCorners = true)
    {
        if (material == null) throw new System.ArgumentNullException(nameof(material));
        material.Validate();
        ClothParticle[] particles = new ClothParticle[restPositions.Length];
        for (int i = 0; i < particles.Length; i++)
        {
            int row = columns > 0 ? i / columns : -1;
            int column = columns > 0 ? i % columns : -1;
            bool isTopCorner = columns > 0 && row == rows - 1 && (column == 0 || column == columns - 1);
            bool pinned = pinTopCorners && isTopCorner;
            float mass = material.arealDensity * vertexAreas[i];
            particles[i] = new ClothParticle(restPositions[i], mass, pinned);
        }
        return particles;
    }
    private static void RequirePositive(float value, string parameterName)
    {
        if (float.IsNaN(value) || float.IsInfinity(value) || value <= 0f)
            throw new System.ArgumentOutOfRangeException(parameterName, "Value must be finite and greater than zero.");
    }
}
