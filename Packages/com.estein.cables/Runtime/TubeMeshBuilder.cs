using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

public class TubeMeshBuilder
{
    private int sides;
    private float radius;
    
    private readonly Mesh mesh;
    
    private List<Vector3> vertices;
    private List<Vector3> normals;
    private List<Vector2> uvs;
    private List<int>     triangles;

    private int previousSegments;

    private float3 lastPosition;
    private float3 lastTangent;
    private float3 lastNormal;
    private bool   hasFrame;
    
    public TubeMeshBuilder(int sides, float radius)
    {
        this.sides = sides;
        this.radius = radius;
        
        mesh = new Mesh();
        mesh.MarkDynamic();
        
        vertices  = new List<Vector3>();
        normals   = new List<Vector3>();
    }
    
    public Mesh Build(float3[] positions, float3[] tangents)
    {
        int segments = positions.Length;

        vertices.Clear();
        normals.Clear();

        hasFrame = false;
        
        for (int i = 0; i < segments; i++)
        {
            float3 position = positions[i];
            float3 tangent  = math.normalize(tangents[i]);

            float3 normal = GetNextNormal(position, tangent);
            float3 binormal = math.cross(tangent, normal);

            for (int j = 0; j < sides; j++)
            {
                float angle = math.PI2 * j / sides;
                
                float3 direction =
                    math.cos(angle) * normal +
                    math.sin(angle) * binormal;

                float3 vertex = position + direction * radius;

                vertices.Add(vertex);
                normals.Add(math.normalize(direction));
            }
        }
        
        mesh.SetVertices(vertices);
        mesh.SetNormals(normals);

        if (NewSegments(segments))
        {
            // Only regenerate triangles and UVs when number of segments changes.
            GenerateTriangles(segments);
            GenerateUVs(segments);
            
            mesh.SetUVs(0, uvs);
            mesh.SetTriangles(triangles, 0);
        }
        
        mesh.RecalculateBounds();

        return mesh;
    }

    private float3 GetNextNormal(float3 position, float3 tangent)
    {
        if (!hasFrame)
        {
            lastNormal = CalculateInitialNormal(tangent);
            lastPosition = position;
            lastTangent = tangent;
            hasFrame = true;
            return lastNormal;
        }

        float3 t0 = lastTangent;
        float3 r0 = lastNormal;
        float3 x0 = lastPosition;
        float3 x1 = position;
        float3 t1 = tangent;

        float3 v1 = x1 - x0;
        float c1 = math.dot(v1, v1);

        float3 rL = c1 > 1e-12f
            ? r0 - (2f / c1) * math.dot(v1, r0) * v1
            : r0;
        float3 tL = c1 > 1e-12f
            ? t0 - (2f / c1) * math.dot(v1, t0) * v1
            : t0;

        float3 v2 = t1 - tL;
        float c2 = math.dot(v2, v2);

        float3 r1 = c2 > 1e-12f
            ? rL - (2f / c2) * math.dot(v2, rL) * v2
            : rL;

        r1 = math.normalize(r1 - t1 * math.dot(r1, t1));

        lastPosition = position;
        lastTangent = tangent;
        lastNormal = r1;

        return r1;
    }

    private float3 CalculateInitialNormal(float3 tangent)
    {
        float3 reference = math.abs(math.dot(tangent, math.up())) > 0.99f
            ? math.right()
            : math.up();

        return math.normalize(math.cross(tangent, reference));
    }

    private void GenerateTriangles(int segments)
    {
        triangles = new List<int>((segments - 1) * sides * 6);
        
        for (int i = 0; i < segments - 1; i++)
        {
            int currentRing = i * sides;
            int nextRing = (i + 1) * sides;

            for (int j = 0; j < sides; j++)
            {
                int current = currentRing + j;
                int next = currentRing + (j + 1) % sides;

                int currentNextRing = nextRing + j;
                int nextNextRing =
                    nextRing + (j + 1) % sides;

                triangles.Add(current);
                triangles.Add(next);
                triangles.Add(currentNextRing);

                triangles.Add(next);
                triangles.Add(nextNextRing);
                triangles.Add(currentNextRing);
            }
        }
    }

    private void GenerateUVs(int segments)
    {
        uvs = new List<Vector2>(segments * sides);
        
        for (int i = 0; i < segments; i++)
        {
            for (int j = 0; j < sides; j++)
            {
                uvs.Add(new Vector2(j / (float)sides, i / (float)(segments - 1)));
            }
        }
    }

    private bool NewSegments(int segments)
    {
        if (previousSegments != segments)
        {
            previousSegments = segments;
            return true;
        }
        
        return false;
    }
}
