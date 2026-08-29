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
        
        for (int i = 0; i < segments; i++)
        {
            float3 position = positions[i];
            float3 tangent  = tangents[i];
            float3 normal   = CalculateNormal(tangent);
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

    private float3 CalculateNormal(float3 tangent)
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