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
    }
    
    public Mesh Build(float3[] positions, float3[] tangents)
    {
        int segments = positions.Length;
        int vertexCount = segments * sides;
        
        vertices  = new List<Vector3>(vertexCount);
        normals   = new List<Vector3>(vertexCount);
        
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
        /*
         * As your cable curves, this can cause the tube to rotate/twist unexpectedly.
         * Once you see the basic tube rendering, the next thing I'd change is the frame
         * calculation to use the previous ring's normal and transport it along the curve.
         * That's the piece that will make your cable look stable and professional rather
         * than like a tube whose orientation is being recalculated independently at every
         * sample.
         */
        float3 reference = math.up();
        float3 normal = math.normalize(math.cross(tangent, reference));

        return normal;
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