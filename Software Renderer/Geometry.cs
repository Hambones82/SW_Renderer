using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Software_Renderer
{
    public struct Triangle
    {
        public Vector3 V0, V1, V2;

        public Triangle(Vector3 v0, Vector3 v1, Vector3 v2)
        {
            V0 = v0; V1 = v1; V2 = v2;
        }

        public Triangle(Triangle copy_from)
        {
            V0 = copy_from.V0;
            V1 = copy_from.V1;
            V2 = copy_from.V2;
        }

        public Triangle Transform(Matrix4x4 transformMatrix)
        {
            var tri = new Triangle(this);
            var v0 = Vector4.Transform(new Vector4(tri.V0, 1), transformMatrix);
            var v1 = Vector4.Transform(new Vector4(tri.V1, 1), transformMatrix  );
            var v2 = Vector4.Transform(new Vector4(tri.V2, 1), transformMatrix);
            tri.V0 = v0.AsVector3();
            tri.V1 = v1.AsVector3();
            tri.V2 = v2.AsVector3();
            return tri;
        }
    }

    public struct Mesh
    {
        public Triangle[] Triangles;

        public Mesh(Triangle[] triangles)
        {
            Triangles = triangles;
        }

        public static Mesh Rotate(Mesh mesh, Matrix4x4 rotation)
        {
            var rotatedTriangles = new Triangle[mesh.Triangles.Length];
            for (int i = 0; i < mesh.Triangles.Length; i++)
            {
                rotatedTriangles[i] = mesh.Triangles[i].Transform(rotation);
            }
            return new Mesh(rotatedTriangles);
        }

        public static Mesh Translate(Mesh mesh, Matrix4x4 translation)
        {
            var translatedTriangles = new Triangle[mesh.Triangles.Length];
            for (int i = 0; i < mesh.Triangles.Length; i++)
            {
                translatedTriangles[i] = mesh.Triangles[i].Transform(translation);
            }
            return new Mesh(translatedTriangles);
        }
    }

    public static class CubeFactory
    {
        public static Mesh CreateCube()
        {
            var triangles = new Triangle[]
            {
            // Front face
            new Triangle(new Vector3(-1, -1, 1), new Vector3(1, -1, 1), new Vector3(1, 1, 1)),
            new Triangle(new Vector3(-1, -1, 1), new Vector3(1, 1, 1), new Vector3(-1, 1, 1)),
            // Back face
            new Triangle(new Vector3(1, -1, -1), new Vector3(-1, -1, -1), new Vector3(-1, 1, -1)),
            new Triangle(new Vector3(1, -1, -1), new Vector3(-1, 1, -1), new Vector3(1, 1, -1)),
            // Top face
            new Triangle(new Vector3(-1, 1, 1), new Vector3(1, 1, 1), new Vector3(1, 1, -1)),
            new Triangle(new Vector3(-1, 1, 1), new Vector3(1, 1, -1), new Vector3(-1, 1, -1)),
            // Bottom face
            new Triangle(new Vector3(-1, -1, -1), new Vector3(1, -1, -1), new Vector3(1, -1, 1)),
            new Triangle(new Vector3(-1, -1, -1), new Vector3(1, -1, 1), new Vector3(-1, -1, 1)),
            // Right face
            new Triangle(new Vector3(1, -1, 1), new Vector3(1, -1, -1), new Vector3(1, 1, -1)),
            new Triangle(new Vector3(1, -1, 1), new Vector3(1, 1, -1), new Vector3(1, 1, 1)),
            // Left face
            new Triangle(new Vector3(-1, -1, -1), new Vector3(-1, -1, 1), new Vector3(-1, 1, 1)),
            new Triangle(new Vector3(-1, -1, -1), new Vector3(-1, 1, 1), new Vector3(-1, 1, -1)),
            };

            return new Mesh(triangles);
        }
    }
}
