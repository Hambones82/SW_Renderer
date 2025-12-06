using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Software_Renderer
{
    public struct Triangle
    {
        public Vec3 V0, V1, V2;

        public Triangle(Vec3 v0, Vec3 v1, Vec3 v2)
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
            var v0 = transformMatrix.Transform(new Vec4(tri.V0, 1));
            var v1 = transformMatrix.Transform(new Vec4(tri.V1, 1));
            var v2 = transformMatrix.Transform(new Vec4(tri.V2, 1));
            tri.V0 = new Vec3(v0);
            tri.V1 = new Vec3(v1);
            tri.V2 = new Vec3(v2);
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
            new Triangle(new Vec3(-1, -1, 1), new Vec3(1, -1, 1), new Vec3(1, 1, 1)),
            new Triangle(new Vec3(-1, -1, 1), new Vec3(1, 1, 1), new Vec3(-1, 1, 1)),
            // Back face
            new Triangle(new Vec3(1, -1, -1), new Vec3(-1, -1, -1), new Vec3(-1, 1, -1)),
            new Triangle(new Vec3(1, -1, -1), new Vec3(-1, 1, -1), new Vec3(1, 1, -1)),
            // Top face
            new Triangle(new Vec3(-1, 1, 1), new Vec3(1, 1, 1), new Vec3(1, 1, -1)),
            new Triangle(new Vec3(-1, 1, 1), new Vec3(1, 1, -1), new Vec3(-1, 1, -1)),
            // Bottom face
            new Triangle(new Vec3(-1, -1, -1), new Vec3(1, -1, -1), new Vec3(1, -1, 1)),
            new Triangle(new Vec3(-1, -1, -1), new Vec3(1, -1, 1), new Vec3(-1, -1, 1)),
            // Right face
            new Triangle(new Vec3(1, -1, 1), new Vec3(1, -1, -1), new Vec3(1, 1, -1)),
            new Triangle(new Vec3(1, -1, 1), new Vec3(1, 1, -1), new Vec3(1, 1, 1)),
            // Left face
            new Triangle(new Vec3(-1, -1, -1), new Vec3(-1, -1, 1), new Vec3(-1, 1, 1)),
            new Triangle(new Vec3(-1, -1, -1), new Vec3(-1, 1, 1), new Vec3(-1, 1, -1)),
            };

            return new Mesh(triangles);
        }
    }
}
