using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Software_Renderer
{
    /*
    public struct Vec3
    {
        public float X, Y, Z;

        public Vec3(float x, float y, float z)
        {
            X = x; Y = y; Z = z;
        }

        public Vec3(Vec4 v4)
        {
            X = v4.X; Y = v4.Y; Z = v4.Z;
        }

        public static Vec3 operator +(Vec3 a, Vec3 b) => new Vec3(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static Vec3 operator -(Vec3 a, Vec3 b) => new Vec3(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static Vec3 operator *(Vec3 v, float s) => new Vec3(v.X * s, v.Y * s, v.Z * s);

        public static float Dot(Vec3 a, Vec3 b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        public static Vec3 Cross(Vec3 a, Vec3 b) => new Vec3(
            a.Y * b.Z - a.Z * b.Y,
            a.Z * b.X - a.X * b.Z,
            a.X * b.Y - a.Y * b.X
        );
    }
    */
    /*
    public struct Vec4
    {
        public float X, Y, Z, W;

        public Vec4(float x, float y, float z, float w)
        {
            X = x; Y = y; Z = z; W = w;
        }

        public Vec4(Vec3 v, float w) : this(v.X, v.Y, v.Z, w) { }
    }
    */
    /*
    public struct Matrix4x4
    {
        private float[,] m = new float[4, 4];

        public Matrix4x4() 
        {
            //
            //for(int i = 0; i < 4; i++)
            //{
            //    for( int j = 0; j < 4; j++)
            //    {
            //        m[i, j] = 0;
            //    }
            //}
        }

        public float this[int row, int col]
        {
            get => m[row, col];
            set => m[row, col] = value;
        }

        public static Matrix4x4 Identity()
        {
            var mat = new Matrix4x4();
            mat[0, 0] = mat[1, 1] = mat[2, 2] = mat[3, 3] = 1;
            return mat;
        }

        public static Matrix4x4 RotationY(float angle)
        {
            var mat = Identity();
            float c = (float)Math.Cos(angle);
            float s = (float)Math.Sin(angle);
            mat[0, 0] = c; mat[0, 2] = s;
            mat[2, 0] = -s; mat[2, 2] = c;
            return mat;
        }

        public static Matrix4x4 RotationX(float angle)
        {
            var mat = Identity();
            float c = (float)Math.Cos(angle);
            float s = (float)Math.Sin(angle);
            mat[1, 1] = c; mat[1, 2] = -s;
            mat[2, 1] = s; mat[2, 2] = c;
            return mat;
        }

        public static Matrix4x4 Translation(float x, float y, float z)
        {
            var mat = Identity();
            mat[0, 3] = x;
            mat[1, 3] = y;
            mat[2, 3] = z;
            return mat;
        }

        public static Matrix4x4 Perspective(float fov, float aspect, float near, float far)
        {
            var mat = new Matrix4x4();
            float f = 1.0f / (float)Math.Tan(fov / 2);
            mat[0, 0] = f / aspect;
            mat[1, 1] = f;
            mat[2, 2] = (far + near) / (near - far);
            mat[2, 3] = (2 * far * near) / (near - far);
            mat[3, 2] = -1;
            return mat;
        }

        public static Matrix4x4 operator *(Matrix4x4 a, Matrix4x4 b)
        {
            var result = new Matrix4x4();
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    for (int k = 0; k < 4; k++)
                        result[i, j] += a[i, k] * b[k, j];
            return result;
        }
        
        public Vec4 Transform(Vec4 v)
        {
            return new Vec4(
                m[0, 0] * v.X + m[0, 1] * v.Y + m[0, 2] * v.Z + m[0, 3] * v.W,
                m[1, 0] * v.X + m[1, 1] * v.Y + m[1, 2] * v.Z + m[1, 3] * v.W,
                m[2, 0] * v.X + m[2, 1] * v.Y + m[2, 2] * v.Z + m[2, 3] * v.W,
                m[3, 0] * v.X + m[3, 1] * v.Y + m[3, 2] * v.Z + m[3, 3] * v.W
            );
        }
    }*/
}
