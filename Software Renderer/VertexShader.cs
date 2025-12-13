using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

namespace Software_Renderer
{
    public interface IVertexShader
    {
        VertexShaderOutput VertexShade(Vector3 position);
    }

    internal class VertexShader : IVertexShader
    {
        public Matrix4x4 mvp;

        public VertexShader(float width, float height)
        {
            float angle = 0.5f;
            var model = Matrix4x4.CreateRotationY(angle) * Matrix4x4.CreateTranslation(0,0,-3f);
            var view = Matrix4x4.CreateLookAt(new Vector3(0,0,5), Vector3.Zero, Vector3.UnitY);

            var projection = Matrix4x4.CreatePerspectiveFieldOfView(MathF.PI / 4f, width / height, 0.1f, 1000f);
            mvp = model * view * projection;
        }
        
        public void SetMVP(float width, float height, float angle)
        {            
            var model = Matrix4x4.CreateRotationY(angle) * Matrix4x4.CreateTranslation(0, 0, -3f);
            var view = Matrix4x4.CreateLookAt(new Vector3(0, 0, 5), Vector3.Zero, Vector3.UnitY);

            var projection = Matrix4x4.CreatePerspectiveFieldOfView(MathF.PI / 4f, width / height, 0.1f, 1000f);
            mvp = model * view * projection;
        }

        public void SetMVP(float width, float height, float angle, float translation)
        {
            var model = Matrix4x4.CreateRotationY(angle) * Matrix4x4.CreateTranslation(0 + translation, 0 + translation, -3f + translation);
            var view = Matrix4x4.CreateLookAt(new Vector3(0, 0, 5), Vector3.Zero, Vector3.UnitY);

            var projection = Matrix4x4.CreatePerspectiveFieldOfView(MathF.PI / 4f, width / height, 0.1f, 1000f);
            mvp = model * view * projection;
        }


        //also need normals, albedo.  maybe others.  -- work on this next.
        public VertexShaderOutput VertexShade(Vector3 position)
        {
            var transformed = Vector4.Transform(new Vector4(position, 1), mvp);
            return new VertexShaderOutput(transformed);
        }

        public void SetMVP(Matrix4x4 mvp)
        {
            this.mvp = mvp;
        }

        public Matrix4x4 GetMVP()
        {
            return mvp;
        }
    }
}
