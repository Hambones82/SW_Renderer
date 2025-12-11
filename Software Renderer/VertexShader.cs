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
            var model = Matrix4x4.CreateRotationY(angle) * Matrix4x4.CreateRotationX(angle * 0.5f);
            var view = Matrix4x4.CreateTranslation(0, 0, 0);
            
            var projection = Matrix4x4.CreatePerspective(width, height, 0.01f, 100);//((float)Math.PI / 4, width / height, 0.1f, 100f);
            mvp = projection * view * model;
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
