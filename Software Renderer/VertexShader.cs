using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Software_Renderer
{
    public interface IVertexShader
    {
        VertexShaderOutput VertexShade(Vec3 position);
    }

    internal class VertexShader : IVertexShader
    {
        private Matrix4x4 mvp;

        public VertexShader(float width, float height)
        {
            float angle = 0.5f;
            var model = Matrix4x4.RotationY(angle) * Matrix4x4.RotationX(angle * 0.5f);
            var view = Matrix4x4.Translation(0, 0, -6);
            
            var projection = Matrix4x4.Perspective((float)Math.PI / 4, width / height, 0.1f, 100f);
            mvp = projection * view * model;
        }

        //also need normals, albedo.  maybe others.  -- work on this next.
        public VertexShaderOutput VertexShade(Vec3 position)
        {
            var transformed = mvp.Transform(new Vec4(position, 1));
            return new VertexShaderOutput(transformed);
        }
    }
}
