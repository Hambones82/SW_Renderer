using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Software_Renderer
{
    public struct Fragment
    {
        public int X, Y;
        public float Depth;
        public float BarycentricW0, BarycentricW1, BarycentricW2;
        public int TriangleIndex;

        public Fragment(int x, int y, float depth, float w0, float w1, float w2, int triIndex)
        {
            X = x;
            Y = y;
            Depth = depth;
            BarycentricW0 = w0;
            BarycentricW1 = w1;
            BarycentricW2 = w2;
            TriangleIndex = triIndex;
        }
    }

    public class Rasterizer
    {
        
    }

    public struct ShadedFragment
    {
        public int X, Y;
        public float Depth;
        public uint Color;

        public ShadedFragment(int x, int y, float depth, uint color)
        {
            X = x;
            Y = y;
            Depth = depth;
            Color = color;
        }
    }
    
    public class OutputMerger
    {        

        public OutputMerger()
        {
            
        }

        public void ProcessFragment(ShadedFragment fragment, FrameBuffer framebuffer)
        {            
            framebuffer.SetPixel(fragment.X, fragment.Y, fragment.Depth, fragment.Color);
        }

        public void ProcessFragments(List<ShadedFragment> fragments, FrameBuffer framebuffer)
        {
            foreach (var fragment in fragments)
            {
                ProcessFragment(fragment, framebuffer);
            }
        }
    }
    
    public struct VertexShaderOutput
    {
        public Vec4 Position;

        public VertexShaderOutput(Vec4 position)
        {
            Position = position;
        }
    }

    public class RenderingPipeline
    {
        private int width;
        private int height;      
        private Rasterizer rasterizer;
        private IPixelShader pixelShader;
        private OutputMerger outputMerger;
        private IVertexShader vertexShader;

        private float EdgeFunction(Vec3 a, Vec3 b, Vec3 c)
        {
            return (c.X - a.X) * (b.Y - a.Y) - (c.Y - a.Y) * (b.X - a.X);
        }
      
        public void Rasterize(Vec3 v0, Vec3 v1, Vec3 v2, int triIndex, int fbWidth, int fbHeight, FrameBuffer frameBuffer)
        {            
            float minX = Math.Min(v0.X, Math.Min(v1.X, v2.X));
            float maxX = Math.Max(v0.X, Math.Max(v1.X, v2.X));
            float minY = Math.Min(v0.Y, Math.Min(v1.Y, v2.Y));
            float maxY = Math.Max(v0.Y, Math.Max(v1.Y, v2.Y));

            int x0 = Math.Max(0, (int)Math.Floor(minX));
            int x1 = Math.Min(fbWidth - 1, (int)Math.Ceiling(maxX));
            int y0 = Math.Max(0, (int)Math.Floor(minY));
            int y1 = Math.Min(fbHeight - 1, (int)Math.Ceiling(maxY));

            float area = EdgeFunction(v0, v1, v2);
            if (area <= 0) return;

            for (int y = y0; y <= y1; y++)
            {
                for (int x = x0; x <= x1; x++)
                {                     

                    float pX = x + 0.5f;
                    float pY = y + 0.5f;

                    //float w0 = EdgeFunction(v1, v2, p);
                    float w0 = (pX - v1.X) * (v2.Y - v1.Y) - (pY - v1.Y) * (v2.X - v1.X);
                    //float w1 = EdgeFunction(v2, v0, p);
                    float w1 = (pX - v2.X) * (v0.Y - v2.Y) - (pY - v2.Y) * (v0.X - v2.X);
                    //float w2 = EdgeFunction(v0, v1, p);
                    float w2 = (pX - v0.X) * (v1.Y - v0.Y) - (pY - v0.Y) * (v1.X - v0.X);

                    bool inside = w0 >= 0 && w1 >= 0 && w2 >= 0;

                    if (inside)
                    {
                        w0 /= area;
                        w1 /= area;
                        w2 /= area;

                        float depth = w0 * v0.Z + w1 * v1.Z + w2 * v2.Z;

                        PixelShade(new Fragment(x, y, depth, w0, w1, w2, triIndex), out uint color);
                        frameBuffer.SetPixel(x, y, depth, color);
                    }
                }
            }

        }


        public RenderingPipeline(int width, int height, IPixelShader pixelShader, IVertexShader vertexShader)
        {
            this.width = width;
            this.height = height;
            this.pixelShader = pixelShader;
            this.vertexShader = vertexShader;

            rasterizer = new Rasterizer();
            outputMerger = new OutputMerger();
        }

        public void NewFrame(uint clearColor, FrameBuffer renderBuffer)
        {         
            renderBuffer.Fill(clearColor);
        }

        public void RenderMesh(Mesh mesh, FrameBuffer framebuffer)
        {
            for (int i = 0; i < mesh.Triangles.Length; i++)
            {
                Triangle tri = mesh.Triangles[i];

                VertexShaderOutput v0 = vertexShader.VertexShade(tri.V0);
                VertexShaderOutput v1 = vertexShader.VertexShade(tri.V1);
                VertexShaderOutput v2 = vertexShader.VertexShade(tri.V2);

                v0 = PerspectiveDivide(v0);
                v1 = PerspectiveDivide(v1);
                v2 = PerspectiveDivide(v2);

                Vec3 s0 = ViewportTransform(v0.Position);
                Vec3 s1 = ViewportTransform(v1.Position);
                Vec3 s2 = ViewportTransform(v2.Position);

                Rasterize(s0, s1, s2, i, width, height, framebuffer);
            }
        }

        private VertexShaderOutput PerspectiveDivide(VertexShaderOutput vertex)
        {
            return new VertexShaderOutput(new Vec4(
                vertex.Position.X / vertex.Position.W,
                vertex.Position.Y / vertex.Position.W,
                vertex.Position.Z / vertex.Position.W,
                1
            ));
        }

        private Vec3 ViewportTransform(Vec4 ndc)
        {
            return new Vec3(
                (ndc.X + 1) * width / 2,
                (1 - ndc.Y) * height / 2,
                ndc.Z
            );
        }

        private void PixelShade(Fragment fragment, out uint color)
        {
            color = pixelShader.Shade(fragment);            
        }

        private void OutputMerge(ShadedFragment shadedFragment, FrameBuffer framebuffer)
        {
            outputMerger.ProcessFragment(shadedFragment, framebuffer);
        }
    }

    
    internal class SWRenderer
    {        
        private int _width, _height;

        public SWRenderer(int width, int height)
        {
            _height = height;
            _width = width;
        }
        
        public void Render(Mesh mesh, FrameBuffer framebuffer)
        {            
            uint[] colors = { 0xFFFF0000, 0xFF00FF00, 0xFF0000FF, 0xFF00F00F, 0xFF0000FF, 0xFF00000F };
            var pixelShader = new DepthShader(); 
            var vertexShader = new VertexShader(_width, _height);
            
            var pipeline = new RenderingPipeline(_width, _height, pixelShader, vertexShader);

            pipeline.RenderMesh(mesh, framebuffer);
            
        }
    }
}
