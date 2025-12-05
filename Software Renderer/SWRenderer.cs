using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;
using System.Diagnostics;

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
        private IPixelShader pixelShader;        
        private IVertexShader vertexShader;

        private float EdgeFunction(Vec3 a, Vec3 b, Vec3 c)
        {
            return (c.X - a.X) * (b.Y - a.Y) - (c.Y - a.Y) * (b.X - a.X);
        }
      
        public void Rasterize(Vec3 v0, Vec3 v1, Vec3 v2, int triIndex, int fbWidth, int fbHeight, FrameBuffer frameBuffer)
        {
            int skippedChunks = 0;

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
            int SIMDcount = Vector<float>.Count;

            var v0Xp5 = new Vector<float>(0.5f - v0.X);
            var v1Xp5 = new Vector<float>(0.5f - v1.X);
            var v2Xp5 = new Vector<float>(0.5f - v2.X);

            Parallel.For(y0, y1 + 1,
                (y) =>
                {                    
                    float pY = (float)y + 0.5f;
                    int pixelNum = y * width + x0;
                    //this is assming the SIMD is a certain bit size...                    
                    Span<float> vals = stackalloc float[SIMDcount];

                    //Vector<float> xValues = new Vector<float>(x0);
                    for (int i = 0; i < SIMDcount; i++)
                    {
                        vals[i] = x0 + i;
                    }

                    Vector<float> xValues = new Vector<float>(vals);

                    bool steppedIn = false;
                    Vector<float> w0Scalar0 = new Vector<float>(v2.Y - v1.Y);                    
                    Vector<float> w0RHS = new Vector<float>((pY - v1.Y) * (v2.X - v1.X));

                    Vector<float> w1Scalar0 = new Vector<float>(v0.Y - v2.Y);                    
                    Vector<float> w1RHS = new Vector<float>((pY - v2.Y) * (v0.X - v2.X));

                    Vector<float> w2Scalar0 = new Vector<float>(v1.Y - v0.Y);                    
                    Vector<float> w2RHS = new Vector<float>((pY - v0.Y) * (v1.X - v0.X));

                    for (int x = x0; x <= x1; x+= SIMDcount)
                    {
                        //float pX = x + 0.5f;
                        //looks like yes, these values are still problematic...                 
                        Vector<float> pXw0 = xValues + v1Xp5;
                        Vector<float> pXw1 = xValues + v2Xp5;
                        Vector<float> pXw2 = xValues + v0Xp5;
                        //float w0 = (pX - v1.X) * (v2.Y - v1.Y) - (pY - v1.Y) * (v2.X - v1.X); 
                        Vector<float> w0 = pXw0 * w0Scalar0 - w0RHS;
                        //float w1 = (pX - v2.X) * (v0.Y - v2.Y) - (pY - v2.Y) * (v0.X - v2.X);                        
                        Vector<float> w1 = pXw1 * w1Scalar0 - w1RHS;
                        //float w2 = (pX - v0.X) * (v1.Y - v0.Y) - (pY - v0.Y) * (v1.X - v0.X);
                        Vector<float> w2 = pXw2 * w2Scalar0 - w2RHS;

                        //vector bool?

                        //Vector4 inside =  w0 >= Vector4.Zero && w1 >= Vector4.Zero && w2 >= Vector4.Zero;
                        var inside = 
                        Vector.GreaterThanOrEqual(w0, Vector<float>.Zero) &
                        Vector.GreaterThanOrEqual(w1, Vector<float>.Zero) &
                        Vector.GreaterThanOrEqual(w2, Vector<float>.Zero);
                        
                        
                        if(!Vector.EqualsAll(inside, Vector<int>.Zero))
                        {                            
                            w0 /= area;
                            w1 /= area;
                            w2 /= area;

                            var depth = w0 * v0.Z + w1 * v1.Z + w2 * v2.Z;

                            //compare depth to at dest...
                            //potential buffer overflow (if we start at last position in depth buffer, vector width can cause overflow
                            Vector<float> storedDepths = new Vector<float>(frameBuffer.depth, pixelNum);
                            Vector<int> depthComp = Vector.LessThanOrEqual(depth, storedDepths);
                            var writeMask = inside & depthComp;
                            if(!Vector.EqualsAll(writeMask, Vector<int>.Zero))
                            {                                
                                var color = pixelShader.ParallelShade(Vector.ConvertToInt32(xValues), new Vector<int>((int)y),
                                                                                                  depth, w0, w1, w2, triIndex);
                                frameBuffer.SetPixelParallel(x, pixelNum, writeMask, depth, color);
                            }
                            /*
                            else
                            {
                                skippedChunks++;
                            }
                            */
                            steppedIn = true;
                        }
                        
                        else if (steppedIn) break; 
                        pixelNum+= SIMDcount;
                        xValues += new Vector<float> ( SIMDcount );
                    }
                }

                );
            //we ARE skipping chunks.
            //maybe a render to DB first...
            //we should have a lot of skipped chunks regardless of rotation, since back face is always occluded by front face...
            //Console.WriteLine($"skipped: {skippedChunks} chunks");
        }


        public RenderingPipeline(int width, int height, IPixelShader pixelShader, IVertexShader vertexShader)
        {
            this.width = width;
            this.height = height;
            this.pixelShader = pixelShader;
            this.vertexShader = vertexShader;           
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
