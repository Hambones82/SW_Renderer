using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;
using System.Diagnostics;
using System.Reflection.Metadata.Ecma335;
using System.Dynamic;

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

        public static void TestEdge(Vec3 a, Vec3 b, float y, ref Span<float> xs, ref int count)
        {
            float Ay = a.Y, By = b.Y;
            float yMin = Math.Min(Ay, By);
            float yMax = Math.Max(Ay, By);

            // Top-inclusive, bottom-exclusive
            if (y >= yMin && y < yMax && By != Ay)
            {
                float t = (y - Ay) / (By - Ay);
                xs[count++] = a.X + t * (b.X - a.X);
            }
        }

        public static bool TryGetSpanForScanline(float y, Vec3 v0, Vec3 v1, Vec3 v2,
                           out float spanMin, out float spanMax)
        {
            Span<float> xs = stackalloc float[3];            
            int count = 0;

            TestEdge(v0, v1, y, ref xs, ref count);
            TestEdge(v1, v2, y, ref xs, ref count);
            TestEdge(v2, v0, y, ref xs, ref count);

            if (count < 2)
            {
                spanMin = spanMax = 0;
                return false;
            }

            float a = xs[0];
            float b = xs[1];
            if (count == 3)
            {
                float c = xs[2];
                spanMin = Math.Min(a, Math.Min(b, c));
                spanMax = Math.Max(a, Math.Max(b, c));
            }
            else
            {
                spanMin = Math.Min(a, b);
                spanMax = Math.Max(a, b);
            }

            return true;
        }

        public struct RowLoopState 
        {
            public Vec3 v0, v1, v2;
            public int x0, x1, y0, y1;
        }
        
        public void Rasterize(Vec3 v0, Vec3 v1, Vec3 v2, int triIndex, int fbWidth, int fbHeight, FrameBuffer frameBuffer)
        {
            
            if(EventCounterLog.enabled)
            {
                EventCounterLog.Inc("tris_processed");
            }
            
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
                    var yVec = new Vector<int>((int)y);
                    float pY = (float)y + 0.5f;

                    int xi0, xi1;

                    if (TryGetSpanForScanline(pY, v0, v1, v2, out float xMin, out float xMax))
                    {
                        xi0 = Math.Max(x0, (int)Math.Ceiling(xMin));
                        xi1 = Math.Min(x1, (int)Math.Floor(xMax));                        
                    }
                    else
                    {
                        return;
                    }

                    int pixelNum = y * width + xi0;
                    
                    Span<float> vals = stackalloc float[SIMDcount];

                    for (int i = 0; i < SIMDcount; i++)
                    {
                        vals[i] = xi0 + i;
                    }

                    Vector<float> xValues = new Vector<float>(vals);
                    
                    Vector<float> w0Increment = new Vector<float>(v2.Y - v1.Y);                    
                    Vector<float> w0RHS = new Vector<float>((pY - v1.Y) * (v2.X - v1.X));

                    Vector<float> w1Increment = new Vector<float>(v0.Y - v2.Y);                    
                    Vector<float> w1RHS = new Vector<float>((pY - v2.Y) * (v0.X - v2.X));

                    Vector<float> w2Increment = new Vector<float>(v1.Y - v0.Y);                    
                    Vector<float> w2RHS = new Vector<float>((pY - v0.Y) * (v1.X - v0.X));
                    
                    Vector<float> w0Initial = (xValues + v1Xp5) * w0Increment - w0RHS;
                    Vector<float> w1Initial = (xValues + v2Xp5) * w1Increment - w1RHS;
                    Vector<float> w2Initial = (xValues + v0Xp5) * w2Increment - w2RHS;

                    var w0 = w0Initial;
                    var w1 = w1Initial;
                    var w2 = w2Initial;

                    var w0ScaledIncrement = w0Increment * SIMDcount;
                    var w1ScaledIncrement = w1Increment * SIMDcount;
                    var w2ScaledIncrement = w2Increment * SIMDcount;

                    var w0Bary = w0 / area;
                    var w1Bary = w1 / area;
                    var w2Bary = w2 / area;

                    var w0BaryIncrement = w0ScaledIncrement / area;
                    var w1BaryIncrement = w1ScaledIncrement / area;
                    var w2BaryIncrement = w2ScaledIncrement / area;

                    int x;

                    var depth = w0Bary * v0.Z + w1Bary * v1.Z + w2Bary * v2.Z;

                    var depthIncrement = w0BaryIncrement * v0.Z + w1BaryIncrement * v1.Z + w2BaryIncrement * v2.Z;

                    //stepped SIMD iterations, runs until a final iteration
                    for (x = xi0; x <= xi1; x+= SIMDcount)
                    {
                        if (x + SIMDcount >= xi1) { break; }
                        //if (pixelNum > frameBuffer._size) 
                        //{ 
                        //    Console.WriteLine($"pixel num is greater in SIMD"); 
                        //}
                        Vector<float> storedDepths = new Vector<float>(frameBuffer.depth, pixelNum);
                        Vector<int> depthComp = Vector.LessThanOrEqual(depth, storedDepths);
                        
                        if(!Vector.EqualsAll(depthComp, Vector<int>.Zero))
                        {
                            var color = pixelShader.ParallelShade(Vector.ConvertToInt32(xValues), 
                                                                    yVec, depth, w0Bary, w1Bary, w2Bary, triIndex);                            
                            frameBuffer.SetPixelParallel(x, pixelNum, depthComp, depth, color);
                        }
                        
                        //Vector<float> writeMaskFloat = Vector.AsVectorSingle(depthComp);
                        if(EventCounterLog.enabled)
                        {
                            int countFrags = 0;
                            for (int i = 0; i < SIMDcount; i++)
                            {
                                if (depthComp[i] == -1) countFrags++;
                            }
                            EventCounterLog.Inc("fragments_processed", countFrags);
                            EventCounterLog.Inc("fragments_visited", SIMDcount);
                        }
                        
                        
                        pixelNum+= SIMDcount;
                        xValues += new Vector<float> ( SIMDcount );
                        w0 += w0ScaledIncrement;
                        w1 += w1ScaledIncrement;
                        w2 += w2ScaledIncrement;

                        w0Bary += w0BaryIncrement;
                        w1Bary += w1BaryIncrement;
                        w2Bary += w2BaryIncrement;

                        depth += depthIncrement;
                    }

                    //THIS IS SIGNIFICANTLY SLOWER THAN WHEN IT WAS SIMD...  
                    //final, scalar iteration
                    //still need to check whether we are in the right row and right column - in bounds.  problem is
                    //at this point we've elided that check since the above for loop loops between the correct bounds.
                    
                    if(pixelNum >= 0 && pixelNum < frameBuffer._size)                    
                    {
                        //this can be optimized as well - don't need to do this by barycentric coords, just whether at end of line
                        var inside =
                        Vector.GreaterThanOrEqual(w0, Vector<float>.Zero) &
                        Vector.GreaterThanOrEqual(w1, Vector<float>.Zero) &
                        Vector.GreaterThanOrEqual(w2, Vector<float>.Zero);


                        if (!Vector.EqualsAll(inside, Vector<int>.Zero))
                        {
                            for(int i = 0; i < SIMDcount; i++)
                            //still need to check whether the depths themselves are all within...                              
                            //while (x < frameBuffer.width)
                            {
                                if (x >= frameBuffer.width)
                                {
                                    break;
                                }                                    
                                //Vector<float> storedDepths = new Vector<float>(frameBuffer.depth, pixelNum);
                                float depthScalarDest = frameBuffer.depth[pixelNum];
                                float depthScalarInc = depth.GetElement(i);
                                bool depthComp = depthScalarInc <= depthScalarDest;
                                var writeMask = (inside.GetElement(i) == -1) & depthComp;

                                /*
                                if (!Vector.EqualsAll(writeMask, Vector<int>.Zero))
                                {
                                    var color = pixelShader.ParallelShade(Vector.ConvertToInt32(xValues),
                                                                            yVec, depth, w0Bary, w1Bary, w2Bary, triIndex);
                                    frameBuffer.SetPixelParallel(x, pixelNum, writeMask, depth, color);
                                }*/
                                if (writeMask)
                                {
                                    var color = pixelShader.Shade(x, y, depthScalarInc, w0.GetElement(i),
                                                                  w1.GetElement(i), w2.GetElement(i), triIndex);
                                    frameBuffer.SetPixel(pixelNum, depthScalarInc, color);
                                }

                                /*
                                if (EventCounterLog.enabled)
                                {
                                    int countFrags = 0;
                                    for (int i = 0; i < SIMDcount; i++)
                                    {
                                        if (writeMask[i] == -1) countFrags++;
                                    }
                                    EventCounterLog.Inc("fragments_processed", countFrags);
                                    EventCounterLog.Inc("fragments_visited", countFrags);
                                }*/
                                x++;
                                pixelNum++;
                            }
                        }
                        

                        //pixelNum += SIMDcount;
                    }
                }
                );
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
