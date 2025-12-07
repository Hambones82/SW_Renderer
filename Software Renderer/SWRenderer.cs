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

    public struct SSTriangle
    {
        public Vec3 s0, s1, s2;
        public float area;

        // Depth plane: z = A*x + B*y + C
        public float A, B, C;

        // Optional: tri-wide min/max depth (useful for conservative bounds)
        public float minZ, maxZ;

        public float EvalDepth(float x, float y)
        {
            return A * x + B * y + C;
        }

        //need to set A, B, C...
    }

    public class RenderingPipeline
    {
        private const int maxBufferedTriangles = 200;
        private int width;
        private int height;      
        private IPixelShader pixelShader;        
        private IVertexShader vertexShader;
        private int bufferedTriangleHead = 0;
        private int bufferedTriangleTail = 0;
        private SSTriangle[] bufferedSSTriangles = new SSTriangle[maxBufferedTriangles];

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

        
        //this is for a bbox corner reject
        //if the edge function for the edge evaluates as <0 for all corners, then the point is not inside the half plane
        //if the edge function evaluates as >= 0 for any corner, then the point is inside the half plane
        private bool PointsInsideHalfPlane(Vec3 edgeV1, Vec3 edgeV2, Vec3 c0, Vec3 c1, Vec3 c2, Vec3 c3)
        {
            if (EdgeFunction(edgeV1, edgeV2, c0) >= 0) return true;
            if (EdgeFunction(edgeV1, edgeV2, c1) >= 0) return true;
            if (EdgeFunction(edgeV1, edgeV2, c2) >= 0) return true;
            if (EdgeFunction(edgeV1, edgeV2, c3) >= 0) return true;
            //edge function is <0 for all four corners
            return false;
        }

        private bool TestTriInTile(Vec3 topLeft, Vec3 topRight, Vec3 bottomLeft, Vec3 bottomRight, 
                                    ref SSTriangle tri)
        {
            Vec3 c0 = topLeft;
            Vec3 c1 = topRight;
            Vec3 c2 = bottomRight;  // <- use bottomRight here
            Vec3 c3 = bottomLeft;

            // If for ANY edge, all corners are outside (EdgeFunction < 0),
            // the triangle cannot overlap the tile → reject.
            if (!PointsInsideHalfPlane(tri.s0, tri.s1, c0, c1, c2, c3)) return false;
            if (!PointsInsideHalfPlane(tri.s1, tri.s2, c0, c1, c2, c3)) return false;
            if (!PointsInsideHalfPlane(tri.s2, tri.s0, c0, c1, c2, c3)) return false;

            // If we didn't fail any edge, the tile may overlap the triangle
            return true;
        }
                            
        public void DebugDrawRect(Vec3 topLeft, Vec3 topRight, Vec3 bottomLeft, Vec3 bottomRight, uint color, FrameBuffer frameBuffer)
        {
            for(int x = (int)topLeft.X; x <= (int)topRight.X; x++)
            {
                for(int y = (int)topLeft.Y; y <= (int)bottomLeft.Y; y++)
                {
                    frameBuffer.SetPixel(x, y, color);
                }
            }         
        }
          
        
        public void NewFrame(FrameBuffer fb)
        {
            for(int i = 0; i < fb.numBins; i++)
            //Parallel.For(0, fb.numBins, (i) =>
            {
                fb.BinXY(i, out int x, out int y);
                FlushBin(i, fb, x, y);
            }
            bufferedTriangleTail = 0;
        }
        
        public void FlushAllBins(FrameBuffer fb)
        {
            NewFrame(fb);
        }

        public void Rasterize(Vec3 v0, Vec3 v1, Vec3 v2, int triIndex, int fbWidth, int fbHeight, FrameBuffer frameBuffer, 
                              uint xClipLow = uint.MinValue, uint xClipHigh = uint.MaxValue,
                              uint yClipLow = uint.MinValue, uint yClipHigh = uint.MaxValue)
        {
            
            if(EventCounterLog.enabled)
            {
                EventCounterLog.Inc("tris_processed");
            }
            
            float minX = Math.Min(v0.X, Math.Min(v1.X, v2.X));
            float maxX = Math.Max(v0.X, Math.Max(v1.X, v2.X));
            float minY = Math.Min(v0.Y, Math.Min(v1.Y, v2.Y));
            float maxY = Math.Max(v0.Y, Math.Max(v1.Y, v2.Y));

            minX = Math.Max(xClipLow, minX);
            minY = Math.Max(yClipLow, minY);
            maxX = Math.Min(xClipHigh, maxX);
            maxY = Math.Min(yClipHigh, maxY);

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

            //Parallel.For(y0, y1 + 1, 
            //    (y) => 
            for(int y = y0; y <= y1; y++)
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
                        continue;
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
        }


        public RenderingPipeline(int width, int height, IPixelShader pixelShader, IVertexShader vertexShader)
        {
            this.width = width;
            this.height = height;
            this.pixelShader = pixelShader;
            this.vertexShader = vertexShader;           
        }

        /*
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
                
                //store in each touching bin

                //when any bin has 100, set a rasterization flag and start rasterization after this triangle is placed in all its bins


                Rasterize(s0, s1, s2, i, width, height, framebuffer);
            }
        }
        */

        public void GetBinCorners(int bx, int by, out Vec3 topLeft, out Vec3 topRight, out Vec3 bottomLeft, out Vec3 bottomRight, 
                                   FrameBuffer fb)
        {
            int tileSize = FrameBuffer.binDimension;
            int tileXmin = bx * tileSize;
            int tileXmax = tileXmin + tileSize - 1;
            int tileYmin = by * tileSize;
            int tileYmax = tileYmin + tileSize - 1;
            topLeft = new Vec3(tileXmin, tileYmin, 0);
            bottomLeft = new Vec3(tileXmin, tileYmax, 0);
            topRight = new Vec3(tileXmax, tileYmin, 0);
            bottomRight = new Vec3(tileXmax, tileYmax, 0);
        }

        private void FlushBin(int binIndex, FrameBuffer framebuffer, int bx, int by)
        {
            ref Bin bin = ref framebuffer.bins[binIndex];
            Vec3 binTopLeft;
            Vec3 binTopRight;
            Vec3 binBottomLeft;
            Vec3 binBottomRight;
            GetBinCorners(bx, by, out binTopLeft, out binTopRight, out binBottomLeft, out binBottomRight, framebuffer);            
            for(int i = 0; i < bin.tail; i++)
            {                
                int triIndex = bin.triIndices[i];
                ref SSTriangle tri = ref bufferedSSTriangles[triIndex];
                CalculateBinTriDepths(ref tri, bx, by, framebuffer, out float tileTriMinZ, out float tileTriMaxZ);
                
                //very confused...  first of all why is it greater than tile min depth...
                //i think we really want like...  the minimum high depth seen for that tile
                if (tileTriMinZ > framebuffer.tileMaxDepth[binIndex])
                {
                    //Console.WriteLine("skipping tile");
                    continue;
                }
                    
                Rasterize(tri.s0, tri.s1, tri.s2, triIndex, framebuffer.width, framebuffer.height, framebuffer, 
                    (uint)binTopLeft.X, (uint)binTopRight.X, (uint)binTopLeft.Y, (uint)binBottomRight.Y);
            }
            bin.Clear();
        }

        public void CalculateBinTriDepths(ref SSTriangle currentTri, int bx, int by, 
                                          FrameBuffer frameBuffer, out float tileTriMinZ, out float tileTriMaxZ)
        {
            GetBinCorners(bx, by, out Vec3 topLeft, out Vec3 topRight, out Vec3 bottomLeft, out Vec3 bottomRight, frameBuffer);            

            float zTL = currentTri.EvalDepth(topLeft.X, topLeft.Y);
            float zTR = currentTri.EvalDepth(topRight.X, topRight.Y);
            float zBL = currentTri.EvalDepth(bottomLeft.X, bottomLeft.Y);
            float zBR = currentTri.EvalDepth(bottomRight.X, bottomRight.Y);

            tileTriMinZ = MathF.Min(
                currentTri.minZ,
                MathF.Min(MathF.Min(zTL, zTR), MathF.Min(zBL, zBR))
            );

            tileTriMaxZ = MathF.Max(
                currentTri.maxZ,
                MathF.Max(MathF.Max(zTL, zTR), MathF.Max(zBL, zBR))
            );
        }

        public void RenderMesh(Mesh mesh, FrameBuffer framebuffer)
        {
            int maxBuffered = bufferedSSTriangles.Length;
            int tilesX = framebuffer.binsX;
            int tilesY = framebuffer.binsY;
            int tileW = FrameBuffer.binDimension;
            int tileH = FrameBuffer.binDimension;

            for (int i = 0; i < mesh.Triangles.Length; i++)
            {
                Triangle tri = mesh.Triangles[i];

                // Vertex processing
                VertexShaderOutput v0 = vertexShader.VertexShade(tri.V0);
                VertexShaderOutput v1 = vertexShader.VertexShade(tri.V1);
                VertexShaderOutput v2 = vertexShader.VertexShade(tri.V2);

                v0 = PerspectiveDivide(v0);
                v1 = PerspectiveDivide(v1);
                v2 = PerspectiveDivide(v2);

                Vec3 s0 = ViewportTransform(v0.Position);
                Vec3 s1 = ViewportTransform(v1.Position);
                Vec3 s2 = ViewportTransform(v2.Position);

                // Store in buffered screen-space triangle buffer
                int triBufIndex = bufferedTriangleTail;
                ref SSTriangle currentTri = ref bufferedSSTriangles[triBufIndex];
                bufferedSSTriangles[triBufIndex].s0 = s0;
                bufferedSSTriangles[triBufIndex].s1 = s1;
                bufferedSSTriangles[triBufIndex].s2 = s2;

                //currentTri.

                bufferedTriangleTail++;

                // Compute screen-space bounds for binning
                float minX = MathF.Min(s0.X, MathF.Min(s1.X, s2.X));
                float maxX = MathF.Max(s0.X, MathF.Max(s1.X, s2.X));
                float minY = MathF.Min(s0.Y, MathF.Min(s1.Y, s2.Y));
                float maxY = MathF.Max(s0.Y, MathF.Max(s1.Y, s2.Y));

                // Clip to framebuffer
                minX = MathF.Max(0, minX);
                minY = MathF.Max(0, minY);
                maxX = MathF.Min(width - 1, maxX);
                maxY = MathF.Min(height - 1, maxY);

                currentTri.area = EdgeFunction(s0, s1, s2);

                // If triangle is completely off-screen, or if it's a back face, skip binning it
                if (minX > maxX || minY > maxY || currentTri.area <= 0)
                {
                    // Optionally roll back bufferedTriangleTail if you want to reuse that slot
                    bufferedTriangleTail--;
                    continue;
                }

                //--CALCULATE MAX/MIN DEPTH
                // --- depth plane coefficients ---
                // Solve for z = A*x + B*y + C using the three vertices
                float x0 = s0.X, y0 = s0.Y, z0 = s0.Z;
                float x1 = s1.X, y1 = s1.Y, z1 = s1.Z;
                float x2 = s2.X, y2 = s2.Y, z2 = s2.Z;

                // Denominator from the 2D area
                float denom = x0 * (y1 - y2) + x1 * (y2 - y0) + x2 * (y0 - y1);

                // Guard against numerical degeneracy
                if (MathF.Abs(denom) < 1e-6f)
                {
                    bufferedTriangleTail--;
                    continue;
                }

                // A and B come from solving the linear system; C from one vertex
                currentTri.A =
                    (z0 * (y1 - y2) + z1 * (y2 - y0) + z2 * (y0 - y1)) / denom;

                currentTri.B =
                    (z0 * (x2 - x1) + z1 * (x0 - x2) + z2 * (x1 - x0)) / denom;

                currentTri.C = z0 - currentTri.A * x0 - currentTri.B * y0;

                // Tri-wide depth bounds (not tile-specific, but good for conservative use)
                currentTri.minZ = MathF.Min(z0, MathF.Min(z1, z2));
                currentTri.maxZ = MathF.Max(z0, MathF.Max(z1, z2));
                //--END CALCULATE MAX/MIN DEPTH


                //i'd like to analytically calculate the bin start and bin end
                // Convert bbox to tile coordinates
                int binX0 = (int)(minX / tileW);
                int binX1 = (int)(maxX / tileW);
                int binY0 = (int)(minY / tileH);
                int binY1 = (int)(maxY / tileH);

                // Clamp to tile grid
                if (binX0 < 0) binX0 = 0;
                if (binY0 < 0) binY0 = 0;
                if (binX1 >= tilesX) binX1 = tilesX - 1;
                if (binY1 >= tilesY) binY1 = tilesY - 1;
                    

                    
                // Put this triangle into all overlapping bins
                for (int by = binY0; by <= binY1; by++)
                {
                    for (int bx = binX0; bx <= binX1; bx++)
                    {                        
                        int binIndex = by * tilesX + bx;

                        CalculateBinTriDepths(ref currentTri, bx, by, framebuffer, out float tileTriMinZ, out float tileTriMaxZ);

                        // Update global per-tile min/max across all tris
                        framebuffer.tileMinDepth[binIndex] =
                            MathF.Min(framebuffer.tileMinDepth[binIndex], tileTriMinZ);

                        framebuffer.tileMaxDepth[binIndex] =
                            MathF.Min(framebuffer.tileMaxDepth[binIndex], tileTriMaxZ);

                        ref Bin bin = ref framebuffer.bins[binIndex];

                        bin.triIndices[bin.tail++] = triBufIndex;

                        if(bufferedTriangleTail >= maxBufferedTriangles)
                        {
                            FlushAllBins(framebuffer);
                        }
                        // If bin is full, flush then reuse it
                        if (bin.tail >= Bin.triangleBufferSize)
                        {
                            FlushBin(binIndex, framebuffer, bx, by);
                        }
                    }
                }
                    
                

                // If we filled the buffered tri array, flush everything
                //if (bufferedTriangleTail >= maxBuffered)
                //{
                //    FlushAllBins(framebuffer);
                //}

                // NOTE: For now, the per-triangle Rasterize call is gone; it happens via FlushBin/FlushAllBins.
                // Old:
                // Rasterize(s0, s1, s2, i, width, height, framebuffer);
            }

            // After finishing the mesh, flush any remaining triangles
            //FlushAllBins(framebuffer);
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

    /*
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

        public void NewFrame(FrameBuffer fb)
        {

        }
    }
    */
}
