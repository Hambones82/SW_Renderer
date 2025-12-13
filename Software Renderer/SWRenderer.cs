using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;
using System.Diagnostics;
using System.Reflection.Metadata.Ecma335;
using System.Dynamic;
using System.Security.Cryptography.X509Certificates;

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
        public Vector4 Position;

        public VertexShaderOutput(Vector4 position)
        {
            Position = position;
        }
    }

    public struct SSTriangle
    {
        public Vector3 s0, s1, s2;
        public float area;

        // Depth plane: z = A*x + B*y + C
        public float A, B, C;

        // Optional: tri-wide min/max depth (useful for conservative bounds)
        public float minZ, maxZ;

        public Vector3 topLeftCoord;
        public float EvalDepth(float x, float y)
        {
            return A * x + B * y + C;
        }
        
        //need to set A, B, C...

        //CACHED SETUP INFO
        //SS Bbox
        public float minX;
        public float maxX;
        public float minY;
        public float maxY;

        //all values below are SIMD-wide values, and correspond to 0-SIMDWidth increments in x
        //these setup vaules are:
        //depth values (top left "initial" value and x and y gradients)
        //barycentric coords (top left "initial" value and x and y gradients)
        //edge equation values (top left "initial" and x and y gradients)

        //top left edge equation values
        public Vector<float> w0TL;
        public Vector<float> w1TL;
        public Vector<float> w2TL;

        //edge equation x gradients
        public float w0dx;
        public float w1dx;
        public float w2dx;

        //edge equation y gradients
        public float w0dy;
        public float w1dy;
        public float w2dy;

        //top left barycentric coords
        public Vector<float> w0BaryTL;
        public Vector<float> w1BaryTL;
        public Vector<float> w2BaryTL;

        //barycentric x gradients        
        public float w0Barydx;
        public float w1Barydx;
        public float w2Barydx;

        //barycentric y gradients        
        public float w0Barydy;
        public float w1Barydy;
        public float w2Barydy;

        //top left depth value
        public Vector<float> depthTL;

        //depth gradients
        public float depthdx;
        public float depthdy;        
    }

    public class RenderingPipeline
    {
        private const int maxBufferedTriangles = 10000;
        private int width;
        private int height;      
        private IPixelShader pixelShader;        
        private IVertexShader vertexShader;
        private int bufferedTriangleHead = 0;
        private int bufferedTriangleTail = 0;
        private SSTriangle[] bufferedSSTriangles = new SSTriangle[maxBufferedTriangles];

        private float EdgeFunction(Vector3 a, Vector3 b, Vector3 c)
        {
            return (c.X - a.X) * (b.Y - a.Y) - (c.Y - a.Y) * (b.X - a.X);
        }

        public static void TestEdge(Vector3 a, Vector3 b, float y, ref Span<float> xs, ref int count)
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

        public static bool TryGetSpanForScanline(float y, Vector3 v0, Vector3 v1, Vector3 v2,
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
        private bool PointsInsideHalfPlane(Vector3 edgeV1,  Vector3 edgeV2, Vector3 c0, 
                                           Vector3 c1,      Vector3 c2,     Vector3 c3)
        {
            if (EdgeFunction(edgeV1, edgeV2, c0) >= 0) return true;
            if (EdgeFunction(edgeV1, edgeV2, c1) >= 0) return true;
            if (EdgeFunction(edgeV1, edgeV2, c2) >= 0) return true;
            if (EdgeFunction(edgeV1, edgeV2, c3) >= 0) return true;
            //edge function is <0 for all four corners
            return false;
        }

        //passing in top left bary, x, y bary gradients, top left tri x, y, and the x,y's of the corners
        private bool InsideEdge(float wTL, float wdx, float wdy,
                    float refX, float refY, float cornerXMin, float cornerYMin,
                    float cornerXMax, float cornerYMax)
        {
            // pick the corner that minimizes w
            float tx = (wdx >= 0.0f) ? cornerXMin : cornerXMax;
            float ty = (wdy >= 0.0f) ? cornerYMin : cornerYMax;

            float wMin = wTL
                + wdx * (tx - refX)
                + wdy * (ty - refY);

            const float eps = -1e-5f; // small conservative bias
            return wMin >= eps;
        }

        bool TileFullyCoveredByTriangle(ref SSTriangle tri,
                                float x0, float y0, float x1, float y1)
        {            

            float refX = tri.topLeftCoord.X;
            float refY = tri.topLeftCoord.Y;

            if (!InsideEdge(tri.w0TL[0], tri.w0dx, tri.w0dy, refX, refY, x0, y0, x1, y1)) return false;
            if (!InsideEdge(tri.w1TL[0], tri.w1dx, tri.w1dy, refX, refY, x0, y0, x1, y1)) return false;
            if (!InsideEdge(tri.w2TL[0], tri.w2dx, tri.w2dy, refX, refY, x0, y0, x1, y1)) return false;

            return true; // all three edge half-spaces fully contain the tile
        }



        private bool TestTriInTile(Vector3 topLeft,     Vector3 topRight,   Vector3 bottomLeft, 
                                   Vector3 bottomRight, ref SSTriangle tri)
        {
            Vector3 c0 = topLeft;
            Vector3 c1 = topRight;
            Vector3 c2 = bottomRight;
            Vector3 c3 = bottomLeft;

            // If for ANY edge, all corners are outside (EdgeFunction < 0),
            // the triangle cannot overlap the tile -> reject.
            if (!PointsInsideHalfPlane(tri.s0, tri.s1, c0, c1, c2, c3)) return false;
            if (!PointsInsideHalfPlane(tri.s1, tri.s2, c0, c1, c2, c3)) return false;
            if (!PointsInsideHalfPlane(tri.s2, tri.s0, c0, c1, c2, c3)) return false;

            // If we didn't fail any edge, the tile may overlap the triangle
            return true;
        }
                            
        public void DebugDrawRect(Vector3 topLeft,      Vector3 topRight,   Vector3 bottomLeft,
                                  Vector3 bottomRight,  uint color,         FrameBuffer frameBuffer)
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
            //for(int i = 0; i < fb.numBins; i++)
            Parallel.For(0, fb.numBins, (i) =>
            {
                fb.BinXY(i, out int x, out int y);
                FlushBin(i, fb, x, y);
            });
            bufferedTriangleTail = 0;
        }
        
        public void FlushAllBins(FrameBuffer fb)
        {
            NewFrame(fb);
        }

        //i think we should align all operations to SIMD-width reads/writes.
        public void Rasterize(ref SSTriangle tri, FrameBuffer frameBuffer, 
                              uint xClipLow = uint.MinValue, uint xClipHigh = uint.MaxValue,
                              uint yClipLow = uint.MinValue, uint yClipHigh = uint.MaxValue)
        {
            Vector3 v0 = tri.s0;
            Vector3 v1 = tri.s1;
            Vector3 v2 = tri.s2;

            int fbWidth = frameBuffer.width;
            int fbHeight = frameBuffer.height;

            if(Logger.enabled)
            {
                Logger.Inc("tris_processed");
            }
            
            
            float minX = Math.Max(xClipLow, tri.minX);
            float minY = Math.Max(yClipLow, tri.minY);
            float maxX = Math.Min(xClipHigh, tri.maxX);
            float maxY = Math.Min(yClipHigh, tri.maxY);

            int x0 = Math.Max(0, (int)Math.Floor(minX));
            int x1 = Math.Min(fbWidth - 1, (int)Math.Ceiling(maxX));
            int y0 = Math.Max(0, (int)Math.Floor(minY));
            int y1 = Math.Min(fbHeight - 1, (int)Math.Ceiling(maxY));
            
            if (tri.area <= 0) return;
            //thsi is a constant
            int SIMDcount = Vector<float>.Count;
            
            Vector<float> w0TLBin = tri.w0TL 
                                + new Vector<float>((x0 - tri.topLeftCoord.X) * tri.w0dx)
                                + new Vector<float>((y0 - tri.topLeftCoord.Y) * tri.w0dy);
            Vector<float> w1TLBin = tri.w1TL
                                + new Vector<float>((x0 - tri.topLeftCoord.X) * tri.w1dx)
                                + new Vector<float>((y0 - tri.topLeftCoord.Y) * tri.w1dy);
            Vector<float> w2TLBin = tri.w2TL
                                + new Vector<float>((x0 - tri.topLeftCoord.X) * tri.w2dx)
                                + new Vector<float>((y0 - tri.topLeftCoord.Y) * tri.w2dy);

            //values for top-left pixel in bin
            //can adjust this to remove a division as this has already been done in triangle setup
            Vector<float> w0BaryTLBin = w0TLBin / tri.area;
            Vector<float> w1BaryTLBin = w1TLBin / tri.area;
            Vector<float> w2BaryTLBin = w2TLBin / tri.area;

            Vector<float> depthTLBin = tri.depthTL
                                    + new Vector<float>((x0 - tri.topLeftCoord.X) * tri.depthdx)
                                    + new Vector<float>((y0 - tri.topLeftCoord.Y) * tri.depthdy);

            for (int y = y0; y <= y1; y++)
            {
                Vector<float> w0 = w0TLBin + new Vector<float>((y - y0) * tri.w0dy);
                Vector<float> w1 = w1TLBin + new Vector<float>((y - y0) * tri.w1dy);
                Vector<float> w2 = w2TLBin + new Vector<float>((y - y0) * tri.w2dy);

                //same for bary, depth
                Vector<float> w0Bary = w0BaryTLBin + new Vector<float>((y - y0) * tri.w0Barydy);
                Vector<float> w1Bary = w1BaryTLBin + new Vector<float>((y - y0) * tri.w1Barydy);
                Vector<float> w2Bary = w2BaryTLBin + new Vector<float>((y - y0) * tri.w2Barydy);

                Vector<float> depth = depthTLBin + new Vector<float>((y - y0) * tri.depthdy);


                var yVec = new Vector<int>((int)y);
                float pY = (float)y + 0.5f;

                int xi0, xi1;

                //might want to elimiate this or just replace it with a "does this tile overlap tri" test
                if (TryGetSpanForScanline(pY, v0, v1, v2, out float xMin, out float xMax))
                {
                    xi0 = Math.Max(x0, (int)Math.Ceiling(xMin));
                    xi1 = Math.Min(x1, (int)Math.Floor(xMax));                        
                }
                else
                {
                    continue;
                }

                //this just aligns the initial pixel to the SIMD boundary.  
                //it over-draws in the left direction, though, so we need to mask that out.
                //we don't need to do it in the simd-tail way, because there's no way the initial set of values will underflow
                int origxi0 = xi0;
                int maskedInitialPixels = xi0 % SIMDcount;
                xi0 -= maskedInitialPixels;

                //instead of this, we want to begin at the SIMD-alignment boundary.  so maybe just adjust xi0 to match that boundary
                //then just use it as normal.
                int pixelNum = y * width + xi0;
                
                Vector<float> xValues = new Vector<float>(xi0) + Constants.SIMDIncrement;

                int x;

                //stepped SIMD iterations, runs until a final iteration

                float horizontalOffset = xi0 - x0;

                w0 += new Vector<float>(tri.w0dx * horizontalOffset);
                w1 += new Vector<float>(tri.w1dx * horizontalOffset);
                w2 += new Vector<float>(tri.w2dx * horizontalOffset);

                w0Bary += new Vector<float>(tri.w0Barydx * horizontalOffset);
                w1Bary += new Vector<float>(tri.w1Barydx * horizontalOffset);
                w2Bary += new Vector<float>(tri.w2Barydx * horizontalOffset);

                depth += new Vector<float>(tri.depthdx * horizontalOffset);

                Vector<int> initialMask = Vector.GreaterThanOrEqual(Vector<int>.Indices,
                    new Vector<int>(maskedInitialPixels));//we need a mask that masks out the initial

                const bool renderSIMD = true;                
                for (x = xi0; x <= xi1; x += SIMDcount)
                {
                    if (x + SIMDcount > xi1 + 1) { break; }
                    Vector<float> storedDepths = new Vector<float>(frameBuffer.depth, pixelNum);
                    Vector<int> depthComp = Vector.LessThanOrEqual(depth, storedDepths);
                    Vector<int> mask = depthComp & initialMask;
                    initialMask = new Vector<int>(-1);
                    if (!Vector.EqualsAll(depthComp, Vector<int>.Zero) && renderSIMD)
                    {
                        var color = pixelShader.ParallelShade(Vector.ConvertToInt32(xValues),
                                                                yVec, depth, w0Bary, w1Bary, w2Bary);
                        frameBuffer.SetPixelParallel(x, pixelNum, mask, depth, color);
                    }

                    pixelNum += SIMDcount;
                    xValues += new Vector<float>(SIMDcount);
                    w0 += new Vector<float>(tri.w0dx * SIMDcount);
                    w1 += new Vector<float>(tri.w1dx * SIMDcount);
                    w2 += new Vector<float>(tri.w2dx * SIMDcount);

                    w0Bary += new Vector<float>(tri.w0Barydx * SIMDcount);
                    w1Bary += new Vector<float>(tri.w1Barydx * SIMDcount);
                    w2Bary += new Vector<float>(tri.w2Barydx * SIMDcount);

                    depth += new Vector<float>(tri.depthdx * SIMDcount);
                }
                
                const bool renderScalarTail = true;

                if(pixelNum >= 0 && pixelNum < frameBuffer._size && renderScalarTail)                    
                {                                        
                    for(int i = 0; i < SIMDcount; i++)                    
                    {
                        //this if statement is for if the triangle spans entirely within one SIMD width (in which case the initial
                        //pixels must be masked out
                        if(x < origxi0)
                        {
                            pixelNum++;
                            x++;
                            continue;
                        }
                        //this if statement is to prevent overflow past the span or the frame buffer width
                        if (x >= frameBuffer.width || x>xi1)
                        {
                            break;
                        }    
                        float depthScalarDest = frameBuffer.depth[pixelNum];
                        float depthScalarInc = depth.GetElement(i);
                        bool depthComp = depthScalarInc <= depthScalarDest;
                                
                        if (depthComp)
                        {
                        var color = pixelShader.Shade(x, y, depthScalarInc, w0.GetElement(i),
                                                        w1.GetElement(i), w2.GetElement(i));                            
                            frameBuffer.SetPixel(pixelNum, depthScalarInc, color);                                
                        }

                        x++;
                        pixelNum++;
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
        }


        public void GetBinCorners(int bx, int by,           out Vector3 topLeft,        out Vector3 topRight, 
                                  out Vector3 bottomLeft,   out Vector3 bottomRight,    FrameBuffer fb)
        {
            int tileSize = FrameBuffer.binDimension;
            int tileXmin = bx * tileSize;
            int tileXmax = tileXmin + tileSize - 1;
            int tileYmin = by * tileSize;
            int tileYmax = tileYmin + tileSize - 1;
            topLeft = new Vector3(tileXmin, tileYmin, 0);
            bottomLeft = new Vector3(tileXmin, tileYmax, 0);
            topRight = new Vector3(tileXmax, tileYmin, 0);
            bottomRight = new Vector3(tileXmax, tileYmax, 0);
        }

        private void FlushBin(int binIndex, FrameBuffer framebuffer, int bx, int by)
        {
            ref Bin bin = ref framebuffer.bins[binIndex];
            Vector3 binTopLeft;
            Vector3 binTopRight;
            Vector3 binBottomLeft;
            Vector3 binBottomRight;
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
                    //EventCounterLog.Inc("skip");
                    continue;
                }
                else
                {
                    //EventCounterLog.Inc("no-skip");
                }

                    Rasterize(ref tri, framebuffer,
                        (uint)binTopLeft.X, (uint)binTopRight.X, (uint)binTopLeft.Y, (uint)binBottomRight.Y);
            }
            bin.Clear();
        }

        public void CalculateBinTriDepths(ref SSTriangle currentTri, int bx, int by, 
                                          FrameBuffer frameBuffer, out float tileTriMinZ, out float tileTriMaxZ)
        {
            GetBinCorners(bx, by,   out Vector3 topLeft,        out Vector3 topRight, out Vector3 bottomLeft, 
                                    out Vector3 bottomRight,    frameBuffer);            

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

        //I THINK THERE'S A SORT OF PRECISION BUG IN SOME OF THESE CALCULATIONS
        //WHAT I MEAN IS THAT THE LEFT MOST X FOR TRIANGLES ARE JUST THE ACTUAL X VALUE OF THE LEFT-MOST VERTEX
        //BUT THIS CAN BE PROBLEMATIC IF TRYING TO ADD 1 TO THIS VALUE TO GET RELATED VALUES FOR SUBSEQUENT PIXELS
        //THIS MIGHT BE THE REASON FOR THE RIGHT-MOST 
        //returns true if triangle accepted, false if triangle rejected
        public bool TriangleSetup(ref SSTriangle tri)
        {
            Vector3 s0 = tri.s0;
            Vector3 s1 = tri.s1;
            Vector3 s2 = tri.s2;

            tri.minX = MathF.Min(s0.X, MathF.Min(s1.X, s2.X));
            tri.maxX = MathF.Max(s0.X, MathF.Max(s1.X, s2.X));
            tri.minY = MathF.Min(s0.Y, MathF.Min(s1.Y, s2.Y));
            tri.maxY = MathF.Max(s0.Y, MathF.Max(s1.Y, s2.Y));

            tri.minX = MathF.Max(0, tri.minX);
            tri.minY = MathF.Max(0, tri.minY);
            tri.maxX = MathF.Min(width - 1, tri.maxX);
            tri.maxY = MathF.Min(height - 1, tri.maxY);

            tri.topLeftCoord = new Vector3(tri.minX, tri.minY, 0);

            tri.area = EdgeFunction(s0, s1, s2);

            if (tri.minX > tri.maxX || tri.minY > tri.maxY || tri.area <= 0)
            {
                return false;
            }

            //plane equation, z calcs
            //but the z's are...  need to check what the code in rasterize() is doing - where is it calculating the min/max for? 
            //min/max z doesn't have to be top left corner but top left corner is necessary for gradient-based depth calc.
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
                return false;
            }

            // A and B come from solving the linear system; C from one vertex
            tri.A =
                (z0 * (y1 - y2) + z1 * (y2 - y0) + z2 * (y0 - y1)) / denom;

            tri.B =
                (z0 * (x2 - x1) + z1 * (x0 - x2) + z2 * (x1 - x0)) / denom;

            tri.C = z0 - tri.A * x0 - tri.B * y0;

            //!!!WHAT PURPOSES DO THESE VALUES SERVE???
            // Tri-wide depth bounds (not tile-specific, but good for conservative use)
            tri.minZ = MathF.Min(z0, MathF.Min(z1, z2));
            tri.maxZ = MathF.Max(z0, MathF.Max(z1, z2));
            //--END CALCULATE MAX/MIN DEPTH


            //all values below are SIMD-wide values, and correspond to 0-SIMDWidth increments in x
            //these setup vaules are:
            //depth values (top left "initial" value and x and y gradients)
            //barycentric coords (top left "initial" value and x and y gradients)
            //edge equation values (top left "initial" and x and y gradients)

            //Vector<float> SIMDWidth = new Vector<float>(Vector<float>.Count);
            float SIMDWidth = Vector<float>.Count;

            //edge equation x gradients
            tri.w0dx = (s2.Y - s1.Y); // = new Vector<float>((s2.Y - s1.Y));// * SIMDWidth);
            tri.w1dx = (s0.Y - s2.Y); // = new Vector<float>((s0.Y - s2.Y));// * SIMDWidth);
            tri.w2dx = (s1.Y - s0.Y); // = new Vector<float>((s1.Y - s0.Y));// * SIMDWidth);

            //edge equation y gradients
            tri.w0dy = s1.X - s2.X;
            tri.w1dy = s2.X - s0.X;
            tri.w2dy = s0.X - s1.X;

            //top left edge equation values
            tri.w0TL = new Vector<float>(EdgeFunction(s1, s2, tri.topLeftCoord)) + Constants.SIMDIncrement * tri.w0dx;
            tri.w1TL = new Vector<float>(EdgeFunction(s2, s0, tri.topLeftCoord)) + Constants.SIMDIncrement * tri.w1dx;
            tri.w2TL = new Vector<float>(EdgeFunction(s0, s1, tri.topLeftCoord)) + Constants.SIMDIncrement * tri.w2dx;

            //top left barycentric coords
            tri.w0BaryTL = tri.w0TL /tri.area;
            tri.w1BaryTL = tri.w1TL / tri.area;
            tri.w2BaryTL = tri.w2TL / tri.area;

            //barycentric x gradients        
            tri.w0Barydx = tri.w0dx / tri.area;
            tri.w1Barydx = tri.w1dx / tri.area;
            tri.w2Barydx = tri.w2dx / tri.area;

            //barycentric y gradients        
            tri.w0Barydy = tri.w0dy / tri.area;
            tri.w1Barydy = tri.w1dy / tri.area;
            tri.w2Barydy = tri.w2dy / tri.area;

            //depth gradients
            tri.depthdx = tri.A;// * SIMDWidth);
            tri.depthdy = tri.B;

            //top left depth value
            tri.depthTL = new Vector<float>(tri.EvalDepth(tri.minX, tri.minY)) + Constants.SIMDIncrement * tri.depthdx;

            return true;
        }

        public void RenderMesh(Mesh mesh, FrameBuffer framebuffer)
        {            
            int tilesX = framebuffer.binsX;
            int tilesY = framebuffer.binsY;
            int tileW = FrameBuffer.binDimension;
            int tileH = FrameBuffer.binDimension;

            //problem is... we might end up storing more triangles into the buffered[] thing than its sized for
            //this can happen because we are going through all triangles of a mesh 
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

                Vector3 s0 = ViewportTransform(v0.Position);
                Vector3 s1 = ViewportTransform(v1.Position);
                Vector3 s2 = ViewportTransform(v2.Position);

                // Store in buffered screen-space triangle buffer
                int triBufIndex = bufferedTriangleTail;
                ref SSTriangle currentTri = ref bufferedSSTriangles[triBufIndex];
                currentTri.s0 = s0;
                currentTri.s1 = s1;
                currentTri.s2 = s2;                

                bufferedTriangleTail++;

                bool keepTri = TriangleSetup(ref currentTri);

                if(!keepTri)
                {
                    bufferedTriangleTail--;
                    continue;
                }                

                //i'd like to analytically calculate the bin start and bin end
                // Convert bbox to tile coordinates
                int binX0 = (int)(currentTri.minX / tileW);
                int binX1 = (int)(currentTri.maxX / tileW);
                int binY0 = (int)(currentTri.minY / tileH);
                int binY1 = (int)(currentTri.maxY / tileH);

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

                        //float tileX0 = bx * tileW;
                        //float tileY0 = by * tileH;
                        //float tileX1 = tileX0 + tileW;
                        //float tileY1 = tileY0 + tileH;

                        // clamp to framebuffer edges if needed
                        //if (tileX1 > width) tileX1 = width;
                        //if (tileY1 > height) tileY1 = height;

                        CalculateBinTriDepths(ref currentTri, bx, by, framebuffer, out float tileTriMinZ, out float tileTriMaxZ);

                        //I DONT THINK WE CARE ABT THIS...
                        //framebuffer.tileMinDepth[binIndex] =
                        //    MathF.Min(framebuffer.tileMinDepth[binIndex], tileTriMinZ);

                        //this should only be updated if the tile takes up the whole tile area...
                        //so if covers all...
                        //i don't think this is a great way of doing things as small triangles aggregated will never fully cover
                        //a tile
                        //let's get rid of this for now and try a different approach.
                        //if(TileFullyCoveredByTriangle(ref currentTri, tileX0, tileY0, tileX1, tileY1))
                        //{                        
                            framebuffer.tileMaxDepth[binIndex] =
                                MathF.Min(framebuffer.tileMaxDepth[binIndex], tileTriMaxZ);
                        //}
                        

                        ref Bin bin = ref framebuffer.bins[binIndex];

                        bin.triIndices[bin.tail++] = triBufIndex;

                        // If bin is full, flush then reuse it
                        if (bin.tail >= Bin.triangleBufferSize)
                        {
                            FlushAllBins(framebuffer);
                        }
                    }
                }
                if (bufferedTriangleTail >= maxBufferedTriangles)
                {
                    FlushAllBins(framebuffer);
                }                
            }
        }
        private VertexShaderOutput PerspectiveDivide(VertexShaderOutput vertex)
        {
            return new VertexShaderOutput(new Vector4(
                vertex.Position.X / vertex.Position.W,
                vertex.Position.Y / vertex.Position.W,
                vertex.Position.Z / vertex.Position.W,
                1
            ));
        }

        private Vector3 ViewportTransform(Vector4 ndc)
        {
            return new Vector3(
                (ndc.X + 1) * width / 2,
                (1 - ndc.Y) * height / 2,
                ndc.Z
            );
        }
    }
}
