using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Net.NetworkInformation;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Software_Renderer
{
    public struct Bin
    {
        public const int triangleBufferSize = 10000;
        public int[] triIndices;
        //head is where you read from, tail is where you write to
        public int head = 0, tail = 0;
        public Bin() 
        { 
            triIndices = new int[triangleBufferSize];
        }

        public void Clear()
        {
            head = 0; tail = 0;
        }
    }

    public class FrameBuffer
    {
        public int width;
        public int height;
        public uint[] pixels;
        public float[] depth;
        public Int64[] coverage;//a per-element coverage bitmask
        public Int64[] tileCoverage; //a bitmask, each of whose bits represents one element of ^coverage
                                     //bit is 1 if all are covered, 0 otherwise
                                     //this means that if tileCoverage[tilenum] == -1, the entire tile is covered.
        public readonly int _size;
        public readonly int numBins;
        public readonly int binsX;
        public readonly int binsY;
        public const int pixelInitialValue = 0;
        public const float depthInitialValue = float.MaxValue;

        //bins are triangle buffer size, as well as _size in dimension (_size x _size)

        public static int binDimension = 64;
        public float[] tileMinDepth; // length = numBins
        public float[] tileMaxDepth; // length = numBins
        public Bin[] bins;

        

        public void BinXY(int binNum, out int x, out int y)
        {
            x = binNum % binsX;
            y = binNum / binsX;
        }

        //note: WithElement<T>(Vector<T>, Int32, T) --> can set a specific element...
        public FrameBuffer(int width, int height)
        {
            this.width = width;
            this.height = height;
            numBins = width * height / (binDimension * binDimension);
            _size = width * height;
            pixels = new uint[_size];
            depth = new float[_size];
            //a bitmask with MSB = lower x and LSB = higher x
            coverage = new Int64[_size/64 + 1];
            tileCoverage = new long[_size / 4096];

            bins = new Bin[numBins];
            for(int i = 0; i < numBins; i++)
            {
                bins[i] = new Bin();
            }
            tileMinDepth = new float[numBins];
            tileMaxDepth = new float[numBins];
            binsX = width / binDimension; 
            binsY = height / binDimension;
        }

        public void SetPixel(int x, int y, uint color)
        {            
            pixels[width*y + x] = color;            
        }

        public void SetDepth(int x, int y, float inDepth)
        {
            depth[width * y + x] = inDepth;
        }

        public void SetCoverage(int x, int y)
        {
            SetCoverage(width * y + x);
        }

        public void SetCoverage(int pixelNum)
        {
            int wordNum = pixelNum / 64;
            int pixelOffset = 63 - (pixelNum % 64);
            Int64 mask = 1L << pixelOffset;
            Int64 newCoverage = coverage[wordNum] | mask;
            SetCoverage64(pixelNum, newCoverage);
            //coverage[wordNum] |= mask;
        }

        //the byte cannot cross a 64 bit aligned boundary
        //the pixelNum must be 8-bit aligned
        public void SetCoverage8(int pixelNum, byte coverage)
        {
            Debug.Assert((pixelNum % 8) == 0, "SetCoverage8 requires pixelNum % 8 == 0");
            Debug.Assert((pixelNum % 64) <= 56, "SetCoverage8 cannot cross 64-bit boundary");            
            int wordnum = pixelNum / 64;
            int pixelOffset = 63 - (pixelNum % 64);            
            Int64 mask = (Int64)(coverage) << (pixelOffset - 7);
            Int64 newCoverage64 = this.coverage[wordnum] | mask;
            //this.coverage[wordnum] |= mask;
            SetCoverage64(pixelNum, newCoverage64);
        }

        public Int64 SetBit(Int64 target, int bitNum)
        {            
            Int64 mask = 1L << (63 - bitNum);
            return target | mask;
        }

        public void SetCoverage64(int pixelNum, Int64 coverage)
        {
            int wordnum = pixelNum / 64;
            this.coverage[wordnum] |= coverage;
            //get tile num from pixel num
            //convert pixelnum to x, y
            
            //i... don't think this is right?
            if(coverage == -1)
            {
                int tileWidth = width / 64;
                int x = pixelNum % width;
                int y = pixelNum / width;
                int tileNumOffset = x / 64;
                int tileNum = y / 64 * tileWidth + tileNumOffset;
                int yOffset = y % 64;
                tileCoverage[tileNum] = SetBit(tileCoverage[tileNum], yOffset);
            }            
        }

        public bool GetCoverage(int pixelNum)
        {
            int wordnum = pixelNum / 64;
            int pixelOffset = 63 - (pixelNum % 64);
            Int64 mask = 1L << pixelOffset;
            return ((coverage[wordnum] & mask) != 0);
        }

        //true if fully covered, false otherwise
        public bool GetTileCoverage(int tileNum)
        {
            return (tileCoverage[tileNum] == -1);
        }

        public void SetPixel(int x, int y, float inDepth, uint color)
        {
            SetPixel(width * y + x, inDepth, color);
            //pixels[width * y + x] = color;
            //depth[width * y + x] = inDepth;
            //coverage
        }

        public void SetPixel(int pixelNum, float inDepth, uint color)
        {
            pixels[pixelNum] = color;
            depth[pixelNum] = inDepth;
            SetCoverage(pixelNum);
        }
        
        //also set coverage mask...
        public void SetPixelParallel(int xStart, int pixelNum, Vector<int> mask, Vector<float> inDepth, Vector<uint> color)
        {            
            Debug.Assert((pixelNum % width) == xStart, $"pixelNum/xStart mismatch");
            int SIMDSize = Vector<float>.Count;
            if (pixelNum >= _size) return;            
            if(xStart + SIMDSize <= width)
            {
                Vector<uint> uIntMask = Vector.AsVectorUInt32(mask);
                
                Vector<uint> destPixels = new Vector<uint>(pixels, pixelNum);
                var comboPixels = Vector.ConditionalSelect(uIntMask, color, destPixels);

                Vector<float> destDepths = new Vector<float>(depth, pixelNum);
                var comboDepths = Vector.ConditionalSelect(mask, inDepth, destDepths);

                byte coverageMask = 0;
                for(int i = 0; i < Vector<int>.Count; i++)
                {
                    coverageMask <<= 1;
                    coverageMask |= mask[i] == -1 ? (byte)1 : (byte)0;
                }
                SetCoverage8(pixelNum, coverageMask);

                unsafe
                {
                    fixed (uint* _pixels = &pixels[pixelNum])
                    {
                        Vector.Store(comboPixels, _pixels);
                    }   
                    fixed(float* _depths = &depth[pixelNum])
                    {
                        Vector.Store(comboDepths, _depths);
                    }
                }                               
            }
            else
            {
                for (int i = 0; i < SIMDSize; i++)
                {
                    if ((xStart + i < width) && (mask.GetElement(i) != 0))
                    {
                        pixels[pixelNum + i] = color.GetElement(i);
                        depth[pixelNum + i] = inDepth.GetElement(i);
                        SetCoverage(pixelNum + i);
                    }
                }
            }

                
        }

        public void Fill(byte a, byte r, byte g, byte b)
        {
            uint color = ((uint)a << 24) + ((uint)r << 16) + ((uint)g << 8) + (uint)b;
            Fill(color);
        }

        public void Fill(uint color)
        {
            Array.Fill(pixels, color);            
        }

        public void ClearDB()
        {
            Array.Fill(depth, float.MaxValue);
            Array.Fill(tileMinDepth, float.MaxValue);
            Array.Fill(tileMaxDepth, float.MaxValue);
        }

        public void ClearCoverage()
        {
            Array.Fill(coverage, 0);
            Array.Fill(tileCoverage, 0);
        }

        public void ValidateFrame()
        {
            for (int i = 0; i < pixels.Length; i++)
            {                
                if (pixels[i] != pixelInitialValue)
                {
                    //Console.WriteLine($"pixel {i} is not equal to initial value");
                    if (depth[i] == depthInitialValue)
                    {
                        throw new InvalidDataException("pixel buffer has a value but depth is not written");
                    }
                    //also check coverage
                    if(!GetCoverage(i))
                    {                        
                        throw new InvalidDataException("coverage is not set for a pixel that is set");
                        
                    }
                }
                if (depth[i] != depthInitialValue)
                {
                    if (pixels[i] == pixelInitialValue)
                    {
                        throw new InvalidDataException("depth is set but pixels are not drawn");
                    }
                }
                if(GetCoverage(i))
                {
                    if (pixels[i] == pixelInitialValue)
                    {
                        throw new InvalidDataException("coverage is set but pixel isn't");
                    }
                    if (depth[i] == depthInitialValue)
                    {
                        throw new InvalidDataException("coverage is set but depth isn't");
                    }

                }
            }
            int numTiles = pixels.Length / 4096;
            //validate all coverages
            //tilecoverage looks like this:
            //c0: if b0 b1 ... b63 all 1's, 1, else 0
            //c1: if b0 b1 ... b63 all 1's, 1, else 0
            //..
            //c63: if b0 b1 ... b63 all 1's, 1, else 0
            for (int i = 0; i < numTiles; i++)
            {
                int tileX = i % binsX;
                int tileY = i / binsX;
                //for each tile... go row by row.  for each row with all pixels set, check if coverage mask is correct
                int pixelOffsetX = tileX * 64;
                int pixelOffsetY = tileY * 64 * width;
                //check if entire tile is covered
                bool tileIsCovered = true;
                int pixelNum = pixelOffsetX + pixelOffsetY;
                for (int rowNum = 0; rowNum < 64; rowNum++)
                {                    
                    for(int n = 0; n < 64; n++)
                    {
                        if (!GetCoverage(pixelNum + n))
                        {
                            tileIsCovered = false;
                            break;
                        }
                    }
                    pixelNum += width;
                    if (tileIsCovered == false) break;                    
                }
                if(!tileIsCovered && GetTileCoverage(i))
                {
                    throw new InvalidDataException("tile coverage is marked as yes but not all pixels are marked as covered");
                }
                if(tileIsCovered && !GetTileCoverage(i))
                {
                    throw new InvalidDataException("all elements are marked as covered but tile is not marked as covered");
                }
            }
        }

    }
}
