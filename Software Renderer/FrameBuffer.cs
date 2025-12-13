using System;
using System.Collections.Generic;
using System.Linq;
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
        public readonly int _size;
        public readonly int numBins;
        public readonly int binsX;
        public readonly int binsY;

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

        public FrameBuffer(int width, int height)
        {
            this.width = width;
            this.height = height;
            numBins = width * height / (binDimension * binDimension);
            pixels = new uint[width * height];
            depth = new float[width * height];
            _size = width * height;            
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

        public void SetPixel(int x, int y, float inDepth, uint color)
        {
            pixels[width * y + x] = color;
            depth[width * y + x] = inDepth;
        }

        public void SetPixel(int pixelNum, float inDepth, uint color)
        {
            pixels[pixelNum] = color;
            depth[pixelNum] = inDepth;
        }
        
        //also set coverage mask...
        public void SetPixelParallel(int xStart, int pixelNum, Vector<int> mask, Vector<float> inDepth, Vector<uint> color)
        {            
            int SIMDSize = Vector<float>.Count;
            if (pixelNum > _size) return;            
            if(xStart + SIMDSize < width)
            {
                Vector<uint> uIntMask = Vector.AsVectorUInt32(mask);
                
                Vector<uint> destPixels = new Vector<uint>(pixels, pixelNum);
                var comboPixels = Vector.ConditionalSelect(uIntMask, color, destPixels);

                Vector<float> destDepths = new Vector<float>(depth, pixelNum);
                var comboDepths = Vector.ConditionalSelect(mask, inDepth, destDepths);


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
    }
}
