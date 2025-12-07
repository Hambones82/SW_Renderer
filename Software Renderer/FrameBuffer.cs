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
        public const int triangleBufferSize = 100;
        public static int binDimension = 8;
        public int[] triIndices = new int[triangleBufferSize];
        public float coarseDepth = float.MaxValue;
        public Bin() { }
    }

    public class FrameBuffer
    {
        public int width;
        public int height;
        public uint[] pixels;
        public float[] depth;
        public readonly int _size;
        public float[] coarseDepth;
                
        //bins are triangle buffer size, as well as 
        public Bin[] bins;


        public FrameBuffer(int width, int height)
        {
            this.width = width;
            this.height = height;
            pixels = new uint[width * height];
            depth = new float[width * height];
            _size = width * height;            
            bins = new Bin[width * height / (Bin.binDimension * Bin.binDimension)];
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

        //this needs to actually occur in parallel...
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
            Array.Fill(coarseDepth, float.MaxValue);
        }
    }
}
