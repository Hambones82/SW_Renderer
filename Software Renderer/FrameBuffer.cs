using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Software_Renderer
{
    public class FrameBuffer
    {
        public int width;
        public int height;
        public uint[] pixels;
        public float[] depth;
        private int _size;
        public FrameBuffer(int width, int height)
        {
            this.width = width;
            this.height = height;
            pixels = new uint[width * height];
            depth = new float[width * height];
            _size = width * height;
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
        public void SetPixelParallel(int xStart, int pixelNum, Vector<int> inside, Vector<float> inDepth, Vector<uint> color)
        {
            for(int i = 0; i < Vector<float>.Count; i++)
            {
                if((xStart + i < width) && (inside.GetElement(i) != 0))
                {
                    pixels[pixelNum + i] = color.GetElement(i);
                    depth[pixelNum + i] = inDepth.GetElement(i);
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
    }
}
