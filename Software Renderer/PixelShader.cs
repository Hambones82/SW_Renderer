using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Software_Renderer
{    
    public interface IPixelShader
    {        
        uint Shade(int x, int y, float depth, float w0, float w1, float w2);
        Vector<uint> ParallelShade(Vector<int> x, Vector<int> y,
                                   Vector<float> depth, Vector<float> w0,
                                   Vector<float> w1, Vector<float> w2);
    }

    /*
    public class SimpleColorShader : IPixelShader
    {
        private uint[] colors;

        public SimpleColorShader(uint[] colors)
        {
            this.colors = colors;
        }

        public uint Shade(Fragment fragment)
        {
            return colors[fragment.TriangleIndex % colors.Length];
        }
    }*/

    public class DepthShader : IPixelShader
    {
        public DepthShader() {}

        public uint Shade(int x, int y, float depth, float w0, float w1, float w2)
        {
            //return 0xFF00FFFF;
            return (uint)(depth * (float)(0x0000FFFF)) + 0xFF000000;
            //return 0xFFFFFFFF;
            //return (uint)(depth * 255f) * (uint)0x00010101 + (uint)0xFF000000;

        }

        public Vector<uint> ParallelShade(Vector<int> x, Vector<int> y,
                                          Vector<float> depth, Vector<float> w0,
                                          Vector<float> w1, Vector<float> w2)
        {
            //return new Vector<uint>(0xFF00FFFF);
            return new Vector<uint>((uint)(0xFF000000)) + Vector.ConvertToUInt32(depth * (float)(0x0000FFFF));            
            //return new Vector<uint>(0xFFFFFFFF);
        }
    }
}
