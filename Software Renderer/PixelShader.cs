using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Software_Renderer
{    
    public interface IPixelShader
    {        
        uint Shade(int x, int y, float depth, float w0, float w1, float w2, int triIndex);
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

        public uint Shade(int x, int y, float depth, float w0, float w1, float w2, int triIndex)
        {
            return (uint)(depth * (float)(0x00FFFFFF)) + 0xFF000000;
        }
    }
}
