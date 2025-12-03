using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Software_Renderer
{    
    public interface IPixelShader
    {
        uint Shade(Fragment fragment);
    }

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
    }

    public class DepthShader : IPixelShader
    {
        public DepthShader() {}

        public uint Shade(Fragment fragment)
        {
            return (uint)(fragment.Depth * (float)(0x00FFFFFF)) + 0xFF000000;
        }
    }
}
