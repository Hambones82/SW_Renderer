using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Software_Renderer
{
    internal static class Constants
    {
        public static Vector<float> SIMDIncrement;

        static Constants()
        {
            Span<float> values = stackalloc float[Vector<float>.Count];
            for (int i = 0; i < Vector<float>.Count; i++)
            {
                values[i] = i;
            }
            SIMDIncrement = new Vector<float>(values);
        }
    }
}
