using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Software_Renderer
{
    public static class Constants
    {
        public static Vector<float> SIMDIncrement;
        public static int SIMDCount;

        static Constants()
        {
            Span<float> values = stackalloc float[Vector<float>.Count];
            for (int i = 0; i < Vector<float>.Count; i++)
            {
                values[i] = i;
            }
            SIMDIncrement = new Vector<float>(values);
            SIMDCount = Vector<float>.Count;
        }
    }
}
