using SDL3;
using System.Runtime.Intrinsics.X86;

namespace Software_Renderer
{
    internal class Program
    {        
        static void Main(string[] args)
        {
            /*
            if (Avx.IsSupported) Console.WriteLine("avx is supported");
            else Console.WriteLine("avx is not supported");
            if (Avx2.IsSupported) Console.WriteLine("avx2 is supported");
            else Console.WriteLine("avx2 is not supported");
            if (Avx512F.IsSupported) Console.WriteLine("avx 512F is supported");
            else Console.WriteLine("avx 512F is not supported");
            if (Avx512BW.IsSupported) Console.WriteLine("avx 512BW is supported");
            else Console.WriteLine("avx 512BW is not supported");
            if (Avx512CD.IsSupported) Console.WriteLine("avx 512CD is supported");
            else Console.WriteLine("avx 512CD is not supported");
            if (Avx512DQ.IsSupported) Console.WriteLine("avx 512DQ is supported");
            else Console.WriteLine("avx 512DQ is not supported");
            if (Avx512Vbmi.IsSupported) Console.WriteLine("avx vbmi is supported");
            else Console.WriteLine("avx vbmi is not supported");
            */
            Core core = new Core();
            core.Run();                        
        }
    }
}
