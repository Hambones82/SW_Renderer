using SDL3;

namespace Software_Renderer
{
    internal class Program
    {
        [STAThread]
        static void Main(string[] args)
        {
            Core core = new Core();
            core.Run();                        
        }
    }
}
