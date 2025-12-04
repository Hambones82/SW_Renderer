using SDL3;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Software_Renderer
{
    internal class Core
    {
        private FrameBuffer _fb;       

        public Core()
        {
            _fb = new FrameBuffer(800, 600);
            _fb.Fill(255, 0, 0, 0);
        }

        public void Run()
        {
            SDLBackEnd backEnd = new SDLBackEnd(_fb);
            SWRenderer renderer = new SWRenderer(800, 600);              

            var loop = true;
            long ticksStart = DateTime.Now.Ticks;
            float rotation = 0.0f;
            float rotationRate = 1.0f;
            const float rateNormalization = 0.0000001f;
            int frameCount = 0;
            int FPSReportingInterval = 4;
            const float backendFrameRate = 30f;
            float BackendInterval = (1.0f / backendFrameRate) * 1000f;
            Stopwatch FPSReportTimer = Stopwatch.StartNew();
            Stopwatch BackendOutputTimer = Stopwatch.StartNew();
            while (loop)
            {
                frameCount++;
                if (FPSReportTimer.Elapsed.TotalSeconds > FPSReportingInterval)
                {
                    Console.WriteLine($"{frameCount/ FPSReportingInterval}FPS");
                    FPSReportTimer.Restart();
                    frameCount = 0;
                }
                long dt = DateTime.Now.Ticks - ticksStart;
                rotation = (float)dt * rotationRate * rateNormalization;
                var initialMesh = CubeFactory.CreateCube();
                var mesh = Mesh.Rotate(initialMesh, Matrix4x4.RotationY(rotation));
                renderer.Render(mesh, _fb);

                if (BackendOutputTimer.Elapsed.TotalMilliseconds > BackendInterval)
                {                    
                    BackendOutputTimer.Restart();
                    loop = backEnd.UpdateGraphics();
                    _fb.Fill(0);
                }
            }
            backEnd.Destroy();            
        }
    }
}
