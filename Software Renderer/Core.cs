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

        private int width = 800, height = 600;

        public Core()
        {
            _fb = new FrameBuffer(width, height);
            _fb.Fill(255, 0, 0, 0);
        }

        public void Run()
        {
            SDLBackEnd backEnd = new SDLBackEnd(_fb);
            //SWRenderer renderer = new SWRenderer(width, height);              
            RenderingPipeline renderer = new RenderingPipeline(width, height, new DepthShader(), new VertexShader(width, height));

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
                _fb.ClearDB();
                _fb.Fill(0);
                frameCount++;
                if (FPSReportTimer.Elapsed.TotalSeconds > FPSReportingInterval)
                {
                    Console.WriteLine($"{frameCount/ FPSReportingInterval}FPS");
                    FPSReportTimer.Restart();
                    frameCount = 0;    
                    
                    if(EventCounterLog.enabled)
                    {
                        Console.WriteLine(EventCounterLog.Dump());

                        EventCounterLog.Clear();
                    }
                    
                }
                long dt = DateTime.Now.Ticks - ticksStart;
                rotation = (float)dt * rotationRate * rateNormalization;
                var initialMesh = CubeFactory.CreateCube();
                var mesh = Mesh.Rotate(initialMesh, Matrix4x4.RotationY(rotation));
                //renderer.Render(mesh, _fb);
                renderer.RenderMesh(mesh, _fb);

                //favorable draw order
                
                for(int i = 0; i < 100; i++)
                {
                    var newMesh = Mesh.Translate(mesh, Matrix4x4.Translation(0.001f*i, 0, -0.001f * i));
                    renderer.RenderMesh(newMesh, _fb);
                }
                

                //unfavorable
                /*
                for (int i = 0; i < 1000; i++)
                {
                    var newMesh = Mesh.Translate(mesh, Matrix4x4.Translation(-0.1f * i, 0, 0.1f * i));
                    renderer.Render(newMesh, _fb);
                }*/
                renderer.NewFrame(_fb);

                if (BackendOutputTimer.Elapsed.TotalMilliseconds > BackendInterval)
                {                    
                    BackendOutputTimer.Restart();
                    loop = backEnd.UpdateGraphics();                                       
                }
                
            }
            backEnd.Destroy();            
        }
    }
}
