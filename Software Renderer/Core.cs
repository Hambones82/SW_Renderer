using SDL3;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
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
            VertexShader VS = new VertexShader(width, height);
            RenderingPipeline renderer = new RenderingPipeline(width, height, new DepthShader(), VS);
            
            var loop = true;
            long ticksStart = DateTime.Now.Ticks;
            float rotation = 0.0f;
            float rotationRate = 1.0f;
            const float rateNormalization = 0.0000001f;
            int frameCount = 0;
            int ReportingInterval = 4;
            const float backendFrameRate = 60f;
            float BackendInterval = (1.0f / backendFrameRate) * 1000f;
            Stopwatch FPSReportTimer = Stopwatch.StartNew();
            Stopwatch BackendOutputTimer = Stopwatch.StartNew();
            Stopwatch renderTime = new Stopwatch();
            var initialMesh = CubeFactory.CreateCube();
            var mesh = initialMesh;
            var initialMVP = VS.GetMVP();    
            
            while (loop)
            {
                Matrix4x4 MVP = initialMVP;
                _fb.ClearDB();
                _fb.Fill(0);
                frameCount++;
                if (FPSReportTimer.Elapsed.TotalSeconds > ReportingInterval)
                {
                    Console.WriteLine($"{frameCount/ ReportingInterval}FPS");
                    Console.WriteLine($"average render time: {Logger.GetMeasurement(runningAvgLoggerType.wholeFrame)/10f} ns");
                    FPSReportTimer.Restart();
                    frameCount = 0;
                    Logger.ClearMeasurement(runningAvgLoggerType.wholeFrame);                    
                }
                long dt = DateTime.Now.Ticks - ticksStart;
                rotation = (float)dt * rotationRate * rateNormalization;

                VS.SetMVP(width, height, rotation);

                
                renderer.RenderMesh(mesh, _fb);

                
                renderTime.Start();
                
                for(int i = 0; i < 100000; i++)
                {
                    //var newMesh = Mesh.Translate(mesh, Matrix4x4.Translation(0.01f*i, 0, -0.01f * i));
                   
                    VS.SetMVP(width, height, rotation, -0.01f*i);
                    renderer.RenderMesh(mesh, _fb);
                }
                Logger.RecordMeasurement(runningAvgLoggerType.wholeFrame, (float)renderTime.ElapsedTicks);
                                
                renderTime.Reset();
                
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
