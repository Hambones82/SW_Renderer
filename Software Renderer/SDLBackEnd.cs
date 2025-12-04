using SDL3;
using System;
using System.Collections.Generic;
using System.Data;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Software_Renderer
{
    internal class SDLBackEnd
    {
        private int width = 800, height = 600;
        private nint window, renderer, renderTexture;
        private FrameBuffer _fb;
        
        public SDLBackEnd(FrameBuffer fb)
        {
            if (!SDL.Init(SDL.InitFlags.Video))
            {
                SDL.LogError(SDL.LogCategory.System, $"SDL could not initialize: {SDL.GetError()}");
                return;
            }

            if (!SDL.CreateWindowAndRenderer("SW Renderer", width, height, 0, out window, out renderer))
            {
                SDL.LogError(SDL.LogCategory.Application, $"Error creating window and rendering: {SDL.GetError()}");
                return;
            }            
            _fb = fb;
            renderTexture = SDL.CreateTexture(renderer, SDL.PixelFormat.ARGB8888, SDL.TextureAccess.Streaming, width, height);
            if(renderTexture == IntPtr.Zero)
            {
                Console.WriteLine("texture creation failed");
            }
        }
        public bool UpdateGraphics()
        {
            SDL.SetRenderDrawColor(renderer, 0, 0, 0, 0);
            
            

            while (SDL.PollEvent(out var e))
            {
                if ((SDL.EventType)e.Type == SDL.EventType.Quit)
                {
                    return false;
                }
            }

            SDL.RenderClear(renderer);

            IntPtr pixelsPtr;
            int pitch;

            if (!SDL.LockTexture(renderTexture, IntPtr.Zero, out pixelsPtr, out pitch))
            {
                Console.WriteLine($"LockTexture failed: {SDL.GetError()}");
            }

            unsafe
            {
                fixed (uint* src = _fb.pixels)
                {                    
                    Buffer.MemoryCopy(
                        src,
                        (void *)pixelsPtr,
                        sizeof(int) * width * height,    
                        sizeof(int) * width * height);                       
                }
            }

            SDL.UnlockTexture(renderTexture);           

            SDL.RenderTexture(renderer, renderTexture, IntPtr.Zero, IntPtr.Zero);

            SDL.RenderPresent(renderer);
            return true;
            
        }
        public void Destroy()
        {
            SDL.DestroyRenderer(renderer);
            SDL.DestroyWindow(window);

            SDL.Quit();
        }
    }    
}
