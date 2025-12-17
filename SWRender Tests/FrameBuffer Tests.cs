using System.Numerics;
using Software_Renderer;

namespace SWRender_Tests
{
    public class FrameBufferTests
    {
        // ---------- Helpers ----------

        private FrameBuffer Create64x64()
        {
            return new FrameBuffer(64, 64);
        }

        private int PixelIndex(FrameBuffer fb, int x, int y)
        {
            return y * fb.width + x;
        }

        // ---------- Bin tests ----------

        [Fact]
        public void Bin_DefaultConstructor_InitializesBufferAndPointers()
        {
            var bin = new Bin();

            Assert.NotNull(bin.triIndices);
            Assert.Equal(Bin.triangleBufferSize, bin.triIndices.Length);
            Assert.Equal(0, bin.head);
            Assert.Equal(0, bin.tail);
        }

        [Fact]
        public void Bin_Clear_ResetsHeadAndTail()
        {
            var bin = new Bin
            {
                head = 10,
                tail = 20
            };

            bin.Clear();

            Assert.Equal(0, bin.head);
            Assert.Equal(0, bin.tail);
        }

        // ---------- Constructor / basic layout ----------

        [Fact]
        public void FrameBuffer_Ctor_InitializesArraysAndBinLayout()
        {
            var fb = Create64x64();

            Assert.Equal(64, fb.width);
            Assert.Equal(64, fb.height);
            Assert.Equal(64 * 64, fb._size);

            Assert.NotNull(fb.pixels);
            Assert.Equal(fb._size, fb.pixels.Length);

            Assert.NotNull(fb.depth);
            Assert.Equal(fb._size, fb.depth.Length);

            Assert.NotNull(fb.coverage);
            Assert.True(fb.coverage.Length >= fb._size / 64);

            Assert.NotNull(fb.tileCoverage);
            Assert.True(fb.tileCoverage.Length >= fb._size / 4096);

            Assert.True(fb.numBins > 0);
            Assert.NotNull(fb.bins);
            Assert.Equal(fb.numBins, fb.bins.Length);

            // At least check first bin
            var firstBin = fb.bins[0];
            Assert.NotNull(firstBin.triIndices);
            Assert.Equal(Bin.triangleBufferSize, firstBin.triIndices.Length);
            Assert.Equal(0, firstBin.head);
            Assert.Equal(0, firstBin.tail);

            Assert.True(fb.binsX > 0);
            Assert.True(fb.binsY > 0);
            Assert.Equal(fb.binsX * fb.binsY, fb.numBins);
        }

        // ---------- SetPixel / SetDepth ----------

        [Fact]
        public void SetPixel_ByXY_SetsPixelColorOnly()
        {
            var fb = Create64x64();
            var idx = PixelIndex(fb, 5, 3);

            uint color = 0xAABBCCDD;
            fb.SetPixel(5, 3, color);

            Assert.Equal(color, fb.pixels[idx]);
            // Depth & coverage unaffected by this overload
            Assert.Equal(0f, fb.depth[idx]);
            Assert.False(fb.GetCoverage(idx));
        }

        [Fact]
        public void SetDepth_ByXY_SetsDepthOnly()
        {
            var fb = Create64x64();
            var idx = PixelIndex(fb, 2, 1);

            float depth = 0.42f;
            fb.SetDepth(2, 1, depth);

            Assert.Equal(depth, fb.depth[idx]);
            Assert.Equal(FrameBuffer.pixelInitialValue, (int)fb.pixels[idx]);
            Assert.False(fb.GetCoverage(idx));
        }

        [Fact]
        public void SetPixel_ByPixelNum_SetsPixelDepthAndCoverage()
        {
            var fb = Create64x64();
            fb.ClearDB();
            fb.ClearCoverage();

            int idx = PixelIndex(fb, 4, 4);
            uint color = 0x11223344;
            float depth = 0.5f;

            fb.SetPixel(idx, depth, color);

            Assert.Equal(color, fb.pixels[idx]);
            Assert.Equal(depth, fb.depth[idx]);
            Assert.True(fb.GetCoverage(idx));
        }

        [Fact]
        public void SetPixel_ByXYFull_SetsPixelDepthAndCoverage()
        {
            var fb = Create64x64();
            fb.ClearDB();
            fb.ClearCoverage();

            uint color = 0x99FF0000;
            float depth = 0.25f;

            fb.SetPixel(1, 2, depth, color);

            int idx = PixelIndex(fb, 1, 2);
            Assert.Equal(color, fb.pixels[idx]);
            Assert.Equal(depth, fb.depth[idx]);
            Assert.True(fb.GetCoverage(idx));
        }

        // ---------- Coverage basic tests ----------

        [Fact]
        public void SetCoverage_ByPixelNum_SetsSingleBit()
        {
            var fb = Create64x64();
            fb.ClearCoverage();

            int idx = PixelIndex(fb, 7, 1); // arbitrary pixel
            fb.SetCoverage(idx);

            Assert.True(fb.GetCoverage(idx));

            // Neighbor pixels should still be uncovered
            if (idx > 0)
                Assert.False(fb.GetCoverage(idx - 1));
            if (idx + 1 < fb._size)
                Assert.False(fb.GetCoverage(idx + 1));
        }

        [Fact]
        public void SetCoverage_ByXY_DelegatesCorrectly()
        {
            var fb = Create64x64();
            fb.ClearCoverage();

            fb.SetCoverage(10, 3);
            int idx = PixelIndex(fb, 10, 3);

            Assert.True(fb.GetCoverage(idx));
        }

        [Fact]
        public void SetBit_SetsCorrectBitPositions()
        {
            var fb = Create64x64();

            // Bit 0 -> MSB (63)
            long res0 = fb.SetBit(0L, 0);
            Assert.Equal(1L << 63, res0);

            // Bit 63 -> LSB (0)
            long res63 = fb.SetBit(0L, 63);
            Assert.Equal(1L << 0, res63);

            // ORing should accumulate
            long combined = fb.SetBit(res0, 63);
            Assert.Equal((1L << 63) | (1L << 0), combined);
        }

        // ---------- SetCoverage8 ----------

        [Fact]
        public void SetCoverage8_SetsEightPixelsAccordingToByte()
        {
            var fb = Create64x64();
            fb.ClearCoverage();

            int pixelNum = 0; // aligned, first word
            byte coverage = 0b1010_0110; // bit 7 is leftmost in this byte

            fb.SetCoverage8(pixelNum, coverage);

            // For pixels pixelNum .. pixelNum+7:
            // bit (7 - i) in 'coverage' maps to pixel 'pixelNum + i'
            for (int i = 0; i < 8; i++)
            {
                bool expected = ((coverage >> (7 - i)) & 0x1) != 0;
                bool actual = fb.GetCoverage(pixelNum + i);
                Assert.Equal(expected, actual);
            }

            // A pixel outside this group should still be false
            Assert.False(fb.GetCoverage(8));
        }

        // ---------- SetCoverage64 / GetTileCoverage ----------

        [Fact]
        public void SetCoverage64_WithMinusOneSetsTileRowBit()
        {
            var fb = Create64x64();
            fb.ClearCoverage();

            // This simulates a fully covered row at y = 0, x starting at 0
            fb.SetCoverage64(0, -1L);

            // For 64x64, there is 1 tile, tile index 0
            Assert.NotEqual(0L, fb.tileCoverage[0]);
            Assert.False(fb.GetTileCoverage(0)); // not all rows yet
        }

        [Fact]
        public void SetCoverage64_AllRowsCovered_MarksTileFullyCovered()
        {
            var fb = Create64x64();
            fb.ClearCoverage();

            // For each row y, mark coverage == -1 for that row
            for (int y = 0; y < 64; y++)
            {
                int pixelNum = y * fb.width; // x = 0
                fb.SetCoverage64(pixelNum, -1L);
            }

            // For a 64x64 framebuffer, there should be exactly 1 tile.
            Assert.True(fb.GetTileCoverage(0));
        }

        // ---------- Fill / ClearDB / ClearCoverage ----------

        [Fact]
        public void Fill_WithBytes_PacksAndFillsPixels()
        {
            var fb = Create64x64();
            byte a = 0x12, r = 0x34, g = 0x56, b = 0x78;

            uint expected = ((uint)a << 24) + ((uint)r << 16) + ((uint)g << 8) + b;
            fb.Fill(a, r, g, b);

            Assert.All(fb.pixels, p => Assert.Equal(expected, p));
        }

        [Fact]
        public void Fill_WithUint_FillsPixels()
        {
            var fb = Create64x64();
            uint color = 0xCAFEBABE;

            fb.Fill(color);

            Assert.All(fb.pixels, p => Assert.Equal(color, p));
        }

        [Fact]
        public void ClearDB_ResetsDepthAndTileDepths()
        {
            var fb = Create64x64();

            // Dirty them first
            for (int i = 0; i < fb.depth.Length; i++)
            {
                fb.depth[i] = -1f;
            }

            fb.ClearDB();

            Assert.All(fb.depth, d => Assert.Equal(FrameBuffer.depthInitialValue, d));
            Assert.All(fb.tileMinDepth, d => Assert.Equal(FrameBuffer.depthInitialValue, d));
            Assert.All(fb.tileMaxDepth, d => Assert.Equal(FrameBuffer.depthInitialValue, d));
        }

        [Fact]
        public void ClearCoverage_ResetsCoverageAndTileCoverage()
        {
            var fb = Create64x64();

            // Dirty coverage
            for (int i = 0; i < fb.coverage.Length; i++)
            {
                fb.coverage[i] = -1;
            }

            for (int i = 0; i < fb.tileCoverage.Length; i++)
            {
                fb.tileCoverage[i] = -1;
            }

            fb.ClearCoverage();

            Assert.All(fb.coverage, c => Assert.Equal(0L, c));
            Assert.All(fb.tileCoverage, c => Assert.Equal(0L, c));
        }

        // ---------- SetPixelParallel ----------

        [Fact]
        public void SetPixelParallel_SimdPath_WritesMaskedPixelsDepthAndCoverage()
        {
            var fb = new FrameBuffer(Vector<float>.Count * 2, 1); // wide enough for SIMD path
            fb.ClearDB();
            fb.ClearCoverage();

            int simdSize = Vector<float>.Count;
            int xStart = 0;
            int pixelNum = 0;

            // Pre-fill pixels and depth so we can see which lanes changed
            for (int i = 0; i < fb._size; i++)
            {
                fb.pixels[i] = 0xDEADBEEFu;
                fb.depth[i] = 123.45f;
            }

            // Build mask pattern: even lanes active, odd lanes inactive
            var maskArr = new int[simdSize];
            var depthArr = new float[simdSize];
            var colorArr = new uint[simdSize];

            for (int i = 0; i < simdSize; i++)
            {
                maskArr[i] = (i % 2 == 0) ? -1 : 0;
                depthArr[i] = i + 0.5f;
                colorArr[i] = 0x01000000u + (uint)i;
            }

            var mask = new Vector<int>(maskArr);
            var inDepth = new Vector<float>(depthArr);
            var color = new Vector<uint>(colorArr);

            fb.SetPixelParallel(xStart, pixelNum, mask, inDepth, color);

            // Check pixels & depth
            for (int i = 0; i < simdSize; i++)
            {
                int idx = pixelNum + i;
                if (maskArr[i] != 0)
                {
                    Assert.Equal(colorArr[i], fb.pixels[idx]);
                    Assert.Equal(depthArr[i], fb.depth[idx]);
                }
                else
                {
                    Assert.Equal(0xDEADBEEFu, fb.pixels[idx]);
                    Assert.Equal(123.45f, fb.depth[idx]);
                }
            }

            // Rebuild expected coverage byte the same way the production code does
            byte expectedCoverageMask = 0;
            for (int i = 0; i < Vector<int>.Count; i++)
            {
                expectedCoverageMask <<= 1;
                expectedCoverageMask |= (maskArr[i] == -1) ? (byte)1 : (byte)0;
            }

            // For the first 8 pixels, compare coverage bits
            for (int i = 0; i < 8 && i < simdSize; i++)
            {
                bool expectedCovered = ((expectedCoverageMask >> (7 - i)) & 0x1) != 0;
                bool actualCovered = fb.GetCoverage(pixelNum + i);
                Assert.Equal(expectedCovered, actualCovered);
            }
        }

        [Fact]
        public void SetPixelParallel_FallbackPath_UsesScalarLoop()
        {
            int simdSize = Vector<float>.Count;
            // Width smaller than xStart + SIMDSize triggers fallback
            var fb = new FrameBuffer(simdSize - 1, 1);
            fb.ClearDB();
            fb.ClearCoverage();

            int xStart = 0;
            int pixelNum = 0;

            for (int i = 0; i < fb._size; i++)
            {
                fb.pixels[i] = 0xABABABABu;
                fb.depth[i] = 99.0f;
            }

            var maskArr = new int[simdSize];
            var depthArr = new float[simdSize];
            var colorArr = new uint[simdSize];

            for (int i = 0; i < simdSize; i++)
            {
                maskArr[i] = (i % 2 == 1) ? -1 : 0; // odd lanes active
                depthArr[i] = i + 10.0f;
                colorArr[i] = 0xFF000000u + (uint)i;
            }

            var mask = new Vector<int>(maskArr);
            var inDepth = new Vector<float>(depthArr);
            var color = new Vector<uint>(colorArr);

            fb.SetPixelParallel(xStart, pixelNum, mask, inDepth, color);

            for (int i = 0; i < simdSize; i++)
            {
                int idx = pixelNum + i;
                if (idx >= fb._size)
                    break;

                if (maskArr[i] != 0)
                {
                    Assert.Equal(colorArr[i], fb.pixels[idx]);
                    Assert.Equal(depthArr[i], fb.depth[idx]);
                    Assert.True(fb.GetCoverage(idx));
                }
                else
                {
                    Assert.Equal(0xABABABABu, fb.pixels[idx]);
                    Assert.Equal(99.0f, fb.depth[idx]);
                    Assert.False(fb.GetCoverage(idx));
                }
            }
        }

        // ---------- ValidateFrame basic sanity ----------

        [Fact]
        public void ValidateFrame_DoesNotThrow_WhenBuffersArePristine()
        {
            var fb = Create64x64();

            // Ensure everything is in the "initial" state:
            fb.Fill(FrameBuffer.pixelInitialValue);
            fb.ClearDB();
            fb.ClearCoverage();

            fb.ValidateFrame(); // if it throws, test fails
        }

        [Fact]
        public void ValidateFrame_Throws_WhenPixelSetButDepthNotSet()
        {
            var fb = Create64x64();
            fb.Fill(FrameBuffer.pixelInitialValue);
            fb.ClearDB();
            fb.ClearCoverage();

            // Set pixel color only – no depth, no coverage
            fb.SetPixel(0, 0, 0xFF0000FFu);

            Assert.Throws<InvalidDataException>(() => fb.ValidateFrame());
        }

        [Fact]
        public void ValidateFrame_Throws_WhenDepthSetButPixelNotSet()
        {
            var fb = Create64x64();
            fb.Fill(FrameBuffer.pixelInitialValue);
            fb.ClearDB();
            fb.ClearCoverage();

            fb.SetDepth(0, 0, 0.1f); // depth only

            Assert.Throws<InvalidDataException>(() => fb.ValidateFrame());
        }

        [Fact]
        public void ValidateFrame_Throws_WhenCoverageSetButPixelAndDepthNotSet()
        {
            var fb = Create64x64();
            fb.Fill(FrameBuffer.pixelInitialValue);
            fb.ClearDB();
            fb.ClearCoverage();

            int idx = PixelIndex(fb, 0, 0);
            fb.SetCoverage(idx); // coverage only

            Assert.Throws<InvalidDataException>(() => fb.ValidateFrame());
        }

    }
}
