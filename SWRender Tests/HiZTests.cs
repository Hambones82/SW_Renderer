using Software_Renderer;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace SWRender_Tests
{
    public class DepthLevelTests
    {
        private static Vector<float> V(params float[] values)
        {
            int n = Vector<float>.Count;
            var arr = new float[n];
            for (int i = 0; i < n; i++)
                arr[i] = i < values.Length ? values[i] : values[values.Length - 1];
            return new Vector<float>(arr);
        }


        [Fact]
        public void DepthLevel_Reduction0_64x64_HasFullResolution()
        {
            int fbWidth = 64;
            int fbHeight = 64;

            var level = new DepthLevel(fbWidth, fbHeight, inReductionX: 0, inReductionY: 0);

            Assert.Equal(0, level.reductionX);
            Assert.Equal(0, level.reductionY);

            // 8^(0) = 1, so no reduction
            Assert.Equal(64, level.numCellsInX);
            Assert.Equal(64, level.numCellsInY);

            int cellCount = level.numCellsInX * level.numCellsInY;

            Assert.NotNull(level.depthData);
            Assert.Equal(cellCount, level.depthData.Length);
            Assert.All(level.depthData, d => Assert.Equal(float.MaxValue, d));

            Assert.NotNull(level.validData);
            Assert.Equal(cellCount, level.validData.Length);
            Assert.All(level.validData, v => Assert.True(v));
        }

        [Fact]
        public void DepthLevel_Reduction1_64x64_IsDividedBy8()
        {
            int fbWidth = 64;
            int fbHeight = 64;

            var level = new DepthLevel(fbWidth, fbHeight, inReductionX: 1, inReductionY: 1);

            Assert.Equal(1, level.reductionX);
            Assert.Equal(1, level.reductionY);

            // 8^(1) = 8 → 64 / 8 = 8
            Assert.Equal(8, level.numCellsInX);
            Assert.Equal(8, level.numCellsInY);

            int cellCount = level.numCellsInX * level.numCellsInY;

            Assert.NotNull(level.depthData);
            Assert.Equal(cellCount, level.depthData.Length);
            Assert.All(level.depthData, d => Assert.Equal(float.MaxValue, d));

            Assert.NotNull(level.validData);
            Assert.Equal(cellCount, level.validData.Length);
            Assert.All(level.validData, v => Assert.True(v));
        }

        [Fact]
        public void DepthLevel_Reduction2_64x64_IsDividedBy64()
        {
            int fbWidth = 64;
            int fbHeight = 64;

            var level = new DepthLevel(fbWidth, fbHeight, inReductionX: 2, inReductionY: 2);

            Assert.Equal(2, level.reductionX);
            Assert.Equal(2, level.reductionY);

            // 8^(2) = 64 → 64 / 64 = 1
            Assert.Equal(1, level.numCellsInX);
            Assert.Equal(1, level.numCellsInY);

            int cellCount = level.numCellsInX * level.numCellsInY;
            Assert.Equal(1, cellCount);

            Assert.NotNull(level.depthData);
            Assert.Single(level.depthData);
            Assert.Equal(float.MaxValue, level.depthData[0]);

            Assert.NotNull(level.validData);
            Assert.Single(level.validData);
            Assert.True(level.validData[0]);
        }

        [Fact]
        public void DepthLevel_ReductionMixed_128x64_ComputesExpectedSizes()
        {
            int fbWidth = 128;  // divisible by 64 as required
            int fbHeight = 64;

            // Reduce X by 8, Y by 64
            var level = new DepthLevel(fbWidth, fbHeight, inReductionX: 1, inReductionY: 2);

            Assert.Equal(1, level.reductionX);
            Assert.Equal(2, level.reductionY);

            // trueReductionX = 8 -> 128 / 8 = 16
            // trueReductionY = 64 -> 64 / 64 = 1
            Assert.Equal(16, level.numCellsInX);
            Assert.Equal(1, level.numCellsInY);

            int cellCount = level.numCellsInX * level.numCellsInY;
            Assert.Equal(16, cellCount);

            Assert.NotNull(level.depthData);
            Assert.Equal(cellCount, level.depthData.Length);
            Assert.All(level.depthData, d => Assert.Equal(float.MaxValue, d));

            Assert.NotNull(level.validData);
            Assert.Equal(cellCount, level.validData.Length);
            Assert.All(level.validData, v => Assert.True(v));
        }

        [Theory]
        [InlineData(0)]
        [InlineData(1)]
        [InlineData(2)]
        public void DepthLevel_ValidReductions_Respect8PowerScaling(int reduction)
        {
            int fbWidth = 64;
            int fbHeight = 64;

            var level = new DepthLevel(fbWidth, fbHeight, inReductionX: reduction, inReductionY: reduction);

            int blockSize = 1 << (reduction * 3); // 8^reduction
            int expectedSizeX = fbWidth / blockSize;
            int expectedSizeY = fbHeight / blockSize;

            Assert.Equal(reduction, level.reductionX);
            Assert.Equal(reduction, level.reductionY);
            Assert.Equal(expectedSizeX, level.numCellsInX);
            Assert.Equal(expectedSizeY, level.numCellsInY);

            int cellCount = expectedSizeX * expectedSizeY;
            Assert.Equal(cellCount, level.depthData.Length);
            Assert.Equal(cellCount, level.validData.Length);
        }

        [Fact]
        public void TestLowestLevelHiZWritesToEmpty_FirstParentCell()
        {
            FrameBuffer fb = new FrameBuffer(256, 256);//256/8, 256/8
            Span<float> depthValues = stackalloc float[Constants.SIMDCount];
            for (int i = 0; i < Constants.SIMDCount; i++)
            {
                //0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7
                depthValues[i] = i * 0.1f;
            }
            Vector<float> depths = new Vector<float>(depthValues);
            fb.UpdateHiZSIMD(depths, 0, 0);
            
            ref DepthLevel testingLevel = ref fb.hiZBuffer[0];
            Debug.Assert(testingLevel.cellXYOfMax[0] == new Coord2D(7, 0));
            Debug.Assert(testingLevel.validData[0] == true);
            Debug.Assert(testingLevel.depthData[0] == 0.7f);

        }

        [Theory]
        [InlineData(64, 128)]
        [InlineData(80, 136)]
        [InlineData(88, 192)]
        public void TestLowestLevelHiZWritesToEmpty_NthParentCell(int screenX, int screenY)
        {
            FrameBuffer fb = new FrameBuffer(256, 256);//256/8, 256/8
            Span<float> depthValues = stackalloc float[Constants.SIMDCount];
            for (int i = 0; i < Constants.SIMDCount; i++)
            {
                //0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7
                depthValues[i] = i * 0.1f;
            }
            Vector<float> depths = new Vector<float>(depthValues);
            fb.UpdateHiZSIMD(depths, screenX, screenY);
            
            ref DepthLevel testingLevel = ref fb.hiZBuffer[0];

            int elementID = fb.GetHiZElementID(screenX, screenY, 0);
            Debug.Assert(testingLevel.cellXYOfMax[elementID] == new Coord2D(screenX + 7, screenY));
            Debug.Assert(testingLevel.validData[elementID] == true);
            Debug.Assert(testingLevel.depthData[elementID] == 0.7f);
            Debug.Assert(testingLevel.depthData[(elementID + 1) % testingLevel.sizeInCells] == float.MaxValue);
        }

        [Theory]
        [InlineData(64, 128)]
        [InlineData(80, 136)]
        [InlineData(88, 192)]
        public void TestLowestLevelHiOverwrites_NthParentCell(int screenX, int screenY)
        {
            FrameBuffer fb = new FrameBuffer(256, 256);//256/8, 256/8
            Span<float> depthValues = stackalloc float[Constants.SIMDCount];
            for (int i = 0; i < Constants.SIMDCount; i++)
            {
                //0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7
                depthValues[i] = i * 0.1f;
            }
            Vector<float> depths = new Vector<float>(depthValues);
            fb.UpdateHiZSIMD(depths, screenX, screenY);
            //
            for(int i = 0; i < Constants.SIMDCount; i++)
            {
                depthValues[i] = 0.05f * i;
            }
            depths = new Vector<float>(depthValues);
            fb.UpdateHiZSIMD(depths, screenX, screenY);
            
            ref DepthLevel testingLevel = ref fb.hiZBuffer[0];

            int elementID = fb.GetHiZElementID(screenX, screenY, 0);
            Debug.Assert(testingLevel.cellXYOfMax[elementID] == new Coord2D(screenX+7, screenY));
            Debug.Assert(testingLevel.validData[elementID] == true);
            Debug.Assert(testingLevel.depthData[elementID] == 0.35f);
            Debug.Assert(testingLevel.depthData[(elementID + 1) % testingLevel.sizeInCells] == float.MaxValue);
        }
        
        [Theory]
        [InlineData(64, 128)]
        [InlineData(80, 136)]
        [InlineData(88, 192)]
        public void TestNoInvalidationsInHigherLevels(int screenX, int screenY)
        {
            FrameBuffer fb = new FrameBuffer(256, 256);//256/8, 256/8
            Span<float> depthValues = stackalloc float[Constants.SIMDCount];
            for (int i = 0; i < Constants.SIMDCount; i++)
            {
                //0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7
                depthValues[i] = i * 0.1f;
            }
            Vector<float> depths = new Vector<float>(depthValues);
            fb.UpdateHiZSIMD(depths, screenX, screenY);
            
            ref DepthLevel testingLevel = ref fb.hiZBuffer[1];
            int elementID = fb.GetHiZElementID(screenX, screenY, 1);            
            Debug.Assert(testingLevel.validData[elementID] == true);//actaully these will all remain valid...
            Debug.Assert(testingLevel.validData[(elementID + 1) % testingLevel.sizeInCells] == true);

            testingLevel = ref fb.hiZBuffer[2];
            elementID = fb.GetHiZElementID(screenX, screenY, 2);
            Debug.Assert(testingLevel.validData[elementID] == true);
            Debug.Assert(testingLevel.validData[(elementID + 1) % testingLevel.sizeInCells] == true);

            testingLevel = ref fb.hiZBuffer[3];
            elementID = fb.GetHiZElementID(screenX, screenY, 3);
            Debug.Assert(testingLevel.validData[elementID] == true);
            Debug.Assert(testingLevel.validData[(elementID + 1) % testingLevel.sizeInCells] == true);
        }
        
        [Theory]
        [InlineData(0, 0)]      //first of all levels - all should be invalidated
        [InlineData(64, 0)]     //same
        [InlineData(64, 64)]    //same
        public void TestL1L2L3Invalid(int screenX, int screenY)
        {
            FrameBuffer fb = new FrameBuffer(256, 256);//256/8, 256/8
            Span<float> depthValues = stackalloc float[Constants.SIMDCount];
            for (int i = 0; i < Constants.SIMDCount; i++)
            {
                //0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7
                depthValues[i] = i * 0.1f;
            }
            Vector<float> depths = new Vector<float>(depthValues);
            fb.UpdateHiZSIMD(depths, screenX, screenY);

            ref DepthLevel testingLevel = ref fb.hiZBuffer[1];
            int elementID = fb.GetHiZElementID(screenX, screenY, 1);
            Debug.Assert(testingLevel.validData[elementID] == false);//actaully these will all remain valid...
            Debug.Assert(testingLevel.validData[(elementID + 1) % testingLevel.sizeInCells] == true);

            testingLevel = ref fb.hiZBuffer[2];
            elementID = fb.GetHiZElementID(screenX, screenY, 2);
            Debug.Assert(testingLevel.validData[elementID] == false);
            Debug.Assert(testingLevel.validData[(elementID + 1) % testingLevel.sizeInCells] == true);

            testingLevel = ref fb.hiZBuffer[3];
            elementID = fb.GetHiZElementID(screenX, screenY, 3);
            Debug.Assert(testingLevel.validData[elementID] == false);
            Debug.Assert(testingLevel.validData[(elementID + 1) % testingLevel.sizeInCells] == true);
        }


        [Theory]
        [InlineData(0, 8)]      //first of all levels - all should be invalidated
        [InlineData(64, 8)]     //same
        [InlineData(64, 72)]    //same
        public void TestL1L2InvalidL3Valid(int screenX, int screenY)
        {
            FrameBuffer fb = new FrameBuffer(256, 256);//256/8, 256/8
            Span<float> depthValues = stackalloc float[Constants.SIMDCount];
            for (int i = 0; i < Constants.SIMDCount; i++)
            {
                //0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7
                depthValues[i] = i * 0.1f;
            }
            Vector<float> depths = new Vector<float>(depthValues);
            fb.UpdateHiZSIMD(depths, screenX, screenY);

            ref DepthLevel testingLevel = ref fb.hiZBuffer[1];
            int elementID = fb.GetHiZElementID(screenX, screenY, 1);
            Debug.Assert(testingLevel.validData[elementID] == false);//actaully these will all remain valid...
            Debug.Assert(testingLevel.validData[(elementID + 1) % testingLevel.sizeInCells] == true);

            testingLevel = ref fb.hiZBuffer[2];
            elementID = fb.GetHiZElementID(screenX, screenY, 2);
            Debug.Assert(testingLevel.validData[elementID] == false);
            Debug.Assert(testingLevel.validData[(elementID + 1) % testingLevel.sizeInCells] == true);

            testingLevel = ref fb.hiZBuffer[3];
            elementID = fb.GetHiZElementID(screenX, screenY, 3);
            Debug.Assert(testingLevel.validData[elementID] == true);
            Debug.Assert(testingLevel.validData[(elementID + 1) % testingLevel.sizeInCells] == true);
        }

        [Theory]
        [InlineData(8, 8)]      //first of all levels - all should be invalidated
        [InlineData(72, 8)]     //same
        [InlineData(72, 72)]    //same
        public void TestL1InvalidL2L3Valid(int screenX, int screenY)
        {
            FrameBuffer fb = new FrameBuffer(256, 256);//256/8, 256/8
            Span<float> depthValues = stackalloc float[Constants.SIMDCount];
            for (int i = 0; i < Constants.SIMDCount; i++)
            {
                //0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7
                depthValues[i] = i * 0.1f;
            }
            Vector<float> depths = new Vector<float>(depthValues);
            fb.UpdateHiZSIMD(depths, screenX, screenY);
            
            ref DepthLevel testingLevel = ref fb.hiZBuffer[1];
            int elementID = fb.GetHiZElementID(screenX, screenY, 1);
            Debug.Assert(testingLevel.validData[elementID] == false);//actaully these will all remain valid...
            Debug.Assert(testingLevel.validData[(elementID + 1) % testingLevel.sizeInCells] == true);

            testingLevel = ref fb.hiZBuffer[2];
            elementID = fb.GetHiZElementID(screenX, screenY, 2);
            Debug.Assert(testingLevel.validData[elementID] == true);
            Debug.Assert(testingLevel.validData[(elementID + 1) % testingLevel.sizeInCells] == true);

            testingLevel = ref fb.hiZBuffer[3];
            elementID = fb.GetHiZElementID(screenX, screenY, 3);
            Debug.Assert(testingLevel.validData[elementID] == true);
            Debug.Assert(testingLevel.validData[(elementID + 1) % testingLevel.sizeInCells] == true);
        }
        
        [Theory]
        [InlineData(8, 10)]      //first of all levels - all should be invalidated
        [InlineData(72, 10)]     //same
        [InlineData(72, 74)]    //same
        public void TestL1L2L3Valid(int screenX, int screenY)
        {
            FrameBuffer fb = new FrameBuffer(256, 256);//256/8, 256/8
            Span<float> depthValues = stackalloc float[Constants.SIMDCount];
            for (int i = 0; i < Constants.SIMDCount; i++)
            {
                //0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7
                depthValues[i] = i * 0.1f;
            }
            Vector<float> depths = new Vector<float>(depthValues);
            fb.UpdateHiZSIMD(depths, screenX, screenY);

            ref DepthLevel testingLevel = ref fb.hiZBuffer[1];
            int elementID = fb.GetHiZElementID(screenX, screenY, 1);
            Debug.Assert(testingLevel.validData[elementID] == true);//actaully these will all remain valid...
            Debug.Assert(testingLevel.validData[(elementID + 1) % testingLevel.sizeInCells] == true);

            testingLevel = ref fb.hiZBuffer[2];
            elementID = fb.GetHiZElementID(screenX, screenY, 2);
            Debug.Assert(testingLevel.validData[elementID] == true);
            Debug.Assert(testingLevel.validData[(elementID + 1) % testingLevel.sizeInCells] == true);

            testingLevel = ref fb.hiZBuffer[3];
            elementID = fb.GetHiZElementID(screenX, screenY, 3);
            Debug.Assert(testingLevel.validData[elementID] == true);
            Debug.Assert(testingLevel.validData[(elementID + 1) % testingLevel.sizeInCells] == true);
        }
    }
}

