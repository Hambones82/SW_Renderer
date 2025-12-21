using Software_Renderer;
using System;
using System.Collections.Generic;
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
        public void UpdateHiZSIMD_SetsValidAndOverwrites_WhenElementWasInvalid()
        {
            var fb = new FrameBuffer(64, 64);

            int elementID = 0;                 // x=0..SIMD-1, y=0 → first Hi-Z cell
            int simdSize = Constants.SIMDCount;
            var depths = new float[simdSize];
            var maskInts = new int[simdSize];

            // Fill depths with a simple increasing pattern
            for (int i = 0; i < simdSize; i++)
            {
                depths[i] = i + 1.0f;          // 1, 2, 3, ...
                maskInts[i] = -1;              // mask is currently ignored
            }

            int expectedMaxLane = simdSize - 1;
            float expectedMaxDepth = depths[expectedMaxLane];

            // Start with bogus values in hiZBuffer[0]
            fb.hiZBuffer[0].validData[elementID] = false;
            fb.hiZBuffer[0].depthData[elementID] = -123.0f;
            fb.hiZBuffer[0].subElementIDOfMax[elementID] = -1;

            var depthVec = new Vector<float>(depths);
            var maskVec = new Vector<int>(maskInts);

            // Act
            fb.UpdateHiZSIMD(depthVec, maskVec, startPixelNum: 0, x: 0, y: 0);

            // Assert: eager update should set valid and overwrite previous values
            Assert.True(fb.hiZBuffer[0].validData[elementID]);
            Assert.Equal(expectedMaxDepth, fb.hiZBuffer[0].depthData[elementID]);
            Assert.Equal(expectedMaxLane, fb.hiZBuffer[0].subElementIDOfMax[elementID]);
        }

        [Fact]
        public void UpdateHiZSIMD_OverwritesPreviousHiZData_EvenIfAlreadyValid()
        {
            var fb = new FrameBuffer(64, 64);

            int elementID = 0;
            int simdSize = Constants.SIMDCount;
            var depths = new float[simdSize];
            var maskInts = new int[simdSize];

            // Initialize a pattern and then force a known max at some lane
            for (int i = 0; i < simdSize; i++)
            {
                depths[i] = 10.0f + i;
                maskInts[i] = -1;
            }

            int expectedMaxLane = simdSize / 2;
            float expectedMaxDepth = 1000.0f;
            depths[expectedMaxLane] = expectedMaxDepth;

            var depthVec = new Vector<float>(depths);
            var maskVec = new Vector<int>(maskInts);

            // Prepopulate hiZBuffer[0] with different "old" values
            fb.hiZBuffer[0].validData[elementID] = true;
            fb.hiZBuffer[0].depthData[elementID] = 1.0f;          // old, smaller max
            fb.hiZBuffer[0].subElementIDOfMax[elementID] = 0;     // old lane

            // Act
            fb.UpdateHiZSIMD(depthVec, maskVec, startPixelNum: 0, x: 0, y: 0);

            // Assert: new max overwrites old depth and lane; valid is true
            Assert.True(fb.hiZBuffer[0].validData[elementID]);
            Assert.Equal(expectedMaxDepth, fb.hiZBuffer[0].depthData[elementID]);
            Assert.Equal(expectedMaxLane, fb.hiZBuffer[0].subElementIDOfMax[elementID]);
        }


        [Fact]
        public void UpdateHiZSIMD_EagerUpdate_ComputesMaxDepthAndLane()
        {
            // Arrange
            var fb = new FrameBuffer(64, 64);

            // For x = 0, y = 0, the corresponding level-0 Hi-Z element
            // will always be index 0 in a row-major layout.
            int elementID = 0;

            int simdSize = Constants.SIMDCount;
            var depths = new float[simdSize];
            var maskInts = new int[simdSize];

            // Fill depths with a known pattern, pick a clear max lane
            for (int i = 0; i < simdSize; i++)
            {
                depths[i] = i + 1.0f;   // 1, 2, 3, ...
                maskInts[i] = -1;       // all lanes "written" (mask not actually used anymore)
            }

            int expectedMaxLane = simdSize / 2;
            float expectedMaxDepth = 100.0f;
            depths[expectedMaxLane] = expectedMaxDepth;

            var depthVec = new Vector<float>(depths);
            var maskVec = new Vector<int>(maskInts);

            // Sanity: start with some different values in hiZBuffer[0]
            fb.hiZBuffer[0].validData[elementID] = false;
            fb.hiZBuffer[0].depthData[elementID] = -1.0f;
            fb.hiZBuffer[0].subElementIDOfMax[elementID] = -1;

            // Act
            fb.UpdateHiZSIMD(depthVec, maskVec, startPixelNum: 0, x: 0, y: 0);

            // Assert: eager update should set valid, depth, and lane-of-max
            Assert.True(fb.hiZBuffer[0].validData[elementID]);
            Assert.Equal(expectedMaxDepth, fb.hiZBuffer[0].depthData[elementID]);
            Assert.Equal(expectedMaxLane, fb.hiZBuffer[0].subElementIDOfMax[elementID]);
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
            Assert.All(level.validData, v => Assert.False(v));
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
            Assert.All(level.validData, v => Assert.False(v));
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
            Assert.False(level.validData[0]);
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

            // trueReductionX = 8 → 128 / 8 = 16
            // trueReductionY = 64 → 64 / 64 = 1
            Assert.Equal(16, level.numCellsInX);
            Assert.Equal(1, level.numCellsInY);

            int cellCount = level.numCellsInX * level.numCellsInY;
            Assert.Equal(16, cellCount);

            Assert.NotNull(level.depthData);
            Assert.Equal(cellCount, level.depthData.Length);
            Assert.All(level.depthData, d => Assert.Equal(float.MaxValue, d));

            Assert.NotNull(level.validData);
            Assert.Equal(cellCount, level.validData.Length);
            Assert.All(level.validData, v => Assert.False(v));
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
        public void UpdateHiZSIMD_Level0_StoresMaxDepthAndLane()
        {
            var fb = new FrameBuffer(64, 64);

            int x = 0, y = 0;
            int startPixelNum = 0;       // elementID = startPixelNum >> 3 = 0

            // Max is 9.0 at lane 2
            var depths = V(1f, 2f, 9f, 3f, 4f, 5f, 6f, 7f);
            var mask = Vector<int>.AllBitsSet;

            fb.UpdateHiZSIMD(depths, mask, startPixelNum, x, y);

            int elementID = startPixelNum >> 3;

            Assert.True(fb.hiZBuffer[0].validData[elementID]);
            Assert.Equal(9f, fb.hiZBuffer[0].depthData[elementID], 6);
            Assert.Equal(2, fb.hiZBuffer[0].subElementIDOfMax[elementID]);
        }

        [Fact]
        public void UpdateHiZSIMD_DoesNotInvalidateParents_WhenParentNotValid()
        {
            var fb = new FrameBuffer(64, 64);

            int x = 0, y = 0;
            int startPixelNum = 0;

            // Mark level2 and level3 as valid so we can verify they don't get touched.
            fb.hiZBuffer[2].validData[0] = true;
            fb.hiZBuffer[3].validData[0] = true;

            // Parent (level1) is invalid -> function should return immediately at parentLevel=1.
            fb.hiZBuffer[1].validData[0] = false;

            fb.UpdateHiZSIMD(V(1f, 2f, 3f, 4f, 5f, 6f, 7f, 8f), Vector<int>.AllBitsSet, startPixelNum, x, y);

            // Level1 remains invalid; higher levels unchanged.
            Assert.False(fb.hiZBuffer[1].validData[0]);
            Assert.True(fb.hiZBuffer[2].validData[0]);
            Assert.True(fb.hiZBuffer[3].validData[0]);
        }

        [Fact]
        public void UpdateHiZSIMD_DoesNotInvalidateParents_WhenLaneDoesNotMatch()
        {
            var fb = new FrameBuffer(64, 64);

            int x = 0, y = 0;
            int startPixelNum = 0;

            // For x=0,y=0: parentLevel=1 -> parentElementID=0 and childLaneID=0.
            // Set level1 valid but with mismatching subElementIDOfMax.
            fb.hiZBuffer[1].validData[0] = true;
            fb.hiZBuffer[1].subElementIDOfMax[0] = 7; // mismatch (expected 0)

            // Also set higher levels valid; should remain valid because we return on mismatch.
            fb.hiZBuffer[2].validData[0] = true;
            fb.hiZBuffer[3].validData[0] = true;

            fb.UpdateHiZSIMD(V(8f, 1f, 2f, 3f, 4f, 5f, 6f, 7f), Vector<int>.AllBitsSet, startPixelNum, x, y);

            Assert.True(fb.hiZBuffer[1].validData[0]); // not invalidated
            Assert.True(fb.hiZBuffer[2].validData[0]); // unchanged
            Assert.True(fb.hiZBuffer[3].validData[0]); // unchanged
        }

        [Fact]
        public void UpdateHiZSIMD_InvalidatesAllParents_WhenValidAndLaneMatches()
        {
            var fb = new FrameBuffer(64, 64);

            int x = 0, y = 0;
            int startPixelNum = 0;

            // For x=0,y=0:
            // parentLevel=1: parentElementID=0, childLaneID=0
            // parentLevel=2: parentElementID=(y>>3)=0, childLaneID=(x>>3)%8=0
            // parentLevel=3: parentElementID=0, childLaneID=(y>>6)%8=0
            fb.hiZBuffer[1].validData[0] = true;
            fb.hiZBuffer[1].subElementIDOfMax[0] = 0;

            fb.hiZBuffer[2].validData[0] = true;
            fb.hiZBuffer[2].subElementIDOfMax[0] = 0;

            fb.hiZBuffer[3].validData[0] = true;
            fb.hiZBuffer[3].subElementIDOfMax[0] = 0;

            fb.UpdateHiZSIMD(V(1f, 2f, 3f, 4f, 5f, 6f, 7f, 8f), Vector<int>.AllBitsSet, startPixelNum, x, y);

            Assert.False(fb.hiZBuffer[1].validData[0]);
            Assert.False(fb.hiZBuffer[2].validData[0]);
            Assert.False(fb.hiZBuffer[3].validData[0]);
        }

        [Fact]
        public void UpdateHiZSIMD_InvalidationStopsAtFirstMismatch_LeavingHigherParentsValid()
        {
            var fb = new FrameBuffer(64, 64);

            int x = 0, y = 0;
            int startPixelNum = 0;

            // Level1 matches -> should be invalidated.
            fb.hiZBuffer[1].validData[0] = true;
            fb.hiZBuffer[1].subElementIDOfMax[0] = 0;

            // Level2 mismatches -> should NOT be invalidated, and we should stop there.
            fb.hiZBuffer[2].validData[0] = true;
            fb.hiZBuffer[2].subElementIDOfMax[0] = 3; // mismatch (expected 0)

            // Level3 should remain valid (because we stop at level2 mismatch).
            fb.hiZBuffer[3].validData[0] = true;
            fb.hiZBuffer[3].subElementIDOfMax[0] = 0;

            fb.UpdateHiZSIMD(V(8f, 7f, 6f, 5f, 4f, 3f, 2f, 1f), Vector<int>.AllBitsSet, startPixelNum, x, y);

            Assert.False(fb.hiZBuffer[1].validData[0]); // invalidated
            Assert.True(fb.hiZBuffer[2].validData[0]);  // unchanged
            Assert.True(fb.hiZBuffer[3].validData[0]);  // unchanged
        }
    }
}

