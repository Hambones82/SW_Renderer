using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Net.NetworkInformation;
using System.Numerics;
using System.Reflection.Emit;
using System.Text;
using System.Threading.Tasks;

namespace Software_Renderer
{
    public record struct Coord2D(int x, int y);

    public struct Bin
    {
        public const int triangleBufferSize = 10000;
        public int[] triIndices;
        //head is where you read from, tail is where you write to
        public int head = 0, tail = 0;
        public Bin() 
        { 
            triIndices = new int[triangleBufferSize];
        }

        public void Clear()
        {
            head = 0; tail = 0;
        }
    }

    public struct DepthLevel
    {
        //reduction must be in powers of 8
        //e.g., "reduction 1" means divided by 8, "reduction 2" means divided by 64
        //reduction can be max 2
        public int reductionX;
        public int reductionY;
        public readonly int trueReductionX, trueReductionY;
        public int numCellsInX;
        public int numCellsInY;
        public int sizeInCells;        
        
        public float[] depthData;
        
        public bool[] validData;
        //for each chunk, which pixel number stores the max value
        //public int[] subElementIDOfMax;
        public Coord2D[] cellXYOfMax;
        
        
        public DepthLevel(int fbWidth, int fbHeight, int inReductionX, int inReductionY)            
        {
            Debug.Assert((inReductionX <= 2) && (inReductionX >= 0), "reduction in x must be 0..2");
            Debug.Assert((inReductionY <= 2) && (inReductionY >= 0), "reduction in y must be 0..2");
            Debug.Assert(((fbWidth % 64) == 0) && ((fbHeight % 64) == 0), "fb dimensions must be divisible by 64");
            reductionX = inReductionX;
            reductionY = inReductionY;
            trueReductionX = (1 << reductionX * 3);
            trueReductionY = (1 << reductionY * 3);
            numCellsInX = fbWidth / trueReductionX;            
            numCellsInY = fbHeight / trueReductionY;
            sizeInCells = numCellsInX * numCellsInY;
            
            depthData = new float[sizeInCells];
            Array.Fill(depthData, float.MaxValue);
            validData = new bool[sizeInCells];
            Array.Fill(validData, true);
            cellXYOfMax = new Coord2D[sizeInCells];
            //this just isn't right...
            Array.Fill(cellXYOfMax, new Coord2D(0,0));            
        }   
        
        public void Reset()
        {
            Array.Fill(depthData, float.MaxValue);
            Array.Fill(validData, true);
            Array.Fill(cellXYOfMax, new Coord2D(0, 0));            
        }
    }

    public class FrameBuffer
    {
        public int width;
        public int height;
        public uint[] pixels;
        public float[] depth;
        //[0]: 8 pixel wide rows;   [1]: 8x8 tiles
        //[2]: 8 rows of 64 pixels; [3]: 64x64 tiles
        public DepthLevel[] hiZBuffer;        
        public Int64[] coverage;//a per-element coverage bitmask
        public Int64[] tileCoverage; //a bitmask, each of whose bits represents one element of ^coverage
                                     //bit is 1 if all are covered, 0 otherwise
                                     //this means that if tileCoverage[tilenum] == -1, the entire tile is covered.
        public readonly int _size;
        public readonly int numBins;
        public readonly int binsX;
        public readonly int binsY;
        public const int pixelInitialValue = 0;
        public const float depthInitialValue = float.MaxValue;

        //bins are triangle buffer size, as well as _size in dimension (_size x _size)

        public static int binDimension = 64;
        public float[] tileMinDepth; // length = numBins
        public float[] tileMaxDepth; // length = numBins
        public Bin[] bins;

        public void BinXY(int binNum, out int x, out int y)
        {
            x = binNum % binsX;
            y = binNum / binsX;
        }

        //note: WithElement<T>(Vector<T>, Int32, T) --> can set a specific element...
        public FrameBuffer(int width, int height)
        {
            this.width = width;
            this.height = height;
            numBins = width * height / (binDimension * binDimension);
            _size = width * height;
            pixels = new uint[_size];
            depth = new float[_size];
            //a bitmask with MSB = lower x and LSB = higher x
            coverage = new Int64[_size/64 + 1];
            tileCoverage = new long[_size / 4096];

            bins = new Bin[numBins];
            for(int i = 0; i < numBins; i++)
            {
                bins[i] = new Bin();
            }
            tileMinDepth = new float[numBins];
            tileMaxDepth = new float[numBins];
            binsX = width / binDimension; 
            binsY = height / binDimension;
            hiZBuffer = new DepthLevel[4];
            hiZBuffer[0] = new DepthLevel(width, height, 1, 0);
            hiZBuffer[1] = new DepthLevel(width, height, 1, 1);
            hiZBuffer[2] = new DepthLevel(width, height, 2, 1);
            hiZBuffer[3] = new DepthLevel(width, height, 2, 2);
            //we should initialize the cellxymax to the first child cell in the parent cell, in child cell xy coords
            for(int i = 0; i < 4; i++)
            {
                InitializeDepthLevelMaxCellXY(i);
            }
        }

        public void InitializeDepthLevelMaxCellXY(int depthLevel)
        {
            Debug.Assert((depthLevel >= 0) && (depthLevel < 4));
            ref DepthLevel depthBuffer = ref hiZBuffer[depthLevel];
            int reductionX = 0;
            int reductionY = 0;
            if(depthLevel > 0)
            {
                reductionX = hiZBuffer[depthLevel - 1].reductionX;
                reductionY = hiZBuffer[depthLevel - 1].reductionY;
            }            
            for (int x = 0; x < depthBuffer.numCellsInX; x++)
            {
                for(int y = 0; y < depthBuffer.numCellsInY; y++)
                {
                    int levelElementID = x + (y * depthBuffer.numCellsInX);
                    ElementIDToScreenCoords(levelElementID, depthLevel, out int screenX, out int screenY);
                    if(depthLevel == 0)
                    {
                        depthBuffer.cellXYOfMax[levelElementID] = new Coord2D(screenX, screenY);
                    }
                    else
                    {
                        int cellX = XScreenToCellCoord(screenX, reductionX);
                        int cellY = YScreenToCellCoord(screenY, reductionY);
                        depthBuffer.cellXYOfMax[levelElementID] = new Coord2D(cellX, cellY);
                    }                    
                }
            }
        }

        public void ClearFrame()
        {

        }

        //we need a CLEAR FRAME function...

        public void SetPixel(int x, int y, uint color)
        {            
            pixels[width*y + x] = color;            
        }

        public void SetDepth(int x, int y, float inDepth)
        {
            depth[width * y + x] = inDepth;
        }

        public void SetCoverage(int x, int y)
        {
            SetCoverage(width * y + x);
        }

        public void SetCoverage(int pixelNum)
        {
            int wordNum = pixelNum / 64;
            int pixelOffset = 63 - (pixelNum % 64);
            Int64 mask = 1L << pixelOffset;
            Int64 newCoverage = coverage[wordNum] | mask;
            SetCoverage64(pixelNum, newCoverage);
            //coverage[wordNum] |= mask;
        }

        //the byte cannot cross a 64 bit aligned boundary
        //the pixelNum must be 8-bit aligned
        public void SetCoverage8(int pixelNum, byte coverage)
        {
            Debug.Assert((pixelNum % 8) == 0, "SetCoverage8 requires pixelNum % 8 == 0");
            Debug.Assert((pixelNum % 64) <= 56, "SetCoverage8 cannot cross 64-bit boundary");            
            int wordnum = pixelNum / 64;
            int pixelOffset = 63 - (pixelNum % 64);            
            Int64 mask = (Int64)(coverage) << (pixelOffset - 7);
            Int64 newCoverage64 = this.coverage[wordnum] | mask;
            //this.coverage[wordnum] |= mask;
            SetCoverage64(pixelNum, newCoverage64);
        }

        public Int64 SetBit(Int64 target, int bitNum)
        {            
            Int64 mask = 1L << (63 - bitNum);
            return target | mask;
        }

        public void SetCoverage64(int pixelNum, Int64 coverage)
        {
            int wordnum = pixelNum / 64;
            this.coverage[wordnum] |= coverage;
            //get tile num from pixel num
            //convert pixelnum to x, y
            
            //i... don't think this is right?
            if(coverage == -1)
            {
                int tileWidth = width / 64;
                int x = pixelNum % width;
                int y = pixelNum / width;
                int tileNumOffset = x / 64;
                int tileNum = y / 64 * tileWidth + tileNumOffset;
                int yOffset = y % 64;
                tileCoverage[tileNum] = SetBit(tileCoverage[tileNum], yOffset);
            }            
        }

        public bool GetCoverage(int pixelNum)
        {
            int wordnum = pixelNum / 64;
            int pixelOffset = 63 - (pixelNum % 64);
            Int64 mask = 1L << pixelOffset;
            return ((coverage[wordnum] & mask) != 0);
        }

        //true if fully covered, false otherwise
        public bool GetTileCoverage(int tileNum)
        {
            return (tileCoverage[tileNum] == -1);
        }


        //--BEGIN HI-Z HELPER FUNCTIONS--
        //reduction in power of 8.
        public int XScreenToCellCoord(int XScreencoord, int reductionExp8)
            => XScreencoord >> (3 * reductionExp8);

        public int YScreenToCellCoord(int YScreencoord, int reductionExp8)
            => YScreencoord >> (3 * reductionExp8);

        public int XCellToFirstScreenCoord(int XCell, int reductionExp8)
            => XCell << (3 * reductionExp8);

        public int YCellToFirstScreenCoord(int YCell, int reductionExp8)
            => YCell << (3 * reductionExp8);

        public int CellCoordsToElementNumber(int xCell, int yCell, int levelNum)                             
            => xCell  + (yCell * hiZBuffer[levelNum].numCellsInX);


        public int GetHiZElementID(int screenX, int screenY, int level)
        {
            int cellXParent = XScreenToCellCoord(screenX, hiZBuffer[level].reductionX);
            int cellYParent = YScreenToCellCoord(screenY, hiZBuffer[level].reductionY);
            return hiZBuffer[level].numCellsInX * cellYParent + cellXParent;
        }

        
        public void UpdateHiZSIMD(Vector<float> depthToBeAtDest, int x, int y)
        {
            Debug.Assert((x % 8 == 0), "UpdateHiZSIMD must write on SIMD-wide boundaries (e.g., %8 = 0)");
            //First: if we've changed the max for L0 (SIMD row), then we check if that row 
            //was the max for the corresponding L1 element.
            //if so, we need to invalidate the corresponding L1 element because it's max depth is no longer correct.
            //Thus, we:
            //1. Store L0's old max
            int elementID = GetHiZElementID(x, y, 0);
            float oldMaxDepthL0 = hiZBuffer[0].depthData[elementID];
            //2. calculate max depth for L0 chunk.
            
            float maxDepthL0 = depthToBeAtDest[0];
            int maxLaneID = 0;
            for(int i = 1; i < Constants.SIMDCount; i++)
            {
                float candidateMax = depthToBeAtDest[i];
                if ((candidateMax > maxDepthL0))
                {
                    maxDepthL0 = candidateMax;
                    maxLaneID = i;
                }                
            }
            hiZBuffer[0].validData[elementID] = true;
            hiZBuffer[0].depthData[elementID] = maxDepthL0;
            
            Coord2D childCoords = new Coord2D(x + maxLaneID, y);
            hiZBuffer[0].cellXYOfMax[elementID] = childCoords;

            int childX = XScreenToCellCoord(x, hiZBuffer[0].reductionX);
            int childY = YScreenToCellCoord(y, hiZBuffer[0].reductionY);
            childCoords = new Coord2D(childX, childY);
            
            int elementIDL1 = GetHiZElementID(x, y, 1);
            Coord2D maxElementCoordsL1 = hiZBuffer[1].cellXYOfMax[elementIDL1];
            bool invalidatedLast = false;
            if ((maxDepthL0 != oldMaxDepthL0) && (maxElementCoordsL1 == childCoords))
            {
                //if it used to be true, we are invalidating it
                invalidatedLast = hiZBuffer[1].validData[elementIDL1];
                hiZBuffer[1].validData[elementIDL1] = false;                
            }
            childX = XScreenToCellCoord(x, hiZBuffer[1].reductionX);
            childY = YScreenToCellCoord(y, hiZBuffer[1].reductionY);
            childCoords = new Coord2D(childX, childY);

            //If we invalidated the last level
            //AND
            //the cell coords of the last level are the same as the stored
            //cell coords for the element of the current level
            //THEN
            //we invalidate the current level            
            for (int parentLevel = 2; parentLevel <= 3; parentLevel++)
            {
                if (!invalidatedLast) break;
                int parentElementID = GetHiZElementID(x, y, parentLevel);
                //if the current (parent) level is invalid, we dno't need to do anything else - you can't invalidate a valid element
                if (!hiZBuffer[parentLevel].validData[parentElementID]) break;
                
                if (hiZBuffer[parentLevel].cellXYOfMax[parentElementID] != childCoords) break;                                
                //if we've invalidated the previous parent, the current parent has valid data, and the previous
                //parent's cell coords are the same as the max for the current level, then we need to invalidate
                hiZBuffer[parentLevel].validData[parentElementID] = false;
                //Finally, we need to set the new child coords for the next loop, which is just the coords of the current element
                //in the current parent.
                childX = XScreenToCellCoord(x, hiZBuffer[parentLevel].reductionX);
                childY = YScreenToCellCoord(y, hiZBuffer[parentLevel].reductionY);
                childCoords = new Coord2D(childX, childY);
            }
        }               

        public int ScreenCoordsToElementID(int x, int y)
        {
            return y * width + x;
        }

        //returns the linear element ID
        private int ElementIDToScreenCoords(int elementID, int level, out int x, out int y)
        {
            Debug.Assert(level >= 0 && level < hiZBuffer.Length, "Invalid hiZ level");

            DepthLevel dl = hiZBuffer[level];

            Debug.Assert(elementID >= 0 && elementID < dl.sizeInCells, "elementID out of range");

            int cellX = elementID % dl.numCellsInX;
            int cellY = elementID / dl.numCellsInX;

            // Top-left pixel of this hi-z cell in screen space
            x = cellX * dl.trueReductionX;
            y = cellY * dl.trueReductionY;

            Debug.Assert(x >= 0 && x < width, "Computed x out of framebuffer bounds");
            Debug.Assert(y >= 0 && y < height, "Computed y out of framebuffer bounds");

            return y * width + x;
        }
               
        
        //OK I THINK THIS IS OK BUT CHECK
        public float GetHiZ(int level, int cellX, int cellY)
        {
            int screenX = XCellToFirstScreenCoord(cellX, hiZBuffer[level].reductionX);
            int screenY = YCellToFirstScreenCoord(cellY, hiZBuffer[level].reductionY);
            int elementID = GetHiZElementID(screenX, screenY, level);
            if (hiZBuffer[level].validData[elementID])
            {
                return hiZBuffer[level].depthData[elementID];
            }
            else
            {                
                int relativeReductionXPow2 = 3 * (hiZBuffer[level].reductionX - hiZBuffer[level - 1].reductionX);
                int relativeReductionYPow2 = 3 * (hiZBuffer[level].reductionY - hiZBuffer[level - 1].reductionY);

                int relative2DReductionPow2 = relativeReductionXPow2 + relativeReductionYPow2;
                int numChildCellsInParentCellXDirection = 1 << relativeReductionXPow2;
                int numChildCellsInParentCellYDirection = 1 << relativeReductionYPow2;
                int numLanesInCell = 1 << relative2DReductionPow2;

                int cellXParent = cellX;
                int cellYParent = cellY;

                int cellXChild = XScreenToCellCoord(screenX, hiZBuffer[level - 1].reductionX);
                int cellYChild = YScreenToCellCoord(screenY, hiZBuffer[level - 1].reductionY);

                //To identify the cell coordinates of the first lane, first convert from screen x, screen y to the lane's (child) level
                //Then % that number by true reduction for both cell x and cell y.  not the actual true reduction but the RELATIVE
                //true reduction.

                float maxSeenDepth = float.MinValue;
                int cellXOfMax = 0;
                int cellYOfMax = 0;

                for (int xIncrement = 0; xIncrement < numChildCellsInParentCellXDirection; xIncrement++)
                {
                    int x = xIncrement + cellXChild;
                    for (int yIncrement = 0; yIncrement < numChildCellsInParentCellYDirection; yIncrement++)
                    {
                        int y = yIncrement + cellYChild;
                        float cellDepth = GetHiZ(level - 1, x, y);
                        if(cellDepth > maxSeenDepth)
                        {
                            maxSeenDepth = cellDepth;
                            cellXOfMax = x;
                            cellYOfMax = y;
                        }
                    }
                }
                hiZBuffer[level].depthData[elementID] = maxSeenDepth;
                hiZBuffer[level].validData[elementID] = true;
                hiZBuffer[level].cellXYOfMax[elementID] = new Coord2D(cellXOfMax, cellYOfMax);

                return maxSeenDepth;                
            }
        }
        
        
        public void SetPixelParallel(int xStart, int y, int pixelNum,
                                     Vector<int> depthAndCoveragemask, Vector<float> depthAndCoverageMaskedDepth, Vector<uint> color)
        {            
            Debug.Assert((pixelNum % width) == xStart, $"pixelNum/xStart mismatch");
            int SIMDSize = Vector<float>.Count;
            if (pixelNum >= _size) return;            
            if(xStart + SIMDSize <= width)
            {
                //here we need to calculate a final mask based on 
                //the depth test...

                Vector<uint> uIntMask = Vector.AsVectorUInt32(depthAndCoveragemask);
                
                Vector<uint> destPixels = new Vector<uint>(pixels, pixelNum);
                var comboPixels = Vector.ConditionalSelect(uIntMask, color, destPixels);
                
                Vector<float> destDepths = new Vector<float>(depth, pixelNum);
                var newFBDepths = Vector.ConditionalSelect(depthAndCoveragemask, depthAndCoverageMaskedDepth, destDepths);

                byte coverageMask = 0;
                for(int i = 0; i < Vector<int>.Count; i++)
                {
                    coverageMask <<= 1;
                    coverageMask |= depthAndCoveragemask[i] == -1 ? (byte)1 : (byte)0;
                }
                SetCoverage8(pixelNum, coverageMask);

                unsafe
                {
                    fixed (uint* _pixels = &pixels[pixelNum])
                    {
                        Vector.Store(comboPixels, _pixels);
                    }   
                    fixed(float* _depths = &depth[pixelNum])
                    {
                        Vector.Store(newFBDepths, _depths);
                    }
                }
                //DO THE HI-Z UPDATES
                //here, "mask" already has the depth test encoded in it.
                //also, we will only get here if there is at least one update...

                UpdateHiZSIMD(newFBDepths, xStart, y);
            }                         
        }

        public void Fill(byte a, byte r, byte g, byte b)
        {
            uint color = ((uint)a << 24) + ((uint)r << 16) + ((uint)g << 8) + (uint)b;
            Fill(color);
        }

        public void Fill(uint color)
        {
            Array.Fill(pixels, color);            
        }

        public void ClearDB()
        {
            Array.Fill(depth, float.MaxValue);
            Array.Fill(tileMinDepth, float.MaxValue);
            Array.Fill(tileMaxDepth, float.MaxValue);
            for(int i = 0; i < 4; i++) 
            {
                hiZBuffer[i].Reset();
            }
            //also need to clear hiz...
        }

        public void ClearCoverage()
        {
            Array.Fill(coverage, 0);
            Array.Fill(tileCoverage, 0);
        }

        public void ValidateFrame()
        {
            for (int i = 0; i < pixels.Length; i++)
            {                
                if (pixels[i] != pixelInitialValue)
                {
                    //Console.WriteLine($"pixel {i} is not equal to initial value");
                    if (depth[i] == depthInitialValue)
                    {
                        throw new InvalidDataException("pixel buffer has a value but depth is not written");
                    }
                    //also check coverage
                    if(!GetCoverage(i))
                    {                        
                        throw new InvalidDataException("coverage is not set for a pixel that is set");
                        
                    }
                }
                if (depth[i] != depthInitialValue)
                {
                    if (pixels[i] == pixelInitialValue)
                    {
                        throw new InvalidDataException("depth is set but pixels are not drawn");
                    }
                }
                if(GetCoverage(i))
                {
                    if (pixels[i] == pixelInitialValue)
                    {
                        throw new InvalidDataException("coverage is set but pixel isn't");
                    }
                    if (depth[i] == depthInitialValue)
                    {
                        throw new InvalidDataException("coverage is set but depth isn't");
                    }

                }
            }
            int numTiles = pixels.Length / 4096;
            //validate all coverages
            //tilecoverage looks like this:
            //c0: if b0 b1 ... b63 all 1's, 1, else 0
            //c1: if b0 b1 ... b63 all 1's, 1, else 0
            //..
            //c63: if b0 b1 ... b63 all 1's, 1, else 0
            for (int i = 0; i < numTiles; i++)
            {
                int tileX = i % binsX;
                int tileY = i / binsX;
                //for each tile... go row by row.  for each row with all pixels set, check if coverage mask is correct
                int pixelOffsetX = tileX * 64;
                int pixelOffsetY = tileY * 64 * width;
                //check if entire tile is covered
                bool tileIsCovered = true;
                int pixelNum = pixelOffsetX + pixelOffsetY;
                for (int rowNum = 0; rowNum < 64; rowNum++)
                {                    
                    for(int n = 0; n < 64; n++)
                    {
                        if (!GetCoverage(pixelNum + n))
                        {
                            tileIsCovered = false;
                            break;
                        }
                    }
                    pixelNum += width;
                    if (tileIsCovered == false) break;                    
                }
                if(!tileIsCovered && GetTileCoverage(i))
                {
                    throw new InvalidDataException("tile coverage is marked as yes but not all pixels are marked as covered");
                }
                if(tileIsCovered && !GetTileCoverage(i))
                {
                    throw new InvalidDataException("all elements are marked as covered but tile is not marked as covered");
                }
            }
        }

    }
}
