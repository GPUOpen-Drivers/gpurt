// This file is part of the DGF-SDK.
//
// Copyright (C) 2025 Advanced Micro Devices, Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and /or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef DGF_DECOMPRESSION_HLSL
#define DGF_DECOMPRESSION_HLSL

#include "../../../shadersClean/common/Common.hlsli" // FIRSTBITHIGH_U64

#define DGF_CTRL_RESTART   0
#define DGF_CTRL_EDGE1     1
#define DGF_CTRL_EDGE2     2
#define DGF_CTRL_BACKTRACK 3
#define DGF_MAX_TRIS       64
#define DGF_HEADER_SIZE    20

uint Align(uint dw0, uint dw1, uint misalign)
{
    uint64_t pack = dw1;
    pack = (pack << 32) | dw0;
    return uint(pack >> (misalign & 31));
}

struct DGFHeader
{
    uint3 bitsPerComponent;
    uint numTriangles;
    uint numVerts;
    uint bitsPerIndex;
    int3 anchor;
    float scale;
    uint primIDBase;
    uint userData;
    uint bitSize;
    uint geomIDMeta;
    bool haveGeomIDPalette;
};

struct DGFBlockInfo
{
    DGFHeader header;
    ByteAddressBuffer dgfBuffer;
    uint blockStartOffset;
    uint bitsPerVertex;
    uint vertexBitStart;
    uint geomIDBitStart;
    uint indexBitStart;
};

DGFHeader DGFLoadHeader(ByteAddressBuffer dgfBuffer, uint blockstartOffset)
{
    DGFHeader result;
    const uint4 H = dgfBuffer.Load4(blockstartOffset);
    const uint h0 = H.x;
    const uint h1 = H.y;
    const uint h2 = H.z;
    const uint h3 = H.w;
    const uint2 H2 = dgfBuffer.Load2(blockstartOffset + 16);
    const uint h4 = H2.x;
    const uint h5 = H2.y;
    result.numTriangles = ((h0 >> 16) & 0x3f) + 1;
    result.numVerts = ((h0 >> 10) & 0x3f) + 1;
    result.bitsPerIndex = ((h0 >> 8) & 3) + 3;
    result.bitsPerComponent.x = (h2 & 0xf) + 1;
    result.bitsPerComponent.y = ((h2 >> 4) & 0xf) + 1;
    result.bitsPerComponent.z = (h3 & 0xf) + 1;
    result.anchor.x = ((int) h1) >> 8;
    result.anchor.y = ((int) h2) >> 8;
    result.anchor.z = ((int) h3) >> 8;
    result.scale = asfloat((h1 & 0xff) << 23);
    result.primIDBase = h4 & ((1 << 29) - 1);

    uint haveUserData = ((h4 >> 29) & 1);
    result.userData = haveUserData * h5;
    result.bitSize = 32 * (5 + haveUserData);

    result.haveGeomIDPalette = (h3 & 0x80) != 0;
    result.geomIDMeta        = h0 >> 22;
    return result;
}

uint ByteAlign(uint numBits)
{
    return (numBits + 7) & ~7;
}

uint ComputeGeomIDPaletteSize(DGFHeader header)
{
    uint geomIDMeta     = header.geomIDMeta;
    uint numGeomIDs     = (geomIDMeta >> 5) + 1;
    uint prefixBitSize  = (geomIDMeta & 0x1F);
    uint payloadBitSize = 25 - prefixBitSize;
    uint indexBitSize   = firstbithigh(numGeomIDs - 1) + 1;
    uint paletteSize    = ByteAlign(numGeomIDs * payloadBitSize + header.numTriangles * indexBitSize + prefixBitSize);

    return (header.haveGeomIDPalette) ? paletteSize : 0;
}

DGFBlockInfo DGFLoadBlockInfo(ByteAddressBuffer dgfBuffer, in uint dgfBlockIndex)
{
    DGFBlockInfo result;
    result.blockStartOffset = dgfBlockIndex * 128;
    result.dgfBuffer = dgfBuffer;
    result.header = DGFLoadHeader(result.dgfBuffer, result.blockStartOffset);
    result.bitsPerVertex  = result.header.bitsPerComponent.x + result.header.bitsPerComponent.y + result.header.bitsPerComponent.z;

    uint vertexBitSize     = ByteAlign(result.header.numVerts * result.bitsPerVertex);
    uint geomIDPaletteSize = ComputeGeomIDPaletteSize(result.header);

    result.vertexBitStart  = result.header.bitSize;
    result.geomIDBitStart  = result.vertexBitStart + vertexBitSize;
    result.indexBitStart   = result.geomIDBitStart + geomIDPaletteSize;

    return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////   Index fetch
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint LoadIndex(ByteAddressBuffer dgfBuffer, uint blockStartOffset, uint bitPos, uint bitsPerIndex)
{
    // indices are up to 6b, potentially spread across 2 dwords
    uint dwordPos = bitPos / 32;
    uint2 dw = dgfBuffer.Load2(blockStartOffset + 4 * dwordPos);
    uint M = (1ul << bitsPerIndex) - 1ul;
    return Align(dw.x, dw.y, bitPos) & M;
}

struct IsFirstScanState
{
    uint3 prefixCount;
    uint3 isFirst;
};

IsFirstScanState InitIsFirstScanState(const uint3 ibAddress)
{
    IsFirstScanState state;
    state.prefixCount.x = min(ibAddress.x, 3); // first 3 'isFirst' bits are implicit ones
    state.prefixCount.y = min(ibAddress.y, 3); // first 3 'isFirst' bits are implicit ones
    state.prefixCount.z = min(ibAddress.z, 3); // first 3 'isFirst' bits are implicit ones
    state.isFirst.x = ibAddress.x < 3;
    state.isFirst.y = ibAddress.y < 3;
    state.isFirst.z = ibAddress.z < 3;
    return state;
}

void IsFirstScan(inout IsFirstScanState state, uint indexStart, const uint3 ibAddress, uint bits)
{
    uint bitsX = (ibAddress.x < indexStart) ? 0 : bits; // don't count bits beyond the query position
    uint bitsY = (ibAddress.y < indexStart) ? 0 : bits;
    uint bitsZ = (ibAddress.z < indexStart) ? 0 : bits;

    uint offsetX = (ibAddress.x - indexStart);
    uint offsetY = (ibAddress.y - indexStart);
    uint offsetZ = (ibAddress.z - indexStart);
    uint maskX = (offsetX < 32) ? (1ul << offsetX) : 0;
    uint maskY = (offsetY < 32) ? (1ul << offsetY) : 0;
    uint maskZ = (offsetZ < 32) ? (1ul << offsetZ) : 0;

    // count number of 'first' indices that precede this IB position
    uint popCountX = countbits(bitsX & (maskX - 1));
    uint popCountY = countbits(bitsY & (maskY - 1));
    uint popCountZ = countbits(bitsZ & (maskZ - 1));
    state.prefixCount.x += popCountX;
    state.prefixCount.y += popCountY;
    state.prefixCount.z += popCountZ;

    // when we pass the query position, note whether or not it is an 'isFirst' index
    state.isFirst.x = (bitsX & maskX) || state.isFirst.x;
    state.isFirst.y = (bitsY & maskY) || state.isFirst.y;
    state.isFirst.z = (bitsZ & maskZ) || state.isFirst.z;
}

uint3 DemuxIndices(uint3 indices, DGFBlockInfo s)
{
    // maximum 'isFirst' size is 189b, potentially spread across 6 dwords
    //  This is an array of 1bit values ordered back to front
    uint numControlBits = 2 * (s.header.numTriangles - 1);
    uint isFirstStart = numControlBits / 32;
    uint ifBase = s.blockStartOffset + 4 * (31 - isFirstStart - 5);
    uint4 IF0 = s.dgfBuffer.Load4(ifBase);
    uint2 IF1 = s.dgfBuffer.Load2(ifBase + 16);
    uint f0 = reversebits(IF1.y);
    uint f1 = reversebits(IF1.x);
    uint f2 = reversebits(IF0.w);
    uint f3 = reversebits(IF0.z);
    uint f4 = reversebits(IF0.y);
    uint f5 = reversebits(IF0.x);

    // align the "is-first" bits
    uint if0 = Align(f0, f1, numControlBits);
    uint if1 = Align(f1, f2, numControlBits);
    uint if2 = Align(f2, f3, numControlBits);
    uint if3 = Align(f3, f4, numControlBits);
    uint if4 = Align(f4, f5, numControlBits);

    // scan the is-first bits to find the compressed index buffer lookup positions
    IsFirstScanState ifScan = InitIsFirstScanState(indices);
    IsFirstScan(ifScan, 3, indices, if0);
    IsFirstScan(ifScan, 35, indices, if1);
    IsFirstScan(ifScan, 67, indices, if2);
    IsFirstScan(ifScan, 99, indices, if3);
    IsFirstScan(ifScan, 131, indices, if4);

    // load any "non-first" indices
    uint3 localIndices;
    uint ibBase  = s.indexBitStart;
    uint bitPosX = ibBase + (indices.x - ifScan.prefixCount.x) * s.header.bitsPerIndex;
    uint bitPosY = ibBase + (indices.y - ifScan.prefixCount.y) * s.header.bitsPerIndex;
    uint bitPosZ = ibBase + (indices.z - ifScan.prefixCount.z) * s.header.bitsPerIndex;
    localIndices.x = LoadIndex(s.dgfBuffer, s.blockStartOffset, bitPosX, s.header.bitsPerIndex);
    localIndices.y = LoadIndex(s.dgfBuffer, s.blockStartOffset, bitPosY, s.header.bitsPerIndex);
    localIndices.z = LoadIndex(s.dgfBuffer, s.blockStartOffset, bitPosZ, s.header.bitsPerIndex);
    localIndices.x = ifScan.isFirst.x ? ifScan.prefixCount.x : localIndices.x;
    localIndices.y = ifScan.isFirst.y ? ifScan.prefixCount.y : localIndices.y;
    localIndices.z = ifScan.isFirst.z ? ifScan.prefixCount.z : localIndices.z;
    return localIndices;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////    Connectivity decode
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint Compact(uint x)
{
    x &= 0x55555555ul;
    x = (x ^ (x >> 1ul)) & 0x33333333ul;
    x = (x ^ (x >> 2ul)) & 0x0f0f0f0ful;
    x = (x ^ (x >> 4ul)) & 0x00ff00fful;
    x = (x ^ (x >> 8ul)) & 0x0000fffful;
    return x;
}

#define bits_t uint64_t

uint64_t Pack64(uint a, uint b, uint c, uint d)
{
    uint64_t q0 = (a | (b << 16ul));
    uint64_t q1 = (c | (d << 16ul));
    return (q1 << 32ul) | q0;
}

void ExtractBits(DGFBlockInfo s, out uint64_t evenBits, out uint64_t oddBits)
{
    uint4 Ctrl = s.dgfBuffer.Load4(s.blockStartOffset + 28ul * 4ul);
    uint ctrl0 = reversebits(Ctrl.w);
    uint ctrl1 = reversebits(Ctrl.z);
    uint ctrl2 = reversebits(Ctrl.y);
    uint ctrl3 = reversebits(Ctrl.x);
    evenBits = Pack64(Compact(ctrl0), Compact(ctrl1), Compact(ctrl2), Compact(ctrl3));
    oddBits = Pack64(Compact(ctrl0 >> 1ull), Compact(ctrl1 >> 1ull), Compact(ctrl2 >> 1ull), Compact(ctrl3 >> 1ull));
    evenBits <<= 1ull;
    oddBits <<= 1ull;
}

uint LoadTriangleControlValues(DGFBlockInfo s, uint triangleId)
{
    if (triangleId == 0)
    {
        return DGF_CTRL_RESTART;
    }

    // The first triangle control code is never stored, so we we need to " - 1" the triangleId to account for that.
    const uint storedTriangleId = triangleId - 1;
    const uint ctrlDWord = s.dgfBuffer.Load( // start-offset of each block:
                                            s.blockStartOffset +
                                            // the control bytes start here within each block:
                                            28 * 4 +
                                            // From the storedTriangleId, we compute the dword index of the code
                                            // (" / 16 ", because there are 16 codes per dword),
                                            // reverse it (that is the "3 - "),
                                            // and compute the byte offset (* 4)
                                            (3 - storedTriangleId / 16) * 4
                                          );
    // Within the ctrlDWord, we compute the index position of the triangle control value:
    // 2 *  : because there are 2 bits code
    // 15 - : because the codes are stored in reverse
    // storedTriangleId & 15: that is storedTriangleId % 16 which gives us triangleId within the DWord.
    const uint bitIdx = 2 * (15 - (storedTriangleId & 15));

    // >> bitIdx: Shift the bits of the control values into the lower two bits.
    // & 3: leave only the lower two bits.
    return (ctrlDWord >> bitIdx) & 3u;
}

bits_t SelectMask(const uint i)
{
    return (bits_t(-1)) >> (~i);
}

uint ComputeMBar(const bits_t isRestart, const uint i)
{
    bits_t m = isRestart & SelectMask(i);
    uint nRestarts = countbits(m);
    return i + 2 * nRestarts;
}

bits_t Select(const bits_t predicate, const bits_t yes, const bits_t no)
{
    return (predicate & yes) | ((~predicate) & no);
}

bits_t VirtualCurrArray(const bits_t isBacktrack, const bits_t isRestart, const bits_t isEdge2)
{
    return Select(isBacktrack, ((isEdge2) << 1), ~(isRestart | isEdge2));
}

bits_t CopyIfEdge2IsSet(const bits_t edge2IsSet)
{
    return ~edge2IsSet;
}

bits_t FlipIfEdge2IsSet(const bits_t edge2IsSet)
{
    return edge2IsSet;
}

bits_t VirtualPrevArray(const bits_t isBacktrack, const bits_t isRestart, const bits_t isEdge2, const bits_t virtualCurrOfPrev)
{
    const bits_t if1 = Select(isBacktrack & ((~isBacktrack) << 2), CopyIfEdge2IsSet(isEdge2 << 2), virtualCurrOfPrev << 1);
    const bits_t if0 = Select(isBacktrack & ((isBacktrack) << 2), FlipIfEdge2IsSet(isEdge2 << 3), if1);
    return isRestart | if0;
}

bits_t ComputeQArray(const bits_t isBacktrack, const bits_t isRestart, const bits_t isEdge2)
{
    const bits_t vc = VirtualCurrArray(isBacktrack, isRestart, isEdge2);
    const bits_t vp = VirtualPrevArray(isBacktrack, isRestart, isEdge2, vc);
    return ((vp ^ vc) & (~(isBacktrack >> 1)));
}

bool ReadBitUnsafe(const bits_t b, uint i)
{
    return (b >> i) & 1;
}

uint ComputeBias(const bits_t isBacktrack, const bits_t isRestart, uint i)
{
    const bool b0 = ReadBitUnsafe(isBacktrack, i);
    const bool b1 = ReadBitUnsafe(isBacktrack, i - 1);
    const bool b2 = ReadBitUnsafe(isBacktrack, i - 2);
    const bool r = ReadBitUnsafe(isRestart, i);
    return (b0 + (b0 & b2)) + ((!b0) & (!r & b1));
}

uint ComputeQBar(const bits_t isBacktrack, const bits_t isRestart, const bits_t qArray, const uint j)
{
    const bits_t x = qArray & SelectMask(j);
    const uint k = FIRSTBITHIGH_U64(x);
    const uint b = ComputeBias(isBacktrack, isRestart, k);
    const uint m = ComputeMBar(isRestart, k);
    return m - b - 2;
}

uint3 ControlScan(uint i, const bits_t isRestart, const bits_t isBacktrack, const bits_t isEdge2)
{
    const bits_t qArray = ComputeQArray(isBacktrack, isRestart, isEdge2);

    const bool next_isBacktrack = ReadBitUnsafe(isBacktrack, i + 1);

    const uint j = i - next_isBacktrack;
    const bool j_IsBacktrack = ReadBitUnsafe(isBacktrack, j);
    const bool j_IsEdge2 = j_IsBacktrack ^ ReadBitUnsafe(isEdge2, j - j_IsBacktrack);
    const bool j_IsRestart = ReadBitUnsafe(isRestart, j);

    const uint mBar = ComputeMBar(isRestart, j);
    const uint qBar = ComputeQBar(isBacktrack, isRestart, qArray, j);
    const uint b = 1 + j_IsBacktrack;

    const bool i_IsEdge2 = ReadBitUnsafe(isEdge2, i);

    uint3 indices;
    indices.x = (j_IsRestart ? mBar - 2 : (next_isBacktrack && !i_IsEdge2) ? mBar : (!j_IsEdge2 ? mBar - b : qBar));
    indices.y = (j_IsRestart ? mBar - 1 : (next_isBacktrack && i_IsEdge2) ? mBar : (j_IsEdge2 ? mBar - b : qBar));
    indices.z = mBar + next_isBacktrack;
    return indices;
}

DGFBlockInfo DGFInit(ByteAddressBuffer dgfBuffer, in uint dgfBlockIndex)
{
    return DGFLoadBlockInfo(dgfBuffer, dgfBlockIndex);
}

struct TriangleBits
{
    bits_t isRestart;
    bits_t isBacktrack;
    bits_t isEdge2;
};

uint64_t Wave64ActiveBallot(bool expr)
{
    uint4 b = WaveActiveBallot(expr);
    uint64_t r = b.y;
    r <<= 32;
    r |= b.x;
    return r;
}

TriangleBits GetTriangleBitsWaveBallot(DGFBlockInfo s)
{
    TriangleBits result;
    result.isRestart = 0;
    result.isBacktrack = 0;
    result.isEdge2 = 0;

    const uint WAVE_SIZE = WaveGetLaneCount();
    for (uint waveBase = 0; waveBase < DGF_MAX_TRIS; waveBase += WAVE_SIZE)
    {
        uint i = waveBase + WaveGetLaneIndex();
        const uint ctrl = LoadTriangleControlValues(s, i);

        // DGF blocks have a limit of 64 triangles.
        //  On a Wave128 architecture we only need the first 64
        result.isRestart |= (Wave64ActiveBallot(ctrl == DGF_CTRL_RESTART) << waveBase);
        result.isBacktrack |= (Wave64ActiveBallot(ctrl == DGF_CTRL_BACKTRACK) << waveBase);
        result.isEdge2 |= (Wave64ActiveBallot(ctrl == DGF_CTRL_EDGE2) << waveBase);
    }

    const bits_t mask = ((bits_t(1)) << s.header.numTriangles) - (bits_t) (1);
    result.isBacktrack &= mask;
    return result;
}

TriangleBits GetTriangleBitsSingleLane(DGFBlockInfo s)
{
    TriangleBits result;
    bits_t evenBits, oddBits;
    ExtractBits(s, evenBits, oddBits);
    const bits_t mask = ((bits_t(1)) << s.header.numTriangles) - (bits_t) (1);
    result.isRestart = ((~evenBits) & (~oddBits));
    result.isBacktrack = ((evenBits) & (oddBits)) & mask;
    result.isEdge2 = ((evenBits) & (~oddBits));
    return result;
}

// Decode a triangle using the full wave.
//  Must be called within wave-uniform control flow with a wave-uniform DGF block
uint3 DGFGetTriangle_BitScan_Wave(DGFBlockInfo s, uint triangleIndexInBlock)
{
    TriangleBits triangleBits = GetTriangleBitsWaveBallot(s);
    uint3 indices = ControlScan(triangleIndexInBlock, triangleBits.isRestart, triangleBits.isBacktrack, triangleBits.isEdge2);
    return DemuxIndices(indices, s);
}

// Decode a triangle using a single lane
uint3 DGFGetTriangle_BitScan_Lane(DGFBlockInfo s, uint triangleIndexInBlock)
{
    TriangleBits triangleBits = GetTriangleBitsSingleLane(s);
    uint3 indices = ControlScan(triangleIndexInBlock, triangleBits.isRestart, triangleBits.isBacktrack, triangleBits.isEdge2);
    return DemuxIndices(indices, s);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////    Vertex Fetch
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

float3 DGFGetVertex(DGFBlockInfo s, uint vertexIndex)
{
    // vertices are up to 48b, potentially spanning 3 dwords
    uint bitPos = s.vertexBitStart + vertexIndex * s.bitsPerVertex;
    uint dwordPos = bitPos / 32;
    uint3 f = s.dgfBuffer.Load3(s.blockStartOffset + 4 * dwordPos);
    uint dw0 = Align(f.x, f.y, bitPos); // align the fetch
    uint dw1 = Align(f.y, f.z, bitPos);

    // x and y are guaranteed to be in the lower dword.  Z may be straddling the boundary
    uint64_t vert = (((uint64_t) dw1) << 32) | dw0;
    int3 v = int3(dw0, dw0 >> s.header.bitsPerComponent.x, uint(vert >> (s.header.bitsPerComponent.x + s.header.bitsPerComponent.y)));

    const uint3 mask = (uint3(1, 1, 1) << s.header.bitsPerComponent) - uint3(1, 1, 1);
    v &= mask;
    v += s.header.anchor;
    return float3(v * s.header.scale);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////    Serial Connectivity Decode ( for reference )
//////////////      This algorithm is much slower than the bit-scan algorithm and should probably not be used
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct ControlScanState
{
    uint3 indices; // index buffer positions for last decoded triangle
    uint3 prev; // index buffer positions for the triangle before last
    uint lastCtrl; // control bit value of the last decoded triangle
    uint indexPos; // position in the index buffer
    uint restartCount;
};

ControlScanState InitControlScanState()
{
    // first triangle is an implicit restart.. indices are 0,1,2
    ControlScanState state;
    state.indices = uint3(0, 1, 2);
    state.indexPos = 3;
    state.lastCtrl = 0;
    state.prev = uint3(0, 0, 0);
    state.restartCount = 1;
    return state;
}

void ControlScan(inout ControlScanState s, uint triStart, uint triIndex, uint bits)
{
    uint i = 30; // control bit pairs are stored back to front
    for (uint tri = triStart; tri <= min(triIndex, triStart + 15); tri++)
    {
        uint ctrl = (bits >> i) & 3;
        i -= 2;
        uint3 prevPrev = s.prev;
        uint3 prev = s.indices;

        bool restart = (ctrl == DGF_CTRL_RESTART);
        bool backtrack = (ctrl == DGF_CTRL_BACKTRACK);
        uint px = (backtrack) ? prevPrev.x : prev.x; // choose base triangle based on backtrack
        uint py = (backtrack) ? prevPrev.y : prev.y;
        uint pz = (backtrack) ? prevPrev.z : prev.z;
        uint c = (backtrack) ? s.lastCtrl ^ 3 : ctrl; // if backtracking, use inverted ctrl values from last tri
        bool edge1 = c == DGF_CTRL_EDGE1;

        // select edge1 or edge2 from base triangle
        uint x = (edge1) ? pz : px;
        uint y = (edge1) ? py : pz;
        uint z = s.indexPos;

        // handle restart
        s.indices.x = (restart) ? s.indexPos : x;
        s.indices.y = (restart) ? s.indexPos + 1 : y;
        s.indices.z = (restart) ? s.indexPos + 2 : z;
        s.indexPos += (restart) ? 3 : 1;

        s.lastCtrl = ctrl;
        s.prev = prev;
    }
}

uint3 DGFGetTriangle_Serial(inout DGFBlockInfo s, uint triIndex)
{
    ControlScanState cscan = InitControlScanState();

    // at most four control-bit dwords
    // these are ordered back to front, but they contain 2-bit fields whose order isn't reversed
    uint4 Ctrl = s.dgfBuffer.Load4(s.blockStartOffset + 28 * 4);
    uint ctrl0 = Ctrl.w;
    uint ctrl1 = Ctrl.z;
    uint ctrl2 = Ctrl.y;
    uint ctrl3 = Ctrl.x;

    // scan the control bits to find the index buffer addresses for this triangle
    ControlScan(cscan, 1, triIndex, ctrl0);
    ControlScan(cscan, 17, triIndex, ctrl1);
    ControlScan(cscan, 33, triIndex, ctrl2);
    ControlScan(cscan, 49, triIndex, ctrl3);

    return DemuxIndices(cscan.indices, s);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////    GeomID Fetch
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Reads geometry ID and opacity from a DGF block.  X contains geomID.  Y contains opaque flag
uint2 DGFGetGeomIDAndOpacity(DGFBlockInfo s, uint triIndex)
{
    uint geomID = s.header.geomIDMeta; // if there is no palette, the meta field *is* the ID
    if (s.header.haveGeomIDPalette)
    {
        // read a per-triangle ID from the palette

        // parse palette parameters
        uint geomIDMeta = geomID;
        uint numGeomIDs     = (geomIDMeta >> 5) + 1;
        uint prefixBitSize  = (geomIDMeta & 0x1F);
        uint payloadBitSize = 25 - prefixBitSize;
        uint indexBitSize   = firstbithigh(numGeomIDs - 1) + 1;

        uint prefixBase   = s.geomIDBitStart;
        uint indicesBase  = prefixBase + prefixBitSize;
        uint payloadsBase = indicesBase + s.header.numTriangles * indexBitSize;

        // load the prefix value.  This is guaranteed to be byte aligned but may not be dword aligned
        uint prefix = LoadIndex(s.dgfBuffer, s.blockStartOffset, prefixBase, prefixBitSize);

        // load the index.  This is up to 5b, potentially spread across two dwords
        uint indexBitPos = indicesBase + indexBitSize * triIndex;
        uint geomIDIndex = LoadIndex(s.dgfBuffer, s.blockStartOffset, indexBitPos, indexBitSize);

        // load the payload.  Payloads are up to 25b, potentially spread across two dwords
        uint payloadBitPos = payloadsBase + geomIDIndex * payloadBitSize;
        uint geomIDPayload = LoadIndex(s.dgfBuffer, s.blockStartOffset, payloadBitPos, payloadBitSize);

        // reconstruct the geomID
        geomID = (prefix << payloadBitSize) + geomIDPayload;
    }

    // opaque flag is in the LSB
    return uint2(geomID >> 1, geomID & 1);
}

// Given a primitive ID, returns its DGF block index (x) and its position in the block (y)
uint2 DGFLookupBlockMap(ByteAddressBuffer dgfBlockMap, uint primID)
{
    // The block map buffer contains one dword for every 32 triangles.  The bits are laid out thusly:
    //
    //  X:
    //    uint32_t blockIndex      : 24;  // Index of block containing first triangle
    //    uint32_t positionInBlock : 6;   // position of the first triangle in the block
    //    uint32_t unused          : 2;
    //
    //  Y:
    //    uint32_t endOfBlock;    // For each triangle in the group, is it the last one in its block
    //
    //  This allows us to infer the location of any of the 32 triangles, at a memory overhead of 0.25 B/Tri.
    //

    uint blockMapIndex = primID / 32;
    uint2 mapVal = dgfBlockMap.Load2(8 * blockMapIndex);
    uint blockIndex = mapVal.x & 0x00FFFFFF;
    uint positionInBlock = (mapVal.x >> 24);
    uint eobMask = mapVal.y;

    uint blockMapMod = primID % 32;
    uint prefixMask = (1u << blockMapMod) - 1; // mask out eob bits for this triangle and above
    blockIndex += countbits(eobMask & prefixMask); // count number of eob tris prior to this one.  Step forward this much

    int boundary = firstbithigh(eobMask & prefixMask); // find the position of the most recent block transition
    positionInBlock = (boundary == -1) ? positionInBlock : 0; // reset position in block if there was a block transition
    positionInBlock += blockMapMod - boundary - 1; // figure out how far we are from start of our current block

    return uint2(blockIndex, positionInBlock);
}

#endif
