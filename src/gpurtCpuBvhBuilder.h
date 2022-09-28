/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2020-2022 Advanced Micro Devices, Inc. All Rights Reserved.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *
 **********************************************************************************************************************/

#pragma once
#include "palInlineFuncs.h"
#include "gpurtInternal.h"
#include "gpurtBvhBuilder.h"
#include "gpurtCpuUtils.h"

namespace GpuRt
{

using NodePointer = uint32;

// =====================================================================================================================
// Helper class for CPU-side building and updating of acceleration structures.
class CpuBvhBuilder : public BvhBuilder
{
public:
    CpuBvhBuilder(
        Internal::Device*      const pDevice,
        const Pal::DeviceProperties& deviceProps,
        ClientCallbacks              clientCb,
        const DeviceSettings&        deviceSettings);

    struct ScratchOffsets
    {
        uint32 maxScratchPrimitives;                // Max number of ScratchPrimitives
        uint32 maxScratchBuildNodes;                // Max number of ScratchBuildNodes
        uint32 maxQbvhInternalNodes;                // Max number of ScratchUpdateNodes

        uint32 scratchPrimsOffset;                  // All:    ScratchPrimitive leaf buffer
        uint32 sortedPrimIndicesOffset;             // Build:  Buffer of unsorted-to-sorted primitive indices
        uint32 scratchBuildNodesOffset;             // Build:  ScratchBuildNode buffer
        uint32 scratchUpdateNodesOffset;            // Update: ScratchUpdateNode buffer
        uint32 updateParentsOffsets[2];             // Update: Parent pointer double-buffer for updates
        uint32 updateLeafsToPrimMappingOffset;      // Update: Leafs node to ScratchPrimitive mapping array
    };

    static uint32 CalculateScratchBufferInfo(
        const BuildConfig&    buildConfig,
        ScratchOffsets*       pOffsets);

    static uint32 CalculateUpdateScratchBufferInfo(
        const BuildConfig&    buildConfig,
        ScratchOffsets*       pOffsets);

    // Builds or updates an acceleration structure and stores it in a result buffer
    void BuildRaytracingAccelerationStructure(const AccelStructBuildInfo& buildInfo) override;

    // Emit postbuild info
    void EmitAccelerationStructurePostBuildInfo(const AccelStructPostBuildInfo& postBuildInfo);

    void EmitASSerializationType(const AccelStructPostBuildInfo& postBuildInfo);

    void EmitASCompactedType(const AccelStructPostBuildInfo& postBuildInfo);

    uint32 CalcCompactedSize(
        const AccelStructHeader* pSrcHeader,
        AccelStructDataOffsets*  pDstOffsets,
        uint32*                  pMetadataSizeInBytes);

    // Copy Acceleration Structure
    void CopyAccelerationStructure(const AccelStructCopyInfo& copyArgs);

    void CopyASSerializeMode(const AccelStructCopyInfo& copyArgs);

    void CopyASCloneMode(const AccelStructCopyInfo& copyArgs);

    void CopyASCompactMode(const AccelStructCopyInfo& copyArgs);

    void CopyASDeserializeMode(const AccelStructCopyInfo& copyArgs);

private:
    using UintBoundingBox           = Shaders::UintBoundingBox;
    using TriangleData              = Shaders::TriangleData;
    using InstanceNode              = Shaders::InstanceNode;
    using float3                    = Shaders::float3;

    struct CpuBuildConfig
    {
        uint32               buildNodeCount;            // Number of allocated build scratch nodes
        uint32               leafNodeSize;              // Byte size of a leaf node
        uint32               internalNodeType;          // Pointer type of a box node
        uint32               internalNodeSize;          // Byte size of a box node
        float                sahCostTraversal;          // SAH: cost traversal/intersect ratio
        uint32               sahMinPrimitives;          // SAH: minimum number of prims before
                                                        // punting to simple insertion sort
        uint32               sahBucketCount;            // SAH: Number of candidate split points
                                                        // to minimize SAH cost over.
    };

    struct ScratchPrimitive
    {
        Aabb        bound;       // Bounds for this leaf primitive
        float3      centroid;    // Leaf primitive centroid
        NodePointer nodePointer; // Pointer to assigned leaf node
    };

    struct ScratchBuildNode
    {
        Aabb   bound;           // Interior node bound
        uint32 child[2];        // Two child indices to ScratchBuildNode array
        uint32 firstPrimitive;  // If leaf node, index range into sorted primitive buffer of leaf primitives
        uint32 primCount;       // If non-zero, this is a leaf node
    };

    struct ScratchUpdateNode
    {
        uint32 touched;
        Aabb   bound;
    };

    struct FaceData
    {
        uint32 primId; // PrimitiveID of the triangle in the Ngon
        uint32 vOff;   // Used to specify the vertex rotation of the triangle (used for barycentrics)
    };

    // An Ngon is a symbolic representation of the 4 triangle nodes containing 4 triangles and 5 vertices.
    struct Ngon
    {
        uint32   inds[5];
        FaceData f[4];
        float3   v[5];
    };

    static uint32 CalculateScratchBufferInfo(
        bool                            update,
        uint32                          aabbCount,
        TriangleCompressionMode         triCompressionMode,
        ScratchOffsets*                 pOffsets);

    AccelStructHeader* ResultHeader()
    {
        return static_cast<AccelStructHeader*>(
            Util::VoidPtrInc(m_buildArgs.pDstAccelStructCpuAddr, m_metadataSizeInBytes));
    }

    const AccelStructHeader* SourceHeader()
    {
        const AccelStructHeader* pHeader;

        if ((IsUpdate() == false) || IsInPlace())
        {
            pHeader = ResultHeader();
        }
        else
        {
            PAL_ASSERT(m_buildArgs.pSrcAccelStructCpuAddr != nullptr);

            pHeader = static_cast<const AccelStructHeader*>(
                Util::VoidPtrInc(m_buildArgs.pSrcAccelStructCpuAddr, m_metadataSizeInBytes));
        }

        return pHeader;
    }

    const void* SourceMetaData()
    {
        const void* pMetaData;

        if ((IsUpdate() == false) || IsInPlace())
        {
            pMetaData = ResultMetaData();
        }
        else
        {
            PAL_ASSERT(m_buildArgs.pSrcAccelStructCpuAddr != nullptr);

            pMetaData = Util::VoidPtrInc(m_buildArgs.pSrcAccelStructCpuAddr,
                                         sizeof(AccelStructMetadataHeader));
        }

        return pMetaData;
    }

    void* ResultMetaData()
    {
        PAL_ASSERT(m_buildArgs.pDstAccelStructCpuAddr != nullptr);

        return Util::VoidPtrInc(m_buildArgs.pDstAccelStructCpuAddr,
                                sizeof(AccelStructMetadataHeader));
    }

    AccelStructMetadataHeader* ResultMetadataHeader() const
    {
        return static_cast<AccelStructMetadataHeader*>(m_buildArgs.pDstAccelStructCpuAddr);
    }

    const AccelStructMetadataHeader* SourceMetadataHeader() const
    {
        const AccelStructMetadataHeader* pHeader;

        if ((IsUpdate() == false) || IsInPlace())
        {
            pHeader = ResultMetadataHeader();
        }
        else
        {
            PAL_ASSERT(m_buildArgs.pSrcAccelStructCpuAddr != nullptr);

            pHeader = static_cast<const AccelStructMetadataHeader*>(m_buildArgs.pSrcAccelStructCpuAddr);
        }

        return pHeader;
    }

    template<typename T> inline T* ScratchBuffer(uint32 offset = 0)
    {
        return static_cast<T*>(ScratchBufferCpuAddr(offset));
    }

    template<typename T> inline T* ResultBuffer(uint32 offset = 0)
    {
        return static_cast<T*>(ResultBufferCpuAddr(offset));
    }

    template<typename T> inline const T* SourceBuffer(uint32 offset = 0)
    {
        return static_cast<const T*>(SourceBufferCpuAddr(offset));
    }

    void* ResultBufferCpuAddr(uint32 offset = 0)
    {
        PAL_ASSERT(m_buildArgs.pDstAccelStructCpuAddr != nullptr);

        return Util::VoidPtrInc(m_buildArgs.pDstAccelStructCpuAddr, m_metadataSizeInBytes + offset);
    }

    const void* SourceBufferCpuAddr(uint32 offset = 0)
    {
        PAL_ASSERT(m_buildArgs.pSrcAccelStructCpuAddr != nullptr);

        return Util::VoidPtrInc(m_buildArgs.pSrcAccelStructCpuAddr, m_metadataSizeInBytes + offset);
    }

    void* ScratchBufferCpuAddr(uint32 offset = 0)
    {
        PAL_ASSERT(m_buildArgs.scratchAddr.pCpu != nullptr);

        return Util::VoidPtrInc(m_buildArgs.scratchAddr.pCpu, offset);
    }

    bool IsInPlace() const
    {
        return (SourceBufferBaseVa() == ResultBufferBaseVa()) ||
               (m_buildArgs.pSrcAccelStructCpuAddr == nullptr);
    }

    static bool IsActive(TriangleData tri);

    static bool IsActive(Aabb aabb);

    Aabb ComputeInstancedBoundingBox(
        const InstanceDesc&      desc,
        const AccelStructHeader* pBlas);

    void EncodeTriangleNodes(
        uint32                   primitiveOffset,
        const GeometryTriangles* pDesc,
        uint32                   primitiveCount,
        uint32                   geometryIndex,
        GeometryFlags            geometryFlags);

    void EncodeAABBNodes(
        uint32                primitiveOffset,
        const GeometryAabbs*  pDesc,
        uint32                primitiveCount,
        uint32                geometryIndex,
        GeometryFlags         geometryFlags);

    void EncodeInstances(
        const void*        pInstanceCpuAddr,
        uint32             numDesc,
        InputElementLayout descLayout);

    void BuildAccelerationStructure(
        uint32          primitiveCount,
        AccelStructType type,
        uint32          size);

    void UpdateAccelerationStructure(
        uint32          primitiveCount,
        AccelStructType type,
        uint32          size);

    void FindBestNgonTwoTriangles(
        Ngon*               pNgonArray,
        uint32*             pNgonCount,
        uint32              numIndices,
        uint32              primitiveIndex,
        const uint32        ind[3],
        const TriangleData& tri);

    bool CompareVertices(
        uint32        vertexIndex0,
        const float3& vertexPosition0,
        uint32        vertexIndex1,
        const float3& vertexPosition1);

    const ScratchBuildNode& GetBuildNode(uint32 nodeIndex) const;

    uint32 ConvertTwoToFourChildren(uint32 rootIndex, uint32 childIndices[4]);

    uint32 BuildTree(uint32 startPrim, uint32 endPrim);

    uint32 EncodeTree(uint32 nodeScratchIndex);

    void WriteGeometryInfo(
        uint32 geometryIndex,
        uint32 geometryFlags,
        uint32 numPrimitives,
        uint32 primNodePtrsOffset);

    void WriteParentPointer(NodePointer nodePtr,
                            NodePointer parentPtr);

    void WriteParentNodeChildPointer(
        const void* pSrcBuffer,
        void*       pDstBuffer,
        NodePointer srcNodePointer,
        NodePointer dstNodePointer);

    uint32 ReadParentPointer(NodePointer nodePtr);

    void WriteProceduralNode(NodePointer nodePointer,
                             const Aabb& bound,
                             uint32      primitiveIndex,
                             uint32      geometryIndex,
                             uint32      geometryFlags);

    NodePointer AddBoxNodeForSingleLeaf(NodePointer leafRoot);

    NodePointer AcquirePrimitiveLeaf(uint32 leafType, uint32 primCount);
    NodePointer AcquireInternalNode();
    uint32 AcquireScratchBuildNode();

    uint32 CalcLeafNodeOffset(uint32 leafByteOffset, uint32 primitiveIndex);

    void UpdateSceneBounds(const Aabb& bound);

    void CopyFp32BoxNode(
        const void* pSrcBuffer,
        void*       pDstBuffer,
        uint32      nodeOffset,
        uint32      srcOffsetDataInternalNodes,
        uint32      srcMetadataSizeInBytes,
        uint32      dstOffsetDataInternalNodes,
        uint32      dstMetadataSizeInBytes);

    void WriteInternalBoxNode(
        NodePointer pointer,
        NodePointer childPointers[4],
        Aabb        bounds[4]);

    NodePointer UpdateParentPointer(
        NodePointer srcNodePointer,
        NodePointer dstNodePointer);

    void WriteInstanceLeafNode(
        uint32              instance,
        uint32              instanceIndex,
        const InstanceDesc& desc);

    void WriteTriangleNode0(
        NodePointer nodePointer,
        float3      v0,
        float3      v1,
        float3      v2,
        uint32      primitiveIndex,
        uint32      geometryIndex,
        uint32      geometryFlags);

    uint32 EncodeLeafPrimitives();

    void WritePrimitiveNodePointer(uint32      globalPrimitiveIndex,
                                   NodePointer nodePointer);

    NodePointer ReadPrimitiveNodePointer(uint32 globalPrimitiveIndex);

    uint32 CalcInteriorNodeIndex(NodePointer nodePointer);

    uint32 CalcLeafNodeGlobalPrimitiveIndex(NodePointer nodePointer);

    void AddActivePrimitive(uint32 globalPrimitiveIndex);

    void DetermineSAHParams();

    uint32 SortPrimitives(uint32 start, uint32 end);
    uint32 SortPrimitivesByLargestExtent(uint32 start, uint32 end);
    uint32 SortPrimitivesBySAH(uint32 start, uint32 end);

    void InitBuildConfig(const AccelStructBuildInfo& buildArgs);

    CpuBuildConfig           m_cpuConfig;               // Build Config for CPU
    AccelStructHeader        m_dstHeader;               // In-build result header
    ScratchOffsets           m_scratchOffsets;          // Offsets to scratch buffer
    ScratchPrimitive*        m_pPrimitives;             // Leaf primitive array
    uint32*                  m_pLeafsToPrims;
    ScratchBuildNode*        m_pBuildNodes;             // Builds: Scratch binary tree topology
    ScratchUpdateNode*       m_pUpdateNodes;            // Updates: Scratch bounds tree nodes
    uint32*                  m_pSortedPrims;
    NodePointer*             m_pUpdateParentBuffers[2]; // Parent pointer buffer for updates
};

};
