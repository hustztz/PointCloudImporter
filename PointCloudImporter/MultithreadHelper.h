#pragma once

#include <vector>

#include <tbb/task_scheduler_init.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#include "OctreeIntermediateNode.h"

namespace ambergris { namespace RealityComputing { namespace Import {

    class PointCloudIndexerInterface;

    template<typename PointType>
    class MultiThreadBuildOctreeFunctor
    {
    public:
        MultiThreadBuildOctreeFunctor(PointCloudIndexerInterface* indexer, double leafStopSplitSize, double minDistanceBetweenPoints,bool hasNormals,bool calculateNormalsFromUnstructuredData,
            std::wstring* errorLog, std::vector< std::vector< OctreeIntermediateNode<PointType> * > >* intermediateLeafNodeList);
        MultiThreadBuildOctreeFunctor(const MultiThreadBuildOctreeFunctor&) = default;
        ~MultiThreadBuildOctreeFunctor();

        void operator()(tbb::blocked_range<size_t>& range) const;

    private:
        PointCloudIndexerInterface*  m_indexer;
        double                  m_LeafSplitStopSize;
        double                  m_minDistanceBetweenPoints;
        bool                    m_hasNormals;
        bool                    m_calculateNormalsFromUnstructuredData;
        std::wstring*           m_errorLog;
        std::vector< std::vector< OctreeIntermediateNode<PointType>* > >*    m_intermediateLeafNodeList; 
        int                     mLeafNodeNum;
    };
}}}
