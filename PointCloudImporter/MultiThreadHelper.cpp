
#include <boost/filesystem/fstream.hpp>

#include <utility/RCMutex.h>
#include <utility/RCFilesystem.h>
#include <utility/RCLog.h>
#include <utility/RCAssert.h>

#include "PointCloudIndexer.h"
#include "MultithreadHelper.h"
#include "VoxelOctreeCreator.h"
#include "OctreeIntermediateNode.h"

using namespace std;
using namespace ambergris::RealityComputing::Common;
using namespace ambergris::RealityComputing::Import;
using namespace ambergris::RealityComputing::Utility;

namespace fs = boost::filesystem;

namespace {

    template<typename PointType>
    bool loadPoints(const std::wstring& filename, size_t ptCnt, std::vector<PointType>& points) {
    
        fs::ifstream inputStream( filename, std::ios::in | std::ios::binary );
        if( !inputStream.is_open() ) return false;

        try {
            points.resize(ptCnt);
            inputStream.read( ( char* ) &points[0], sizeof( PointType ) * ptCnt);
            inputStream.close();
        }
        catch (std::exception ex)
        {
            const std::string str = ex.what();
            Log::RCLog::getLogFile()->addMessage( RCString(L"MultiThreadBuildOctreeFunctor loadPoints: ") + RCString(str.c_str()) );
            return false;
        }

        if (inputStream.fail()) {
            return false;
        }

        //remove temp11 (TEMP_UMBRELLA_LEAF_NODE_EXTENSION) file
        return Filesystem::remove( filename );
    }


}

template<typename PointType>
MultiThreadBuildOctreeFunctor<PointType>::MultiThreadBuildOctreeFunctor(PointCloudIndexerInterface* indexer, double leafSplitStopSize, double minDistanceBetweenPoints, bool hasNormals, 
                                                                        bool bCalcultateNormalsFromUnstructuredData, std::wstring* errorLog,
                                                                        std::vector<std::vector<OctreeIntermediateNode<PointType>* >>* intermediateLeafNodeList)
                                                                        :m_indexer(indexer)
                                                                        ,m_LeafSplitStopSize(leafSplitStopSize)
                                                                        ,m_minDistanceBetweenPoints(minDistanceBetweenPoints)
                                                                        ,m_hasNormals(hasNormals)
                                                                        ,m_calculateNormalsFromUnstructuredData( bCalcultateNormalsFromUnstructuredData )
                                                                        ,m_errorLog(errorLog)
                                                                        ,m_intermediateLeafNodeList(intermediateLeafNodeList) 
{
    if (m_intermediateLeafNodeList->size() > 0)
        mLeafNodeNum = (int) ((m_intermediateLeafNodeList->size() -1) * UMBRELLA_LEAF_NODE_WIDTH + m_intermediateLeafNodeList->at(m_intermediateLeafNodeList->size()-1).size());
    else
        mLeafNodeNum = 0;
}

template<typename PointType>
MultiThreadBuildOctreeFunctor<PointType>::~MultiThreadBuildOctreeFunctor() {
}

template<typename PointType>
void MultiThreadBuildOctreeFunctor<PointType>::operator()(tbb::blocked_range<size_t>& range) const {

    auto cancelFunc = [this]() { return m_indexer->getCancelled();  };

    for (size_t i=range.begin(); i != range.end(); i++) {
        if (m_indexer->getCancelled())
            return ;

        //get the voxel container
        auto umbreallaLeafNode = (*m_intermediateLeafNodeList)[i/UMBRELLA_LEAF_NODE_WIDTH][i%UMBRELLA_LEAF_NODE_WIDTH];

        //load the points in this voxel
        std::vector<PointType> points; 
        if(!loadPoints(umbreallaLeafNode->m_fileName, static_cast<size_t>(umbreallaLeafNode->m_totalAmountOfPoints), points)) {
            (*m_errorLog) += L"MultiThreadBuildOctreeFunctor<PointType>::operator() Cannot load points";
            RCASSERTCORE( false, "can't load voxel pts" );
        }

        //initialize the octree creator for this voxel
        VoxelOctreeCreator<PointType> creator(points, Filesystem::basename(umbreallaLeafNode->m_fileName), m_indexer->getTempFolder(), umbreallaLeafNode->m_svoBounds,
            8, m_LeafSplitStopSize,  m_minDistanceBetweenPoints, m_indexer->hasNormals(), m_indexer->hasTimeStamp(), true, cancelFunc);

        if (!m_hasNormals)
            creator.setCalculateNormalsFromUnstructuredData(m_calculateNormalsFromUnstructuredData);

        //create the octree and update the voxel metadata
        if (creator.createOctree()){
            creator.saveToDisk(umbreallaLeafNode);
        }
    }
}

namespace ambergris { namespace RealityComputing { namespace Import
{
    template class MultiThreadBuildOctreeFunctor<BasicOctreeImportPoint>;
    template class MultiThreadBuildOctreeFunctor<ExtendedOctreeImportPoint>;
}}}