#include <stack>
#include <cassert>
#include <sstream>
#include <cmath>
#include <random>

#include <boost/filesystem/fstream.hpp>

#include "OctreeImportPoint.h"
#include "VoxelOctreeCreator.h"
#include "VoxelHelperNode.h"
#include "OctreeDefs.h"

#include <Urho3D/PointCloud/AgPointCloudOptions.h>

#include <common/RCString.h>
#include <common/RCPlane.h>

#include <utility/RCFilesystem.h>
#include <utility/RCLog.h>
#include <utility/RCMemoryUtils.h>

using namespace ambergris::PointCloudEngine;
using namespace ambergris::RealityComputing::Import;
using namespace ambergris::RealityComputing::Common;
using namespace ambergris::RealityComputing::Utility;
using namespace ambergris::RealityComputing::Utility::Log;

namespace fs = boost::filesystem;

namespace
{
    const unsigned int LEAF_RANDOMIZATION_SEED = 8675309;           // (https://www.youtube.com/watch?v=6WTdTwcmxyo)

    template <class T>
    void shuffleInputBuffer(std::vector<T>& data)
    {
        std::mt19937 gen;
        gen.seed(LEAF_RANDOMIZATION_SEED);
        std::shuffle(data.begin(), data.end(), gen);
    }
}

//////////////////////////////////////////////////////////////////////////
//Class VoxelCreator
//////////////////////////////////////////////////////////////////////////
template<typename PointType>
VoxelOctreeCreator<PointType>::VoxelOctreeCreator(std::vector<PointType>& points, const std::wstring& voxelName, const std::wstring& tempFolder, const ambergris::RealityComputing::Common::RCBox& svoBounds,
            int maxDepth, double stopSize, double minPointDistance, bool hasNormals, bool hasTimestamps, bool logOutput, const CancelCallback& cb) :
    m_startDepth(maxDepth),
    m_LeafStopSplitSize(stopSize),
    m_minPointDistance(minPointDistance),
    m_cancelCb(cb),
    m_fileName(tempFolder + voxelName + std::wstring( TEMP_VOXEL_OCTREE_EXTENSION )),
    m_timeStampFileName(tempFolder + voxelName + std::wstring( TEMP_VOXEL_TIMESTAMP_EXTENSION )),
    m_octreePoints(points),
    m_amountOfPoints(static_cast<int>(m_octreePoints.size())),
    m_outputToLog(logOutput),
    m_maxLeafDepth(0),
    mHasNormals(hasNormals),
    mHasTimestamps(hasTimestamps)
{
    m_rootPtr.reset( new VoxelHelperNode() ); 
    m_rootPtr->m_svoBounds = svoBounds;

    // set m_LeafStopSplitSize to default size if it's too small, this can limit the file size
    if( m_LeafStopSplitSize < m_minPointDistance )
        m_LeafStopSplitSize = m_minPointDistance;

    if( m_outputToLog )
    {
        RCLog::getLogFile()->addMessage( L"=============================");
        RCLog::getLogFile()->addMessage( m_fileName.c_str() );
    }
}

template<typename PointType>
VoxelOctreeCreator<PointType>::~VoxelOctreeCreator()
{
    Memory::delayedDelete( m_rootPtr );
}

template<typename PointType>
bool VoxelOctreeCreator<PointType>::createOctree()
{
    shuffleInputBuffer(m_octreePoints);

    //add points
    for( int i = 0; i < m_amountOfPoints; i++ )
    {
        if (m_cancelCb())
        {
            return false;
        }
        addPointToTree( i );
    }

    //split leaf nodes, because they contain more than 1 point and big
    splitVoxelTree();

    //merge leaf nodes
    mergeVoxelTree();

    //calculate normals from unstructured data
    if( !mHasNormals && mCalculateNormalsFromUnstructuredData)
    {
        calculateNormalsFromUnstructuredData( 9 );
    }

    //linearize tree
    linearizeSvoTree();

    //done
    return true;
}

template<typename PointType>
bool VoxelOctreeCreator<PointType>::splitVoxelTree()
{
    std::vector<VoxelHelperNode*> resultList;
    int numIterations = 0;
    bool iterate = true;

    while( iterate ) //split up leaf nodes
    {
        if (m_cancelCb())
        {
            return false;
        }

        //fetch all leaf nodes that needs to be splitted up
        resultList = evaluateOctree( ENUM_EVALUATE_METHOD_GET_SPLIT_NODES );

        if( !resultList.size() )
            break;

        //split this node into new children
        for (const auto& rl : resultList)
        {
            splitVoxelNode( rl );
        }

        if( m_outputToLog)
        {
            std::wstringstream ss;
            ss << L"Splitting nodes found: ";
            ss << static_cast<int>(resultList.size() );
            ss << L", iteration ";
            ss << numIterations;
            RCLog::getLogFile()->addMessage( ss.str( ).c_str() );
        }
        numIterations++;
    }
    return true;
}

template<typename PointType>
bool VoxelOctreeCreator<PointType>::mergeVoxelTree()
{
    std::vector<VoxelHelperNode*> resultList;

    int numIterations = 0;
    bool iterate = true;
    while( iterate ) //try to merge leaf nodes 
    {
        if (m_cancelCb())
        {
            return false;
        }

        resultList = evaluateOctree( ENUM_EVALUATE_METHOD_GET_MERGE_NODES );
        if( !resultList.size() )
            break;

        //merge this node with its parent
        for (const auto& rl : resultList)
        {
            mergeVoxelNode( rl );
        }

        if( m_outputToLog )
        {
            std::wstringstream ss;
            ss << L"Merge nodes found: ";
            ss << static_cast<int>(resultList.size() );
            ss << L", iteration: ";
            ss << numIterations;
            RCLog::getLogFile()->addMessage( ss.str( ).c_str() );
        }
        numIterations++;
    }
    return true;
}

template<typename PointType>
bool VoxelOctreeCreator<PointType>::splitVoxelNode( VoxelHelperNode* nodePtr )
{
    // Don't split if the bbox is already small enough and keep only the point that is closest to the center of the leaf-node.
    if((nodePtr->m_svoBounds.getMax().x - nodePtr->m_svoBounds.getMin().x) <= m_LeafStopSplitSize)
    {
        RCVector3d bboxCenter = nodePtr->m_svoBounds.getCenter();
        int numIndices = static_cast<int>( nodePtr->m_pointIndices.size() );
        double minDistance = 9999;
        int closestPointIndex = -1;
        for( int i = 0; i < numIndices; i++ )
        {
            int index = nodePtr->m_pointIndices[i];
            RCVector3d newPoint( m_octreePoints[index].m_pos[0], m_octreePoints[index].m_pos[1], m_octreePoints[index].m_pos[2] );

            double dist = newPoint.distanceSqrd(bboxCenter);

            if(dist<minDistance)
            {
                minDistance = dist;
                closestPointIndex = index;
            }
        }

        nodePtr->m_pointIndices.clear();
        if (closestPointIndex != -1)
            nodePtr->m_pointIndices.push_back(closestPointIndex);
    }
    else
    {
        int numIndices = static_cast<int>( nodePtr->m_pointIndices.size() );
        for( int i = 0; i < numIndices; i++ )
        {
            int index = nodePtr->m_pointIndices[i];

            OctreeChildInformation childInfo; 
            RCVector3d point( m_octreePoints[index].m_pos[0], m_octreePoints[index].m_pos[1], m_octreePoints[index].m_pos[2] );

            getChildInformation( nodePtr->m_svoBounds, point, childInfo );

            VoxelHelperNode* childNodePtr  = nodePtr->m_childList[ childInfo.m_childId ];
            if( !childNodePtr )
            {
                childNodePtr = new VoxelHelperNode( nodePtr );
                childNodePtr->m_isVoxelLeaf = true;
                childNodePtr->m_svoBounds = childInfo.m_childBounds;
                nodePtr->m_childList[ childInfo.m_childId ] = childNodePtr;
            }
            childNodePtr->m_pointIndices.push_back( index );
        }

        //clear point indices
        nodePtr->m_pointIndices.clear();
        std::vector<int>().swap( nodePtr->m_pointIndices );
        nodePtr->m_isVoxelLeaf = false; //not a leaf anymore
    }

    return true;
}

template<typename PointType>
bool VoxelOctreeCreator<PointType>::mergeVoxelNode( VoxelHelperNode* parentPtr )
{
    std::vector<int> newIndices;
    for( int i = 0; i < 8; i++ )
    {
        VoxelHelperNode* childPtr = parentPtr->m_childList[i];
        if( childPtr )
        {
            newIndices.insert( newIndices.end(), childPtr->m_pointIndices.begin(), childPtr->m_pointIndices.end() );
        }
    }
    Utility::Memory::delayedDeletePtrContainer( parentPtr->m_childList );
    parentPtr->m_childList.fill( nullptr );

    //promote to leaf node
    parentPtr->m_pointIndices = newIndices;
    parentPtr->m_isVoxelLeaf  = true;

    return true;
}

template<typename PointType>
std::vector<VoxelHelperNode*> VoxelOctreeCreator<PointType>::evaluateOctree( int evaluateMethod ) const
{
    std::vector<VoxelHelperNode*> resultList;

    std::stack<VoxelHelperNode*> nodeStack;
    nodeStack.push( m_rootPtr.get() );
    //push onto stack
    while( !nodeStack.empty() )
    {
        VoxelHelperNode* parentNodePtr = nodeStack.top(); 
        nodeStack.pop();

        if( parentNodePtr->m_isVoxelLeaf )
        {
            int numPoints = static_cast<int>( parentNodePtr->m_pointIndices.size() );
            if( ENUM_EVALUATE_METHOD_GET_SPLIT_NODES == evaluateMethod )
            {
                if( numPoints > POINTS_PER_SVO_LEAF )
                    resultList.push_back( parentNodePtr );
            }
            else if( ENUM_EVALUATE_METHOD_GET_MERGE_NODES == evaluateMethod )
            {
                if( canMergeVoxelNode( parentNodePtr ) )
                    resultList.push_back( parentNodePtr->m_parentPtr );
            }
            else if( ENUM_EVALUATE_METHOD_GET_ALL_LEAF_NODES == evaluateMethod )
            {
                resultList.push_back( parentNodePtr );
            }
        }
        else
        {
            // 'recurse'
            for( int i = 0; i < 8; i++ )
            {
                VoxelHelperNode* childNodePtr = parentNodePtr->m_childList[i];
                if( childNodePtr ) 
                    nodeStack.push( childNodePtr );
            }
        }
    }

    return resultList;
}


template<typename PointType>
bool VoxelOctreeCreator<PointType>::addPointToTree( int index )
{
    VoxelHelperNode* parentPtr = m_rootPtr.get();

    //traverse
    while( parentPtr )
    {
        //done
        if( parentPtr->m_isVoxelLeaf )
        {
            parentPtr->m_pointIndices.push_back( index );
            parentPtr = NULL;
        }
        else
        {
            OctreeChildInformation childInfo; 
            RCVector3d point( m_octreePoints[index].m_pos[0], m_octreePoints[index].m_pos[1], m_octreePoints[index].m_pos[2] );
            getChildInformation( parentPtr->m_svoBounds, point, childInfo );

            VoxelHelperNode* childNodePtr  = parentPtr->m_childList[ childInfo.m_childId ];
            // first time we see this child
            if( !childNodePtr ) 
            {
                childNodePtr   = new VoxelHelperNode( parentPtr);
                //assign members
                childNodePtr->m_svoBounds   = childInfo.m_childBounds;
                //assign child node to parent
                parentPtr->m_childList[ childInfo.m_childId ] = childNodePtr;

                //bounding box size
                double  bboxSize = childNodePtr->m_svoBounds.getMax().x - childNodePtr->m_svoBounds.getMin().x;  
                //create temp leaf node - either m_currentDepth is deep enough or the bbox is small enough
                if( childNodePtr->m_currentDepth == m_startDepth || bboxSize <= m_LeafStopSplitSize )
                    childNodePtr->m_isVoxelLeaf = true;
            }
            parentPtr = childNodePtr;
        }
    }
    return true;
}

template<typename PointType>
bool VoxelOctreeCreator<PointType>::updateOctreeLeafNodeData( int pointIndex )
{
    if (pointIndex < 0)
        return false;

    std::stack<VoxelHelperNode*> nodeStack;
    nodeStack.push( m_rootPtr.get() );
    //traverse
    while( !nodeStack.empty() )
    {
        VoxelHelperNode* parentNodePtr = nodeStack.top(); 
        nodeStack.pop();
        //done
        if( parentNodePtr->m_isVoxelLeaf )
        {
            if (parentNodePtr->m_pointIndices.size() > 0)
            {
                parentNodePtr->m_pointIndices[0]   = pointIndex;
                parentNodePtr->m_indexToFirstChild = pointIndex;
            }
        }
        else //traverse tree
        {
            OctreeChildInformation childInfo;
            RCVector3d point( m_octreePoints[pointIndex].m_pos[0], m_octreePoints[pointIndex].m_pos[1], m_octreePoints[pointIndex].m_pos[2] );
            getChildInformation( parentNodePtr->m_svoBounds, point, childInfo );
            VoxelHelperNode* childNodePtr  = parentNodePtr->m_childList[ childInfo.m_childId ];
            if( childNodePtr )
                nodeStack.push( childNodePtr );
        }
    }
    return true;
}


template<typename PointType>
bool VoxelOctreeCreator<PointType>::saveToDisk(OctreeIntermediateNode<PointType>* leafPtr)
{
    std::vector<AgVoxelInteriorNode> nodeList[MAX_SVO_NODE_LEVELS];
    //create interior node lists
    for( int i = 0; i < ( m_maxLeafDepth + 1 ); i++ )
    {
        VoxelInformation* curVoxDepthPtr = &m_voxelInfoList[i];
        if (!curVoxDepthPtr) continue;
        for( int j = 0; j < static_cast<int>( curVoxDepthPtr->m_interiorNodes.size() ); j++ )
        {
            VoxelHelperNode* curHelperNodePtr = curVoxDepthPtr->m_interiorNodes[j];

            int childIndex = 0;
            for( int k = 0; k < 8; k++ )
                if( curHelperNodePtr->m_childList[k] ) //adjust bitfield
                    childIndex |= ( 1 << k ); 

			AgVoxelInteriorNode interiorNode;
            interiorNode.setChildInfo ( childIndex );
            interiorNode.setIndexToChild ( curHelperNodePtr->m_indexToFirstChild );
            interiorNode.setLeafInfo ( curHelperNodePtr->m_isVoxelLeaf );

            //add to current list
            nodeList[i].push_back( interiorNode );
        }
    }
    //fill in umbrella leaf node data
    leafPtr->m_amountOfPointsInFile = m_amountOfPoints;
    leafPtr->m_maxTreeDepth   = m_maxLeafDepth + 1;
    for( int i = 0; i < MAX_SVO_NODE_LEVELS; i++ )
    {
        leafPtr->m_numLeafNodes[i] = m_voxelInfoList[i].m_numLeafNodes; 
        leafPtr->m_numNonLeafNodes[i] = m_voxelInfoList[i].m_numNonLeafNodes;
        //update filesize
        leafPtr->m_fileSize += sizeof(AgVoxelInteriorNode ) * ( leafPtr->m_numLeafNodes[i] + leafPtr->m_numNonLeafNodes[i] );
    }
    //update filesize
    leafPtr->m_fileSize += sizeof(AgVoxelLeafNode ) * ( m_amountOfPoints );

    {
        //now output to disk
        fs::ofstream outputStream( m_fileName, std::ios::out | std::ios::binary );
        if (!outputStream.is_open())
        {
            return false;
        }

        //write interior node info
        for( int i = 0; i < ( m_maxLeafDepth + 1 ); i++ )
        {
            outputStream.write( ( char* ) &nodeList[i][0], sizeof(AgVoxelInteriorNode ) * nodeList[i].size() );

            if (outputStream.fail())
            {
                return false;
            }
        }


        //write actual point data
        for( int i = 0; i < m_amountOfPoints; i++ )
        {
            //convert to leaf node
			AgVoxelLeafNode leafNode;
            const PointType& curPoint = m_octreePoints[i];

			RCVector3d rawOffset = 1000 * (curPoint.m_pos - m_rootPtr->m_svoBounds.getMin());
            leafNode.setRawOffsetFromBoundingBox(Urho3D::Vector3((float)rawOffset.x, (float)rawOffset.y, (float)rawOffset.z));
            leafNode.setRGBA (AgCompactColor( curPoint.m_rgba[0], curPoint.m_rgba[1], curPoint.m_rgba[2], curPoint.m_rgba[3] ) );
            leafNode.setNormal ( curPoint.m_normal );
            leafNode.setLidarClassification ( curPoint.m_misc[0] );
            leafNode.setPrimitiveClassification ( curPoint.m_misc[3] );

            outputStream.write( ( char* ) &leafNode, sizeof(AgVoxelLeafNode ) );

            if (outputStream.fail())
            {
                return false;
            }
        }

        outputStream.close();
    }

    if ( mHasTimestamps )
    {
        if ( !saveTimeStampToDisk() )
            return false;
    }

    return true;
}

template<>
bool VoxelOctreeCreator<ExtendedOctreeImportPoint>::saveTimeStampToDisk()
{
    //now output to disk
    fs::ofstream outputStream( m_timeStampFileName, std::ios::out | std::ios::binary );
    if (!outputStream.is_open())
    {
        return false;
    }

    //write actual point data
    for( int i = 0; i < m_amountOfPoints; i++ )
    {
        double timeStamp = m_octreePoints[i].m_reserved;
        outputStream.write( ( char* ) &timeStamp, sizeof( double ) );

        if (outputStream.fail())
        {
            return false;
        }
    }
    outputStream.close();

    return true;
}

template<>
bool VoxelOctreeCreator<BasicOctreeImportPoint>::saveTimeStampToDisk()
{
    return false;
}

template<typename PointType>
bool VoxelOctreeCreator<PointType>::linearizeSvoTree()
{
    std::vector<int> leafIndices; //Optimize amount of leafs, isn't always the same as amount of points!!
    //FILL IN 
    {
        std::stack<VoxelHelperNode*> nodeStack;
        nodeStack.push( m_rootPtr.get() );

        while( !nodeStack.empty() )
        {
            VoxelHelperNode* parentNodePtr = nodeStack.top(); 
            nodeStack.pop();

            //fetch parent depth
            int parentDepth = parentNodePtr->m_currentDepth;
            //adjust max depth
            if( parentDepth > m_maxLeafDepth )
                m_maxLeafDepth = parentDepth;

            //add new node
            if(parentDepth >= VoxelOctreeCreator::MAX_SVO_NODE_LEVELS)
            {
                RCLog::getLogFile()->addMessage(L"Voxel tree depth is deeper than max level");
                return false;
            }

            m_voxelInfoList[parentDepth].m_interiorNodes.push_back( parentNodePtr );

            //update  info
            if( parentNodePtr->m_isVoxelLeaf )
            {
                m_voxelInfoList[parentDepth].m_numLeafNodes++;
                if( POINTS_PER_SVO_LEAF == static_cast<int>( parentNodePtr->m_pointIndices.size() ) )
                {
                    if (parentNodePtr->m_pointIndices[0] >= 0)
                        leafIndices.push_back( parentNodePtr->m_pointIndices[0] );
                    else
                        RCLog::getLogFile()->addMessage(L"index error");
                }
            }
            else //'recurse' with children
            {
                m_voxelInfoList[parentDepth].m_numNonLeafNodes++;
                for( int i = 0; i < 8; i++ )
                {
                    VoxelHelperNode* childNodePtr = parentNodePtr->m_childList[i];
                    if( childNodePtr ) 
                        nodeStack.push( childNodePtr );
                }
            }
        }
    }

    //RE-ARRANGE
    int cumulativeLeafNodes = 0;
    for( int i = 0; i < ( m_maxLeafDepth + 1 ); i++ )
    {
        VoxelInformation* curVoxDepthPtr = &m_voxelInfoList[i];

        //adjust amount of cumulative leaf nodes
        cumulativeLeafNodes  += curVoxDepthPtr->m_numLeafNodes;
        curVoxDepthPtr->m_numCumulativeLeafNodes += cumulativeLeafNodes;

        //re-arranged list for next level
        std::vector<VoxelHelperNode*> nextLevel;

        //fetch all child voxels
        for( int j = 0; j < static_cast<int>(curVoxDepthPtr->m_interiorNodes.size()); j++ )
        {
            VoxelHelperNode* curVoxelHelper = curVoxDepthPtr->m_interiorNodes[j];

            //adjust index to first child
            if( !curVoxelHelper->m_isVoxelLeaf )
            {
                curVoxelHelper->m_indexToFirstChild = static_cast<int>( nextLevel.size() );
                for( int k = 0; k < 8; k++ )
                {
                    if( curVoxelHelper->m_childList[k] )
                    {
                        curVoxelHelper->m_numChilds++;
                        nextLevel.push_back( curVoxelHelper->m_childList[k] );
                    }
                }
            }
            else
            {
                if( POINTS_PER_SVO_LEAF == static_cast<int>( curVoxelHelper->m_pointIndices.size() ) )
                    curVoxelHelper->m_indexToFirstChild = curVoxelHelper->m_pointIndices[0];
            }
        }
        //re-assign arranged list
        if( nextLevel.size() )
        {
            assert( m_voxelInfoList[i+1].m_interiorNodes.size() == nextLevel.size() );
            m_voxelInfoList[i+1].m_interiorNodes = nextLevel;
        }
    }


    int numIndices = static_cast<int>( leafIndices.size() );

    //do we need to resize the m_octreePoints array?
    if( m_amountOfPoints != numIndices  )
    {
        std::vector<PointType> newPointList; newPointList.reserve(numIndices);
        for(const auto& index : leafIndices) {
            newPointList.emplace_back(m_octreePoints[index]);
        }

        std::swap(m_octreePoints, newPointList);
        m_amountOfPoints = numIndices;

        shuffleInputBuffer(m_octreePoints);

        //update also leaf nodes with new indices
        for( int i = 0; i < m_amountOfPoints; i++ )
            updateOctreeLeafNodeData( i );
    }

    return true;
}

template<typename PointType>
bool VoxelOctreeCreator<PointType>::removeLeafNodeFromList( VoxelHelperNode* nodePtr )
{
    std::vector<VoxelHelperNode*>::iterator iter = m_leafList.begin();
    for( ; iter != m_leafList.end(); )
    {
        if( *iter == nodePtr )
        {
            m_leafList.erase( iter );
            return true;
        }
        else 
            iter++;
    }
    return false;
}

template<typename PointType>
bool VoxelOctreeCreator<PointType>::canMergeVoxelNode( VoxelHelperNode* nodePtr ) const
{
    //A voxel node can be merged if the amount of point of all of its siblings
    //are below m_maxPointsPerLeaf
    int pointCount   = 0;
    if( nodePtr->m_parentPtr )
    {
        for( int i = 0; i < 8; i++ )
        {
            VoxelHelperNode* childNode = nodePtr->m_parentPtr->m_childList[i];
            if( childNode  )
            {
                if( childNode->m_isVoxelLeaf )
                    pointCount +=  static_cast<int>( childNode->m_pointIndices.size() );
                else //tree goes deeper we can't merge
                    return false;
            }
        }
        return( ( pointCount <= POINTS_PER_SVO_LEAF ) );
    }
    return false;
}

template<typename PointType>
std::vector<int> VoxelOctreeCreator<PointType>::getNearestNeighbours( const RCBox& bounds ) const
{
    std::vector<int> neighbourList;

    VoxelHelperNode* parentPtr = m_rootPtr.get();
    std::stack<VoxelHelperNode*> nodeStack;
    nodeStack.push( parentPtr );
    while( nodeStack.size() )
    {
        parentPtr = nodeStack.top(); nodeStack.pop();
        if( parentPtr->m_isVoxelLeaf )
        {
            if ( parentPtr->m_pointIndices.size() > 0 )
            {
                int pointIndex = parentPtr->m_pointIndices[0];
                if( bounds.pointInBox( RCVector3d( m_octreePoints[pointIndex].m_pos[0], m_octreePoints[pointIndex].m_pos[1], m_octreePoints[pointIndex].m_pos[2] ) ) )
                {
                    neighbourList.push_back( pointIndex );
                }
            }
        }
        else //traverse tree
        {
            for( int i = 0; i < 8; i++ )
            {
                VoxelHelperNode* childNodePtr  = parentPtr->m_childList[ i ];
                if( childNodePtr )
                    if( childNodePtr->m_svoBounds.intersect( bounds ) )
                        nodeStack.push( childNodePtr );
            }
        }
    }
    return neighbourList;
}

template<typename PointType>
void VoxelOctreeCreator<PointType>::calculateNormalsFromUnstructuredData( int numNeighbours /*= 9 */ )
{
    std::vector<int> pointIndices;
    std::stack<VoxelHelperNode*> nodeStack;
    nodeStack.push( m_rootPtr.get() );


    //fetch all valid point indices
    while( !nodeStack.empty() )
    {
        VoxelHelperNode* parentNodePtr = nodeStack.top(); 
        nodeStack.pop();
        //done
        if( parentNodePtr->m_isVoxelLeaf ){
            if (parentNodePtr->m_pointIndices.size() > 0)
                pointIndices.push_back ( parentNodePtr->m_pointIndices[0] );
            if( parentNodePtr->m_currentDepth > m_maxLeafDepth )
                m_maxLeafDepth = parentNodePtr->m_currentDepth;
        }
        else //traverse tree
        {
            for( int i = 0; i < 8; i++ )
            {
                VoxelHelperNode* childNodePtr = parentNodePtr->m_childList[i];
                if( childNodePtr )
                    nodeStack.push( childNodePtr );
            }
        }
    }

    if( static_cast<int>(pointIndices.size()) < numNeighbours || !m_maxLeafDepth )
        return;

    //start search radius when search for nearest neighbors
    double xExtents = m_rootPtr->m_svoBounds.getMax().x - m_rootPtr->m_svoBounds.getMin().x;
    double startSearchRadius = xExtents / pow( 2.0, ( double (m_maxLeafDepth - 1 ) ) );

    for( size_t i = 0; i < pointIndices.size(); i++ )
    {
        int pointIndex = pointIndices[i];
        PointType& importPoint = m_octreePoints[pointIndex];
        std::vector<int> neighbourList;
        float iteration = 1.0f;
        while( static_cast<int>(neighbourList.size()) < numNeighbours )
        {
            double bboxRadius = startSearchRadius * iteration;
            RCVector3d bboxCenter( importPoint.m_pos[0], importPoint.m_pos[1], importPoint.m_pos[2]); 
            RCBox searchBounds(bboxCenter, bboxRadius);
            neighbourList = getNearestNeighbours( searchBounds );

            iteration *= 2.0f;
        }
        int neighSize = static_cast<int>( neighbourList.size() );
        if( neighSize >= numNeighbours )
        {
            std::vector<RCVector3d> coordList; coordList.reserve(neighSize);
            for(const auto& nIdx : neighbourList)
            {
                coordList.emplace_back(m_octreePoints[nIdx].m_pos[0], m_octreePoints[nIdx].m_pos[1], m_octreePoints[nIdx].m_pos[2]);
            }

            double residual = 0;
            RCPlane plane = fitPlane( &coordList[0], neighSize, residual );
            importPoint.m_normal      = std::uint16_t( Math::NormalUtils::indexForNormal( plane.getNormal().convertTo<float>() ) );
        }
    }
}

template<typename PointType>
bool VoxelOctreeCreator<PointType>::getCalculateNormalsFromUnstructuredData()
{
    return mCalculateNormalsFromUnstructuredData;
}

template<typename PointType>
void VoxelOctreeCreator<PointType>::setCalculateNormalsFromUnstructuredData(bool calculateNormals)
{
    mCalculateNormalsFromUnstructuredData = calculateNormals;
}

namespace ambergris { namespace RealityComputing { namespace Import
{
    //
    template class VoxelOctreeCreator<BasicOctreeImportPoint>;
    template class VoxelOctreeCreator<ExtendedOctreeImportPoint>;
}}}
