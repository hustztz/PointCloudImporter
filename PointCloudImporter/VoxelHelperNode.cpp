#include <common/RCMemoryHelper.h>

#include "VoxelHelperNode.h"

using namespace ambergris::RealityComputing::Import;
using namespace ambergris::RealityComputing::Common;

//////////////////////////////////////////////////////////////////////////
// VoxelHelperNode
//////////////////////////////////////////////////////////////////////////
VoxelHelperNode::VoxelHelperNode()
{
    m_totalAmountOfPoints = 0;
    m_currentDepth = 0;
    m_parentPtr = NULL;
    m_isVoxelLeaf = false;
    m_normalId = 0;
    m_lidarClassification = 0;
    m_primitiveClassification = 0;
    m_numChilds = 0;
    m_indexToFirstChild = -1;
    for( int i = 0; i < 8; i++ )
        m_childList[i] = NULL;
}

VoxelHelperNode::VoxelHelperNode( VoxelHelperNode* parentPtr )
{
    m_totalAmountOfPoints = 0;
    m_currentDepth = parentPtr->m_currentDepth + 1;
    m_parentPtr = parentPtr;
    m_isVoxelLeaf = false;
    m_normalId = 0;
    m_lidarClassification = 0;
    m_primitiveClassification = 0;
    m_numChilds = 0;
    m_indexToFirstChild = -1;
    for( int i = 0; i < 8; i++ )
        m_childList[i] = NULL;
}


VoxelHelperNode::~VoxelHelperNode()
{
    for( int i = 0; i < 8; i++ )
        DeletePtr( m_childList[i] );
}


