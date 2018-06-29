//////////////////////////////////////////////////////////////////////////////
//
//                     (C) Copyright 2016 by Autodesk, Inc.
//
// The information contained herein is confidential, proprietary to Autodesk,
// Inc., and considered a trade secret as defined in section 499C of the
// penal code of the State of California.  Use of this information by anyone
// other than authorized employees of Autodesk, Inc. is granted only under a
// written non-disclosure agreement, expressly prescribing the scope and
// manner of such use.
//
//        AUTODESK PROVIDES THIS PROGRAM "AS IS" AND WITH ALL FAULTS.
//        AUTODESK SPECIFICALLY DISCLAIMS ANY IMPLIED WARRANTY OF
//        MERCHANTABILITY OR FITNESS FOR A PARTICULAR USE.  AUTODESK, INC.
//        DOES NOT WARRANT THAT THE OPERATION OF THE PROGRAM WILL BE
//        UNINTERRUPTED OR ERROR FREE.
//
//////////////////////////////////////////////////////////////////////////////

#include <vector>

#include <boost/filesystem/fstream.hpp>


#include "OctreeImportPoint.h"
#include "OctreeIntermediateNode.h"
#include <common/RCVector.h>
#include <common/RCMemoryHelper.h>

using namespace ambergris::RealityComputing::Import;
using namespace ambergris::RealityComputing::Common;

namespace fs = boost::filesystem;

//////////////////////////////////////////////////////////////////////////
//Class OctreeIntermediateNode
//////////////////////////////////////////////////////////////////////////
template<typename PointType>
OctreeIntermediateNode<PointType>::OctreeIntermediateNode()
{
    m_parentPtr = NULL;
    m_octreeDepth = 0;
    reset();
}

template<typename PointType>
OctreeIntermediateNode<PointType>::OctreeIntermediateNode( OctreeIntermediateNode<PointType>* parentPtr)
{
    m_parentPtr = parentPtr;
    m_octreeDepth = parentPtr->m_octreeDepth + 1;
    reset();
}

template<typename PointType>
OctreeIntermediateNode<PointType>::~OctreeIntermediateNode()
{
    for( int i = 0; i < 8; i++ ) 
        DeletePtr( m_childList[i] );
}

template<typename PointType>
void OctreeIntermediateNode<PointType>::writeTemporaryPointsToFile()
{
    //only write if there is data
    if( m_currentPointList.size() )
    {
        fs::ofstream outputStream( m_fileName.c_str(), std::ios::out | std::ios::binary | std::ios::app );
        if (!outputStream.is_open())
        {
            return;
        }

        for (const auto& cp : m_currentPointList)
        {
            outputStream.write( ( char* ) &cp[0], sizeof( PointType ) * static_cast<int>( cp.size() ) );
        }

        outputStream.close();

        for (auto& cp : m_currentPointList)
        {
            cp.clear();
            //clears also the 'capacity'
            std::vector<PointType>().swap( cp );
        }
        m_currentPointList.clear();
        std::vector<std::vector<PointType>>().swap( m_currentPointList );
    }
}

template<typename PointType>
void OctreeIntermediateNode<PointType>::updatePointInformation( const PointType& point )
{
    if( !m_totalAmountOfPoints )
    {
        m_rgba = RCVector4ub( point.m_rgba[0], point.m_rgba[1], point.m_rgba[2], point.m_rgba[3] );
    }
    else //average
    {
        std::uint8_t rgba[4];

        for( int i = 0; i < 4; i++ )
            rgba[i] = std::uint8_t( ( m_rgba[i] + point.m_rgba[i] ) >> 1 );

        m_rgba = RCVector4ub( rgba[0], rgba[1], rgba[2], rgba[3] );
    }
    //TODO Paul Holverda, average these also?
    m_normal = point.m_normal;
    memcpy( m_misc, point.m_misc, sizeof( std::uint8_t ) * 4 );

    m_nodeBounds.updateBounds( RCVector3d( point.m_pos[0], point.m_pos[1], point.m_pos[2] ) );

    m_totalAmountOfPoints++;
}


template<typename PointType>
void OctreeIntermediateNode<PointType>::reset()
{
    m_isOctreeLeaf = false;

    m_totalAmountOfPoints = 0;
    m_amountOfPointsInFile = 0;
    m_maxTreeDepth = 0;
    m_fileSize = 0;
    m_normal = 0;

    m_childInformation = 0;
    m_firstChildIndex = -1;
    m_indexInToLeafArray = -1;
    memset( m_numLeafNodes , 0, sizeof( int ) * 32 );
    memset( m_numNonLeafNodes , 0, sizeof( int ) * 32 );
    memset( m_misc , 0, sizeof( std::uint8_t ) * 4 ); 

    for( int i = 0; i < 8; i++ )
        m_childList[i] = NULL;
}

namespace ambergris { namespace RealityComputing { namespace Import
{
    template struct OctreeIntermediateNode<BasicOctreeImportPoint>;
    template struct OctreeIntermediateNode<ExtendedOctreeImportPoint>;
}}}

void ambergris::RealityComputing::Import::getChildInformation( const RCBox& parentBounds, const RCVector3d& point, OctreeChildInformation& childInfoOut )
{
    RCVector3d boundsCenter = parentBounds.getCenter();
    RCVector3d boundsMax = parentBounds.getMax();
    RCVector3d boundsMin = parentBounds.getMin();

    int childId = 0; //child bit mask
    double min[3];
    double max[3];

    for( int i = 0; i < 3; i++ )
    {
        if( point[i] >= boundsCenter[i] )
        {
            childId |= ( 1 << i );
            min[i] = boundsCenter[i];
            max[i]  = boundsMax[i];
        }
        else
        {
            min[i] = boundsMin[i];
            max[i] = boundsCenter[i];
        }
    }

    childInfoOut.m_childId = childId;
    childInfoOut.m_childBounds.setMin( RCVector3d(min[0], min[1], min[2]) );
    childInfoOut.m_childBounds.setMax( RCVector3d(max[0], max[1], max[2]) );
}
