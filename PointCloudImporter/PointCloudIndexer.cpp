#include <stack>
#include <cassert>
#include <sstream>
#include <algorithm>
#include <ostream>

#include <boost/filesystem/fstream.hpp>

#include <algorithm/BoundGeometry.h>

#include <common/RCMath.h>
#include <common/RCTransform.h>
#include <common/RCPlane.h>
#include <common/RCUUID.h>
#include <common/RCString.h>
#include <common/RCScanProperties.h>
#include <common/RCSegmentInfo.h>
#include <common/RCOrientedBoundingBox.h>

#include <utility/RCAssert.h>
#include <utility/RCFilesystem.h>
#include <utility/RCLog.h>
#include <utility/RCThread.h>
#include <utility/RCStringUtils.h>
#include <utility/RCTimer.h>
#include <utility/RCSpatialReference.h>

#include "OctreeDefs.h"

#include "OctreeImportPoint.h"
#include "DataSourceIndexer.h"
#include "MultithreadHelper.h"
//#include <import/ImportUtils.h>
//#include <import/TempFileDataSource.h>
//#include <import/RCImportCodecUtilities.h>

#include "OctreeIntermediateNode.h"
#include "VoxelOctreeCreator.h"
#include "PointCloudIndexer.h"
//#include <import/IPointCloudFilter.h>
//#include <import/PointCloudCSFilter.h>

using namespace ambergris::RealityComputing::Common;
using namespace ambergris::RealityComputing::Utility;
using namespace ambergris::RealityComputing::Utility::Log;
using namespace ambergris::RealityComputing::Utility::String;
using namespace ambergris::RealityComputing::Formats;
using namespace ambergris::RealityComputing::DataProcessing;
using namespace ambergris::RealityComputing::Utility::Threading;
using namespace ambergris::RealityComputing::Import;
using namespace ambergris::RealityComputing::Config;
using namespace std;

namespace fs = boost::filesystem;

#ifdef _WIN32
#define MAX_SIZE_LL84_UMBRELLA_LEAFNODE 4.0
#endif

const std::uint64_t MAX_TEMP_POINTS_IN_FILE = 2000000;
const int MAX_FILES_FOR_FAST_REMOVE = 10000;

const double ALICELABS_MAX = 10000000000.0;

namespace {

#ifdef _WIN32
    const wchar_t* PATH_SEP = L"\\";
#else
    const wchar_t* PATH_SEP = L"/";
#endif

    template<typename PointType>
    size_t getUmbrellaLeafNodeCount(const std::vector<std::vector<OctreeIntermediateNode<PointType>* > >& leafNodeList){
        size_t totalLeafNodeCount = 0;
        for( const auto& leafNodeI : leafNodeList)
            totalLeafNodeCount += leafNodeI.size();
        return totalLeafNodeCount;
    }

    template<typename PointType>
    size_t getTotalPointCount(const std::vector<std::vector<OctreeIntermediateNode<PointType>* > >& leafNodeList){

        size_t totalPointCount = 0;
        for( const auto& leafNodeI : leafNodeList)
        {
            for (const auto& leafNodeJ : leafNodeI)
            {
                if(leafNodeJ)
                    totalPointCount+= static_cast<size_t>(leafNodeJ->m_totalAmountOfPoints);
            }
        }
        return totalPointCount;
    }
}

//////////////////////////////////////////////////////////////////////////
//Class PointCloudIndexer 
//////////////////////////////////////////////////////////////////////////
template<typename PointType>
PointCloudIndexer<PointType>::PointCloudIndexer() :
    m_upDirection(UP_DIRECTION_Z),
    mHasRGB(false),
    mHasIntensity(false),
    mHasTimeStamp(false)
{
    //reset defaults
    resetToDefault();
}

template<typename PointType>
PointCloudIndexer<PointType>::~PointCloudIndexer()
{
    DeletePtr( m_rootNodePtr );
    //remove any temp files
    //removeTempFiles();
}

template<typename PointType>
const RCBox& PointCloudIndexer<PointType>::getBoundingBox() const
{
    return m_scanBounds;
}

template<typename PointType>
const RCOrientedBoundingBox& PointCloudIndexer<PointType>::getOrientedBoundingBox() const
{
    return m_scanOrientedBoundingBox;
}

template<typename PointType>
void PointCloudIndexer<PointType>::setOrientedBoundingBox(const RCOrientedBoundingBox& obb)
{
    m_scanOrientedBoundingBox = obb;
}


template<typename PointType>
void PointCloudIndexer<PointType>::resetToDefault()
{
    m_pointCloudFilterPtr.reset(nullptr);
    m_isCancelled = false;
    m_totalAmountOfPoints = 0;
    m_estimatedAmountOfPoints = 0;
    m_amountOfTempFiles = 0;
    m_numThreads = 1;
    m_amountOfPointInTempFile = 0;
    m_amountOfSplitNodes = 0;
    m_isTerrestrial = false;
    mNormalizeIntensity = true;
    mIntensityMaxValue = -999999.0;
    mIntensityMinValue =  999999.0;
    m_rootNodePtr = NULL;
    m_minimumAmountOfPointsPerSVOLeaf = MIN_SVO_LEAF_POINTS;
    m_maximiumAmountOfPointsPerLeaf = MAX_POINTS_PER_OCTREE_LEAFNODE;
    m_maxOctreeLeafBounds = MAX_SIZE_LIDAR_UMBRELLA_LEAFNODE;
    m_scale = RCVector3d( 1.0, 1.0, 1.0 );
    m_translation = RCVector3d( 0.0, 0.0, 0.0 );
    m_scannerOrigin = RCVector3d( 0.0, 0.0, 0.0 ); 
    m_rotation = RCVector3d( 0.0, 0.0, 0.0 ); 

    mLeafSplitStopSize = 0.01;

    m_scanBounds.clear();
    m_scanOrientedBoundingBox.clear();
    m_tempFileNames.clear();
    for (auto& iln : m_intermediateLeafNodeList)
    {
        iln.clear();
    }
    m_intermediateLeafNodeList.clear();

    mFilterType = NOISE_FILTER_MEDIUM;

    mRangeClippingEnabled = true;

    mScanIdToTempFilesMap.clear();
    mCurrentScanId = 0;

    mErrorCode = rcOK;
}

template<typename PointType>
const RCVector3d& PointCloudIndexer<PointType>::getTranslation() const
{
    return m_translation;
}

template<typename PointType>
const RCVector3d& PointCloudIndexer<PointType>::getRotation() const
{
    return m_rotation;
}

template<typename PointType>
const RCVector3d& PointCloudIndexer<PointType>::getScale() const
{
    return m_scale;
}

template<typename PointType>
RCBox	PointCloudIndexer<PointType>::getSVOBounds() const
{
	return m_rootNodePtr->m_svoBounds;
}

template<typename PointType>
RCBox	PointCloudIndexer<PointType>::getScanBounds() const
{
	return m_rootNodePtr->m_nodeBounds;
}

template<typename PointType>
std::wstring PointCloudIndexer<PointType>::getOutputFolder() const
{
    RCASSERTCORE( ! m_outputFolder.empty(), "Empty output directory" );
    return ( m_outputFolder + PATH_SEP );
}

template<typename PointType>
std::wstring PointCloudIndexer<PointType>::getTempFolder() const
{
    RCASSERTCORE( ! m_tempFolder.empty(), "Empty temp directory" );
    return ( m_tempFolder + PATH_SEP );
}

template<typename PointType>
void PointCloudIndexer<PointType>::setOutputFolder( const std::wstring& val )
{
    RCASSERTCORE( Filesystem::exists( val ), "Folder does not exist" );
    m_outputFolder = val;
}

template<typename PointType>
void PointCloudIndexer<PointType>::setInput( const std::wstring& name, const std::wstring& filepath )
{
    m_inputFilePath = filepath;

    m_nodeName = name;
}

template<typename PointType>
const std::wstring& PointCloudIndexer<PointType>::getNodeName() const
{
    return m_nodeName;
}

template<typename PointType>
void PointCloudIndexer<PointType>::setTempFolder( const std::wstring& val )
{
    RCASSERTCORE( Filesystem::exists( val ), "Folder does not exist" );
    m_tempFolder = val;
}

template<typename PointType>
IPointCloudFilter* PointCloudIndexer<PointType>::getPointCloudFilter()
{
    return m_pointCloudFilterPtr.get();
}

template<typename PointType>
void PointCloudIndexer<PointType>::setPointCloudFilter( std::unique_ptr<IPointCloudFilter> filter)
{
    m_pointCloudFilterPtr = std::move(filter);
}

template<typename PointType>
void PointCloudIndexer<PointType>::setMinimumAmountOfPointsSVOLeaf( int val )
{
    m_minimumAmountOfPointsPerSVOLeaf = val;
}

template<typename PointType>
void PointCloudIndexer<PointType>::setMaximumAmountOfPointsOctreeLeaf( int val )
{
    m_maximiumAmountOfPointsPerLeaf = val;
}

template<typename PointType>
int PointCloudIndexer<PointType>::getMinimumAmountOfPointsSVOLeaf() const
{
    return m_minimumAmountOfPointsPerSVOLeaf;
}

template<typename PointType>
void PointCloudIndexer<PointType>::setMaxOctreeLeafBounds( double val )
{
    m_maxOctreeLeafBounds =  val ;
}

template<typename PointType>
bool PointCloudIndexer<PointType>::getScanTempFileNames()
{
    int numScans = getNumScans();
    int currentScanId = getCurrentScanId();

    if (numScans > 1)
    {
        if (mScanIdToTempFilesMap.count(currentScanId) > 0)
        {
            m_tempFileNames = mScanIdToTempFilesMap[currentScanId];
        }
        else
        {
            RCLog::getLogFile()->addMessage(L"Generating Range Image error");
            return false;
        }
    }

    return true;
}

template<typename PointType>
void PointCloudIndexer<PointType>::cleanupDataSource(IRCStructuredDataSource*& structureDataSource)
{
    if (structureDataSource)
    {
        structureDataSource->close();
        delete structureDataSource;
        structureDataSource = nullptr;
    }
}

template<typename PointType>
std::int64_t  PointCloudIndexer<PointType>::linearizeUmbrellaOctree()
{
    //add root 
    mUmbrellaNodeInformation[0].m_umbrellaNodeList.push_back( m_rootNodePtr );

    for( int i = 0; i < MAX_UMBRELLA_NODE_LEVELS - 1; i++ )
    {
        auto& curNodeInfoList = mUmbrellaNodeInformation[i];
        std::vector<OctreeIntermediateNode<PointType>* > nextLevel;
        //loop over all parent nodes
        for( size_t j = 0; j < curNodeInfoList.m_umbrellaNodeList.size(); j++ )
        {
            OctreeIntermediateNode<PointType>* curParentNode = curNodeInfoList.m_umbrellaNodeList[j];
            if( !curParentNode->m_isOctreeLeaf )
            {
                curParentNode->m_firstChildIndex = static_cast<int>( nextLevel.size() );
                //gather child info
                for( int k = 0; k < 8; k++ )
                {
                    if( curParentNode->m_childList[k] )
                    {
                        curParentNode->m_childInformation |= ( 1 << k );
                        nextLevel.push_back( curParentNode->m_childList[k] );
                    }
                }
            }
        }
        if( nextLevel.size() )
            mUmbrellaNodeInformation[i+1].m_umbrellaNodeList = nextLevel;
    }

    std::int64_t amountOfNodes = 0;
    for( int i = 0; i < MAX_UMBRELLA_NODE_LEVELS; i++ )
        amountOfNodes += std::int64_t( mUmbrellaNodeInformation[i].m_umbrellaNodeList.size() );

    return amountOfNodes;

}

template<typename PointType>
void PointCloudIndexer<PointType>::setCancelled( bool val )
{
    m_isCancelled = val;
}

template<typename PointType>
bool PointCloudIndexer<PointType>::getCancelled() const
{
    return m_isCancelled;
}

template<typename PointType>
bool PointCloudIndexer<PointType>::generateUmbrellaOctreeFromUnstructuredData()
{
    if (m_tempFileNames.size() == 0)
        return false;

    const auto progressPerTempFileScale = 1.0 / static_cast<double>(m_tempFileNames.size());
    const auto addPointProgressWeight = 0.5;
    const auto writePointProgressWeight = 1.0 - addPointProgressWeight;

    for (const auto& tempFile : m_tempFileNames)
    {
        fs::ifstream inputStream( tempFile.c_str(), std::ios::in | std::ios::binary );
        if (!inputStream.is_open())
        {
            return false;
        }
        int numVerts = 0;
        inputStream.read( ( char* ) &numVerts, sizeof( numVerts ) );
        m_tempPoints.resize(numVerts);
        inputStream.read( ( char* ) &m_tempPoints[0], sizeof( PointType ) * numVerts );
        inputStream.close();
        if (inputStream.fail())
            return false;

        //if we aren't normalizing, assume one of 3 options for intensity range (0-1, 0-255, or 0-65535)
        float intensityScaleFactor = 255.0f;    //0-1 range
        if (mIntensityMaxValue > 1.0f)
            intensityScaleFactor = 1.0f;        //0-255 range
        if (mIntensityMaxValue > 255.0f)
            intensityScaleFactor = 1/255.0f;    //0-65535 range

        //add point to umbrella octree 
        for( int j = 0; j < numVerts; j++ )
        {
            PointType& indexPoint = m_tempPoints[j];

            //intensity check
            if( ( mIntensityMaxValue - mIntensityMinValue ) > 0.0f )
            {
                if (mNormalizeIntensity)
                {
                    indexPoint.m_rgba[3] = getNormalizedIntensityValue(mIntensityMinValue, mIntensityMaxValue, indexPoint.m_intensity);
                }
                else
                {
                    indexPoint.m_rgba[3] = static_cast<std::uint8_t>(indexPoint.m_intensity * intensityScaleFactor);
                }
            }
            //update intensity
            if( !mHasIntensity )
            {
                std::uint8_t rgba = getNormalizedIntensityValue( 
                    (float)m_scanBounds.getMin().z, 
                    (float)m_scanBounds.getMax().z, 
                    (float)indexPoint.m_pos[2]  );

                indexPoint.m_rgba[3] = rgba;
            }
            if( !mHasRGB )
            {
                indexPoint.m_rgba[0] = indexPoint.m_rgba[3];
                indexPoint.m_rgba[1] = indexPoint.m_rgba[3];
                indexPoint.m_rgba[2] = indexPoint.m_rgba[3];
            }

            //subtract offset
            indexPoint.m_pos[0] -= m_scannerOrigin.x; 
            indexPoint.m_pos[1] -= m_scannerOrigin.y;
            indexPoint.m_pos[2] -= m_scannerOrigin.z;

            if( ! addPointToUmbrellaTree( m_tempPoints[j] ) )
                return false;

            if (getCancelled())
                return false;
        }

        //remove current temp file, we can work on the leaf nodes directly from now on
        Filesystem::remove( tempFile );

        //check if m_intermediateLeafNodeList is empty
        if( m_intermediateLeafNodeList.size() == 0)
            return false;

        // get total number of intermediateLeafNode
        int totalIntermediateLeafNodeNum = 0;
        for( int k = 0; k < static_cast<int>(m_intermediateLeafNodeList.size()); k++ )
            totalIntermediateLeafNodeNum += static_cast<int>(m_intermediateLeafNodeList[k].size());

        //flush node
        for( size_t j = 0; j < m_intermediateLeafNodeList.size(); j++ ){
            for (size_t l = 0; l < m_intermediateLeafNodeList[j].size(); l++)
            {
                if (getCancelled())
                    return false;

                m_intermediateLeafNodeList[j][l]->writeTemporaryPointsToFile();
            }
        }
    }

    return true;
}

template<typename PointType> 
bool PointCloudIndexer<PointType>::generateUmbrellaOctree( std::wstring& errorLog )
{
    RCLog::getLogFile()->addMessage( L"PointCloudIndexer::generateUmbrellaOctree()" );

    //create root node
    m_rootNodePtr = new OctreeIntermediateNode<PointType>();

    /*
    Note: Scan bounds can be something like this min/max --> [-20....50], Effectively a range of 70.0 meters
    We need to translate this bounding box to the origin so its min/max becomes -->[-35....35]
    And also translate all the temp points found in
    */
    double extents;

	extents = Math::max3(m_scanBounds.getMax() - m_scanBounds.getMin()) * 0.5;
	m_scannerOrigin = m_scanBounds.getCenter();
	m_translation = m_scannerOrigin;

    //create bounding box centered at origin
    RCBox svoBounds( RCVector3d( -extents, -extents, -extents ), RCVector3d( extents, extents, extents ) );

    //expand 1 cm to avoid floating point issues
    svoBounds.expand( 0.01 );
    m_scanBounds.print();

    // return false if the scan bounds is invalid
    if( abs(m_scanBounds.getMin().x) >= ALICELABS_MAX || 
        abs(m_scanBounds.getMin().y) >= ALICELABS_MAX ||
        abs(m_scanBounds.getMin().z) >= ALICELABS_MAX || 
        abs(m_scanBounds.getMax().x) >= ALICELABS_MAX ||
        abs(m_scanBounds.getMax().y) >= ALICELABS_MAX || 
        abs(m_scanBounds.getMax().z) >= ALICELABS_MAX )
    {
        RCLog::getLogFile()->addMessage(L"scan bounds error", false);
        return false;
    }

    double maxExtends = svoBounds.getMax().x;
    double size = m_maxOctreeLeafBounds - 0.1;

    // add maxExtends & size to log file 
    std::wstringstream ss;
    ss << L"maxExtends:";
    ss << maxExtends;
    ss << L" , size:";
    ss << size;
    RCLog::getLogFile()->addMessage( ss.str().c_str() );

    //in some cases, maxExtends is smaller than m_maxOctreeLeadBounds. In this case, we will set maxOctreeLeafBounds according to maxExtends.
    if (m_maxOctreeLeafBounds > maxExtends) 
        m_maxOctreeLeafBounds = maxExtends / 5.0;

    //In certain situations the logic above can set the umbrella leaf node size (m_maxOctreeLeafBounds) to larger than 262m.  In that case
    //the position offsets can overrun the 18bits used to represent them and cause data corruption.  This check will prevent that from occurring
    //ultimately we should replace the logic above with something simpler and more comprehensible.
    if(m_maxOctreeLeafBounds > MAX_SIZE_LIDAR_UMBRELLA_LEAFNODE - 0.1) m_maxOctreeLeafBounds = MAX_SIZE_LIDAR_UMBRELLA_LEAFNODE - 0.1;

    //adjust size
    size = m_maxOctreeLeafBounds * 0.9;  //using -0.1 will cause the following expression into infinite iteration

    while( size < maxExtends )
        size *= 2.0;

    svoBounds.setMin( RCVector3d( -size, -size, -size ) );
    svoBounds.setMax( RCVector3d( size, size, size ) );
    m_rootNodePtr->m_svoBounds = svoBounds;
    m_rootNodePtr->m_nodeBounds.clear();

    Time::RCTimer timer; timer.start();
	if (!generateUmbrellaOctreeFromUnstructuredData())
		return false;

    //for progress update

    std::vector< OctreeIntermediateNode<PointType>* > splitList = evaluateUmbrellaLeafNodes( errorLog );

    while( splitList.size() ) //go on until there is nothing more to be split
    {
        if( getCancelled() )
            return false;

        for (const auto& sl : splitList)
        {
            if( ! splitUmbrellaLeafNode( sl ) )
            {
                return false;
            }
        }

        splitList = evaluateUmbrellaLeafNodes( errorLog );
    }
    timer.stop();
	
    return true;
}

template<typename PointType> 
bool PointCloudIndexer<PointType>::writeNewTempFile()
{
    //nothing to write
    if( !m_amountOfPointInTempFile )
        return false;

    std::wstringstream stream;
    stream << getTempFolder();
    stream << m_nodeName<< L"_";
    stream << TEMP_POINT_FILE_NAME;
    stream << m_amountOfTempFiles;
    stream << TEMP_POINT_EXTENSION;

    std::wstring outputFile = stream.str();

    //output stream
    fs::ofstream outputStream( outputFile.c_str(),  std::ios::out | std::ios::binary );
    if (!outputStream.is_open())
    {
        return false;
    }

    //now write!
    outputStream.write( ( char* ) &m_amountOfPointInTempFile, sizeof( m_amountOfPointInTempFile ) );
    outputStream.write( ( char* ) &m_tempPoints[0], sizeof( PointType ) * m_amountOfPointInTempFile );
    outputStream.close();

    if (outputStream.fail())
    {
        return false;
    }

    //add temp file to list
    m_tempFileNames.push_back( outputFile );

    //increment
    m_amountOfTempFiles++;

    return true;
}

template<typename PointType> 
void PointCloudIndexer<PointType>::setNumberOfThreads( int val )
{
    m_numThreads = val;
}

template<typename PointType> 
int PointCloudIndexer<PointType>::getNumberOfThreads() const
{
    return m_numThreads;
}

template<typename PointType>
PointType PointCloudIndexer<PointType>::updateWithBasicOctreeImportPoint( const PointType& inputPoint, const BasicOctreeImportPoint& update )
{
    PointType outPoint = inputPoint;

    //just copy the common parts
    outPoint.m_intensity = update.m_intensity;

    outPoint.m_misc[0] = update.m_misc[0];
    outPoint.m_misc[1] = update.m_misc[1];
    outPoint.m_misc[2] = update.m_misc[2];
    outPoint.m_misc[3] = update.m_misc[3];

    outPoint.m_pos[0] = update.m_pos[0];
    outPoint.m_pos[1] = update.m_pos[1];
    outPoint.m_pos[2] = update.m_pos[2];

    outPoint.m_rgba[0] = inputPoint.m_rgba[0];
    outPoint.m_rgba[1] = inputPoint.m_rgba[1];
    outPoint.m_rgba[2] = inputPoint.m_rgba[2];
    outPoint.m_rgba[3] = inputPoint.m_rgba[3];

    outPoint.m_normal  = update.m_normal;
    outPoint.m_segmentId = update.m_segmentId;

    return outPoint;
};

template<typename PointType>
void PointCloudIndexer<PointType>::updatePointByUpDirection(const RCVector3d& pointIn, RCVector3d& pointOut, UP_DIRECTION upDirection)
{
	switch (upDirection)
	{
	case UP_DIRECTION_X:
		pointOut.x = -pointIn.z;
		pointOut.y = pointIn.y;
		pointOut.z = pointIn.x;
		break;
	case UP_DIRECTION_Y:
		pointOut.x = pointIn.x;
		pointOut.y = -pointIn.z;
		pointOut.z = pointIn.y;
		break;
	case UP_DIRECTION_Z:
		pointOut = pointIn;
		break;
	case UP_DIRECTION_NEGATIVE_X:
		pointOut.x = pointIn.z;
		pointOut.y = pointIn.y;
		pointOut.z = -pointIn.x;
		break;
	case UP_DIRECTION_NEGATIVE_Y:
		pointOut.x = pointIn.x;
		pointOut.y = pointIn.z;
		pointOut.z = -pointIn.y;
		break;
	case UP_DIRECTION_NEGATIVE_Z:
		pointOut.x = -pointIn.x;
		pointOut.y = pointIn.y;
		pointOut.z = -pointIn.z;
		break;
	}
}

bool proximityTest(const RCVector3d& newPoint, const RCVector3d& previousPoint)
{
	if (newPoint.length() != 0) // only do the proximity check for points with valid range values, otherwise it is a point with only RGB info and will be filtered during indexing
	{
		double distance = newPoint.distanceSqrd(previousPoint);

		// we use distance 0.5mm to filter duplicate points as this is enough for scanner resolution,
		// if the resolution improves, we also should update the value
		if (distance < 0.00000025) //0.0005 * 0.0005
		{
			return false;
		}
	}
	return true;
}

template<typename PointType>
AddPointResult PointCloudIndexer<PointType>::addPointToTempFile( const PointType& point)
{
    // change point by up direction here
    RCVector3d oldPoint( point.m_pos[0], point.m_pos[1], point.m_pos[2] );
    RCVector3d newPoint = oldPoint;
    updatePointByUpDirection( oldPoint, newPoint, m_upDirection );

    PointType pointOut = point;
    pointOut.m_pos[0] = newPoint.x;
    pointOut.m_pos[1] = newPoint.y;
    pointOut.m_pos[2] = newPoint.z;
    PointType pointIn = pointOut;

    //if there is an installed filter apply it here!
    if( m_pointCloudFilterPtr != NULL )
    {
        BasicOctreeImportPoint basicPointOut;
        if(!m_pointCloudFilterPtr->applyFilter( pointIn, basicPointOut ))
        {
            return AddPointResult::FILTERED;
        }
        pointOut = updateWithBasicOctreeImportPoint(pointIn, basicPointOut);
        pointIn = pointOut;
    }

    AddPointResult result = AddPointResult::OK;
    if (m_amountOfPointInTempFile > 0) {
        const auto& prevPoint = m_tempPoints[m_amountOfPointInTempFile - 1];
        if (!proximityTest(RCVector3d(pointOut.m_pos[0], pointOut.m_pos[1], pointOut.m_pos[2]), RCVector3d(prevPoint.m_pos[0], prevPoint.m_pos[1], prevPoint.m_pos[2])))
        {
			std::wstringstream ss;
			ss.precision(16);
			ss << L"Ignore point (";
			ss << pointOut.m_pos[0] << L", " << pointOut.m_pos[1] << L", " << pointOut.m_pos[2] << L") ";
			ss << L" - proximity culling";
			RCLog::getLogFile()->addMessage(ss.str().c_str());

            return AddPointResult::FILTERED;
        }
    }

    if (AddPointResult::OK == result)
    {
        //update intensity values
        if (pointOut.m_intensity > mIntensityMaxValue)
            mIntensityMaxValue = pointOut.m_intensity;
        if (pointOut.m_intensity < mIntensityMinValue)
            mIntensityMinValue = pointOut.m_intensity;

        //update bounding box
        m_scanBounds.updateBounds(RCVector3d(pointOut.m_pos[0], pointOut.m_pos[1], pointOut.m_pos[2]));
    }

    //add point to tempList
    if(m_tempPoints.empty()) m_tempPoints.resize(MAX_TEMP_POINTS_IN_FILE); //allocate memory if necessary
    m_tempPoints[m_amountOfPointInTempFile++] = pointOut;
    m_totalAmountOfPoints++;

    //write temp file
    if( MAX_TEMP_POINTS_IN_FILE == m_amountOfPointInTempFile )
    {
        bool wrote = writeNewTempFile();
        if( !wrote )
        {
            RCASSERTCORE( false, "Could not write temp file" );
            return AddPointResult::FATAL_ERROR;
        }

        //reset # of temp points  
        m_amountOfPointInTempFile = 0;
    }

    return result;
}

template<typename PointType>
bool PointCloudIndexer<PointType>::convertUmbrellaOctreeLeafNodes( std::wstring& errorLog, double minDistanceBetweenPoints  )
{
    errorLog += L"";
   
    {
        wstringstream ss;
        ss << L"Amount of Leaf Nodes ";
        ss << static_cast<int>(m_intermediateLeafNodeList.size()*UMBRELLA_LEAF_NODE_WIDTH);
        RCLog::getLogFile()->addMessage( ss.str().c_str() );
    }

    //if (getMultithreadIndexEnable())
    //{
    RCLog::getLogFile()->addMessage(L"Multi-thread Converting Umbrella Octree Leaf Nodes Begin");

    Time::RCTimer timer; timer.start();

    //multi-thread converting by TBB thread
    int leafNodeNum = static_cast<int>(m_intermediateLeafNodeList.size()>0 ? 
        ((m_intermediateLeafNodeList.size()-1)*UMBRELLA_LEAF_NODE_WIDTH + m_intermediateLeafNodeList[m_intermediateLeafNodeList.size()-1].size()) : 0);
    MultiThreadBuildOctreeFunctor<PointType> octreeFunctor(this, mLeafSplitStopSize, minDistanceBetweenPoints,mHasNormals,mCalculateNormalsFromUnstructuredData,&errorLog,&m_intermediateLeafNodeList);
    tbb::parallel_for(tbb::blocked_range<size_t>(0,leafNodeNum),octreeFunctor);

    //for (size_t i=0; i != leafNodeNum; i++) {
    //    if (getCancelled())
    //        return false;

    //    VoxelOctreeCreator<PointType> creator(this, 
    //        (m_intermediateLeafNodeList)[i/PointCloudIndexerInterface::UMBRELLA_LEAF_NODE_WIDTH][i%PointCloudIndexerInterface::UMBRELLA_LEAF_NODE_WIDTH], this->getTempFolder(), 
    //        true, 8, mLeafSplitStopSize,  minDistanceBetweenPoints );

    //    if (!this->isStructured() && !mHasNormals)
    //        creator.setCalculateNormalsFromUnstructuredData(mCalculateNormalsFromUnstructuredData);

    //    if (creator.createOctree(errorLog)){
    //    }

    //    //update progress
    //    this->incrementStageProgress(1.0 / leafNodeNum);
    //}

    //update progress
    //finishStageProgress();
    timer.stop();

    RCLog::getLogFile()->addMessage(L"Multi-thread Converting Umbrella Octree Leaf Nodes end");

    //calculate normal finished then update normal flag
    if( mCalculateNormalsFromUnstructuredData && !hasNormals())
    {
        setHasNormals( true );
    }

    return true;
}

template<typename PointType> 
void PointCloudIndexer<PointType>::setDensity(int density)
{
    if(density >1000) density=1000;
    else if(density <0) density=0;

    // convert mm to meter
    mLeafSplitStopSize = double(density)/1000;
}

template<typename PointType> 
void PointCloudIndexer<PointType>::setFilterType(NOISE_FILTER filterType)
{
    mFilterType = filterType;
}

template<typename PointType> 
NOISE_FILTER PointCloudIndexer<PointType>::getFilterType()
{
    return mFilterType;
}

template<typename PointType> 
RCVector3d PointCloudIndexer<PointType>::getScannerOrigin() const
{
    return m_scannerOrigin;
}

template<typename PointType> 
bool PointCloudIndexer<PointType>::getTerrestrial() const
{
    return m_isTerrestrial;
}

template<typename PointType> 
void PointCloudIndexer<PointType>::setTerrestrial( bool val )
{
    m_isTerrestrial = val;
}

template<typename PointType> 
std::uint8_t PointCloudIndexer<PointType>::getNormalizedIntensityValue(float minVal, float maxVal, float val )
{
    float range   = maxVal - minVal;
    if (range != 0.0f)
    {
        float normVal = ( val - minVal ) / range;
        if (normVal < 0.0f)
            return std::uint8_t(0.0f);

        return std::uint8_t( normVal * 255.0f );
    }
    else
    {
        return std::uint8_t( 255.0f );
    }
}

template<typename PointType>
void PointCloudIndexer<PointType>::addPointToUmbrellaTreeNode(OctreeIntermediateNode<PointType>* parentNodePtr, const PointType& point)
{
    if (parentNodePtr->m_currentPointList.size() > 0 && 
        parentNodePtr->m_currentPointList[parentNodePtr->m_currentPointList.size()-1].size() < UMBRELLA_LEAF_NODE_WIDTH)

        parentNodePtr->m_currentPointList[parentNodePtr->m_currentPointList.size()-1].push_back( point );

    else
    {
        std::vector<PointType> tempVec;
        tempVec.push_back( point );
        parentNodePtr->m_currentPointList.push_back(tempVec);
    }
}

template<typename PointType>
bool PointCloudIndexer<PointType>::addPointToUmbrellaTree( const PointType& point )
{
    OctreeIntermediateNode<PointType>* parentNodePtr = m_rootNodePtr;

    PointType outPoint = point;

    // need to apply intensity filter for files with true intensity values
    if (mHasIntensity && !getCancelled())
    {
        BasicOctreeImportPoint basicPointOut;
        if(!mIntensityFilter.applyFilter( outPoint, basicPointOut ))
        {
            return true;
        }
        outPoint = updateWithBasicOctreeImportPoint(outPoint, basicPointOut);
    }

    //traverse until empty
    while( parentNodePtr )
    {
        parentNodePtr->updatePointInformation( outPoint );

        //we reached a temp leaf
        if( parentNodePtr->m_isOctreeLeaf  ) 
        {
            try
            {
                addPointToUmbrellaTreeNode(parentNodePtr, outPoint);
            }
            catch (const std::exception& ex)
            {
                RCLog::getLogFile()->addMessage(L"add point error @ m_currentPointList!");
                throw ex;
            }
            return true;
        }
        else //no leaf found yet( or not created )
        {
            //get child information based on location of parent bbox
            OctreeChildInformation childInfo; 
            RCVector3d outPointPos(outPoint.m_pos[0], outPoint.m_pos[1], outPoint.m_pos[2]);
            getChildInformation( parentNodePtr->m_svoBounds, outPointPos, childInfo );

            OctreeIntermediateNode<PointType>* childNode = parentNodePtr->m_childList[ childInfo.m_childId ];

            //create new child & update 
            if( !childNode )  
            {
                childNode = new OctreeIntermediateNode<PointType>( parentNodePtr ); 
                //assign child node to parent node
                parentNodePtr->m_childList[ childInfo.m_childId ] = childNode;

                //fill in first time members
                childNode->m_svoBounds = childInfo.m_childBounds;

                //bounding box size
                double  bboxSize = childNode->m_svoBounds.getMax().x - childNode->m_svoBounds.getMin().x;

                //leaf node found or childNode's octree depth is 8, otherwise there will be too much files on disk which result in low performance
                if( bboxSize <= m_maxOctreeLeafBounds /*|| childNode->m_octreeDepth == 8*/ )
                {
                    int leafNodeNum = static_cast<int>(m_intermediateLeafNodeList.size()>0 ? 
                        ((m_intermediateLeafNodeList.size()-1)*UMBRELLA_LEAF_NODE_WIDTH + m_intermediateLeafNodeList[m_intermediateLeafNodeList.size()-1].size()) : 0);

                    //generate file name
                    std::wstringstream ss;

                    ss << getTempFolder();
                    ss << m_nodeName << L"_";
                    ss << NODE_FILE_NAME;
                    ss << leafNodeNum;
                    ss << TEMP_UMBRELLA_LEAF_NODE_EXTENSION;

                    childNode->m_fileName   = ss.str();
                    childNode->m_isOctreeLeaf = true;
                    //add node to intermediate node list
                    try
                    {
                        if (m_intermediateLeafNodeList.size() > 0 && 
                            m_intermediateLeafNodeList[m_intermediateLeafNodeList.size()-1].size() < UMBRELLA_LEAF_NODE_WIDTH)

                            m_intermediateLeafNodeList[m_intermediateLeafNodeList.size()-1].push_back( childNode );

                        else
                        {
                            std::vector< OctreeIntermediateNode<PointType>* > tempVec;
                            tempVec.push_back(childNode);
                            m_intermediateLeafNodeList.push_back(tempVec);
                        }
                    }
                    catch (const std::exception& ex)
                    {
                        RCLog::getLogFile()->addMessage(L"add point error @ m_intermediateLeafNodeList!");
                        throw ex;
                    }
                }
            }

            parentNodePtr = childNode;
        }
    }
    return false;
}

template<typename PointType>
void PointCloudIndexer<PointType>::removeTempFiles()
{
    auto files = Filesystem::list( getTempFolder() );
    int fileSize = static_cast<int>(files.size());

    for( int i = 0; i < fileSize; i++)
    {
        const std::wstring& file = files[i];
        const std::wstring& fileExtention = Filesystem::extension(file);

        if( 
            fileExtention.compare( ( TEMP_POINT_EXTENSION ) ) == 0  ||
            fileExtention.compare( ( TEMP_UMBRELLA_LEAF_NODE_EXTENSION ) ) == 0  ||
            fileExtention.compare( ( TEMP_VOXEL_OCTREE_EXTENSION ) ) == 0 ||
            fileExtention.compare( ( TEMP_VOXEL_TIMESTAMP_EXTENSION ) ) == 0
            )
        {
            Filesystem::remove( file );
        }
    }
}


template<typename PointType>
std::vector<OctreeIntermediateNode<PointType>* > PointCloudIndexer<PointType>::evaluateUmbrellaLeafNodes( std::wstring& errorLog )
{
    //we MUST split these nodes
    std::vector< OctreeIntermediateNode<PointType>* > nodesToBeSplit;

    for( size_t i = 0; i < m_intermediateLeafNodeList.size(); i++ )
    {
        for (size_t j = 0; j < m_intermediateLeafNodeList[i].size(); j ++)
        {
            OctreeIntermediateNode<PointType>* curNodePtr = m_intermediateLeafNodeList[i][j];
            //we need to split this node in smaller files, because it has more than the 
            //maximum allowed # of points, or 
            if( ( curNodePtr->m_totalAmountOfPoints >= MAX_POINTS_PER_OCTREE_LEAFNODE ) )
                nodesToBeSplit.push_back( curNodePtr );
        }
    }
    errorLog += L"";
    return nodesToBeSplit;
}

template<typename PointType>
bool PointCloudIndexer<PointType>::splitUmbrellaLeafNode( OctreeIntermediateNode<PointType>* parentPtr )
{
    //read in points from umbrella leaf node
    RCASSERTCORE( parentPtr->m_isOctreeLeaf, "Trying to split interior node" );

    {
        //not a leaf anymore
        parentPtr->m_isOctreeLeaf = false;
        //place between brackets so output stream closes
        {
            //open parent points
            fs::ifstream inputStream( parentPtr->m_fileName, std::ios::in | std::ios::binary );
            if( !inputStream.is_open() )
            {
                assert( false );

                return false;
            }

            //print some information
            //if( m_printToLog )
            {
                std::wstringstream ss;
                ss << L"Splitting parent : ( ";
                ss << parentPtr->m_fileName;
                ss << L" ) with # points: ";
                ss << parentPtr->m_totalAmountOfPoints;
                RCLog::getLogFile()->addMessage( ss.str( ).c_str() );
            }

            std::uint64_t amountOfPoints = parentPtr->m_totalAmountOfPoints;
            std::uint64_t totalPointsWritten = 0;

            // select half of points for leaf node when its depth is greater than 8, otherwise the split will run endlessly
            if(parentPtr->m_octreeDepth >= 8)
            {
                //amountOfPoints /= 2;
            }

            std::vector<int> childIds;
            for( std::uint64_t points = 0; points < amountOfPoints; points += MAX_TEMP_POINTS_IN_FILE )
            {
                std::uint64_t remaining = amountOfPoints - points;
                std::uint64_t toBeRead  = std::min( remaining, static_cast<std::uint64_t>(MAX_TEMP_POINTS_IN_FILE) ); 
                
                m_tempPoints.resize(static_cast<unsigned int>(toBeRead));
                inputStream.read( ( char* ) &m_tempPoints[0], sizeof( PointType ) * static_cast<int>( toBeRead ) );
                if (inputStream.fail())
                {
                    return false;
                }
                for( size_t i = 0; i < toBeRead; i++ )
                {
                    //get child information based on location of parent bbox
                    //OctreeChildInformation childInfo = getChildInformation( parentPtr->m_svoBounds, m_tempPoints[i] );

                    OctreeChildInformation childInfo; 
                    RCVector3d tempPoint(m_tempPoints[i].m_pos[0], m_tempPoints[i].m_pos[1], m_tempPoints[i].m_pos[2]);
                    getChildInformation( parentPtr->m_svoBounds, tempPoint, childInfo );
                    OctreeIntermediateNode<PointType>* childNode = parentPtr->m_childList[ childInfo.m_childId ];

                    // save unique child id, this will be used to check if all points fall into one child
                    if( find(childIds.begin(), childIds.end(), childInfo.m_childId) == childIds.end() )
                        childIds.push_back( childInfo.m_childId );

                    if( !childNode )
                    {
                        //create new child
                        childNode =  new OctreeIntermediateNode<PointType>( parentPtr ); 
                        //assign child node to parent node
                        parentPtr->m_childList[ childInfo.m_childId ] = childNode;

                        //generate file name
                        std::wstringstream ss;
                        ss << getTempFolder();
                        ss << m_nodeName << L"_";
                        ss << SPLIT_NODE_FILE_NAME;
                        ss << m_amountOfSplitNodes++;
                        ss << TEMP_UMBRELLA_LEAF_NODE_EXTENSION;

                        //fill in first time members
                        childNode->m_svoBounds = childInfo.m_childBounds;
                        childNode->m_isOctreeLeaf = true;
                        childNode->m_fileName = ss.str();

                        //add new child to leaf list
                        if (m_intermediateLeafNodeList.size() > 0 && 
                            m_intermediateLeafNodeList[m_intermediateLeafNodeList.size()-1].size() < UMBRELLA_LEAF_NODE_WIDTH)

                            m_intermediateLeafNodeList[m_intermediateLeafNodeList.size()-1].push_back( childNode );

                        else
                        {
                            std::vector<OctreeIntermediateNode<PointType>*> tempVec;
                            tempVec.push_back(childNode);
                            m_intermediateLeafNodeList.push_back(tempVec);
                        }
                    }

                    //update point information
                    childNode->updatePointInformation( m_tempPoints[i] );

                    //add current point to child node
                    if (childNode->m_currentPointList.size() > 0 && 
                        childNode->m_currentPointList[childNode->m_currentPointList.size()-1].size() < UMBRELLA_LEAF_NODE_WIDTH)

                        childNode->m_currentPointList[childNode->m_currentPointList.size()-1].push_back( m_tempPoints[i] );

                    else
                    {
                        std::vector<PointType> tempVec;
                        tempVec.push_back(m_tempPoints[i]);
                        childNode->m_currentPointList.push_back(tempVec);
                    }
                }

                //write new leaf nodes 
                for( int i = 0; i < 8; i++ )
                {
                    if( parentPtr->m_childList[i] )
                    {
                        int numPoints = static_cast<int> (parentPtr->m_childList[i]->m_currentPointList.size() > 0 ? 
                            ((parentPtr->m_childList[i]->m_currentPointList.size()-1)*UMBRELLA_LEAF_NODE_WIDTH + 
                            parentPtr->m_childList[i]->m_currentPointList[parentPtr->m_childList[i]->m_currentPointList.size()-1].size()) : 0); 

                        if( numPoints )
                        {
                            //if( m_printToLog )
                            {
                                std::wstringstream ss;
                                ss << L"\tWriting Child ( ";
                                ss << parentPtr->m_childList[i]->m_fileName;
                                ss << L" ) with # points: ";
                                ss << numPoints;
                                RCLog::getLogFile()->addMessage( ss.str( ).c_str() );
                            }

                            parentPtr->m_childList[i]->writeTemporaryPointsToFile();
                            totalPointsWritten += numPoints;
                        }
                    }
                }
            }

            inputStream.close();
            if (inputStream.fail())
            {
                return false;
            }

            assert( amountOfPoints == totalPointsWritten );

            //if( m_printToLog )
            {
                std::wstringstream ss;
                ss << L"\t\t Total amount of child points written: ";
                ss << totalPointsWritten;
                RCLog::getLogFile()->addMessage( ss.str( ).c_str() );
            }

            // for unstructured data, if there is only one child when octree depth is deeper than 8, 
            // we need to remove the duplicate points, otherwise split may be endless
            if( parentPtr->m_octreeDepth >= 8 && childIds.size() == 1 )
            {
                RCLog::getLogFile()->addMessage(L"try to remove duplicate nodes");
                try
                {
                    int childId = childIds.at(0);
                    if( parentPtr->m_childList[childId] )
                    {
                        // read all points
                        std::wstring fileName = parentPtr->m_childList[childId]->m_fileName;
                        inputStream.open( fileName, std::ios::in | std::ios::binary );
                        if (!inputStream.is_open())
                        {
                            return false;
                        }
                        int lengthOfName = static_cast<int>(fileName.length());
                        std::wstring newFileName = fileName.substr(0, 
                            lengthOfName - wcslen(TEMP_UMBRELLA_LEAF_NODE_EXTENSION)) + L"_r" + TEMP_UMBRELLA_LEAF_NODE_EXTENSION;
                        fs::ofstream outputStream( newFileName, std::ios::out | std::ios::binary );
                        if (!outputStream.is_open())
                        {
                            return false;
                        }
                        std::uint64_t cacheSize = 1000;
                        double mindDistSqr = 0.0005 * 0.0005;
                        int totalPoints = 0;
                        bool hasDuplicatePoints = false;
                        std::uint64_t totalAmountsOfPoints = parentPtr->m_childList[childId]->m_totalAmountOfPoints;
                        for( std::uint64_t pointsRead = 0; pointsRead < totalAmountsOfPoints; pointsRead += cacheSize )
                        {
                            std::vector<bool> flags;
                            flags.resize( static_cast<size_t>(cacheSize), false );
                            std::uint64_t remaining = totalAmountsOfPoints - pointsRead;
                            std::uint64_t toBeRead  = std::min( remaining, cacheSize );
                            PointType* pointsArr = new PointType[ static_cast<int>(toBeRead) ];
                            inputStream.read( ( char* ) pointsArr, sizeof( PointType ) * static_cast<int>(toBeRead) );

                            // remove duplicate points
                            for( size_t k = 0; k < toBeRead; k++ )
                            {
                                if( flags[k] ) continue; // skip already labeled points

                                RCVector3d p1( pointsArr[k].m_pos[0], pointsArr[k].m_pos[1], pointsArr[k].m_pos[2] ); //first point

                                for( size_t j = 0; j < toBeRead; j++ )
                                {
                                    //no self testing or already removed
                                    if( ( k == j ) || ( flags[j] ) )
                                        continue;

                                    RCVector3d p2( pointsArr[j].m_pos[0], pointsArr[j].m_pos[1], pointsArr[j].m_pos[2] ); //test point

                                    double newDistance = p1.distanceSqrd( p2 );
                                    if( newDistance < mindDistSqr ) //invalid point
                                        flags[j] = true;
                                }
                            }

                            // write valid points to new temp file
                            std::vector<PointType> pointList;
                            for( size_t pointIndex = 0; pointIndex < toBeRead; pointIndex++ )
                            {
                                if( !flags[pointIndex] )
                                    pointList.push_back(pointsArr[pointIndex]);
                            }

                            // check whether there are duplicate points for first 1000 points
                            if( pointsRead == 0 && pointList.size() < toBeRead )
                            {
                                hasDuplicatePoints = true;
                            }
                            else if( pointsRead == 0 && pointList.size() == toBeRead )
                            {
                                // skip the file as there are no duplicate points in first 1000 points
                                break;
                            }

							PointType* validPointsArr = new PointType[pointList.size()];

                            for( size_t pointIndex = 0; pointIndex < pointList.size(); pointIndex++ )
                                validPointsArr[pointIndex] = pointList.at(pointIndex);
                            outputStream.write( (char*)validPointsArr, sizeof(PointType) * pointList.size() );
                            totalPoints += static_cast<int>(pointList.size());
                            delete[] validPointsArr;
                            delete[] pointsArr;
                        }

                        inputStream.close();
                        outputStream.close();

                        if (inputStream.fail() || outputStream.fail())
                        {
                            return false;
                        }

                        if(hasDuplicatePoints)
                        {
                            RCLog::getLogFile()->addMessage(L"has duplicate points");

                            // remove old file
                            Filesystem::remove( parentPtr->m_childList[childId]->m_fileName );

                            // set new file name and total amount of points
                            parentPtr->m_childList[childId]->m_totalAmountOfPoints = totalPoints;

                            // rename file
                            Filesystem::rename( newFileName, parentPtr->m_childList[childId]->m_fileName );
                        }
                        else
                        {
                            // remove new file
                            Filesystem::remove( newFileName );
                        }
                    }
                }
                catch (const std::exception&)
                {
                }
            }
        }//output stream closes


        //need to remove parent node from leaf list
        typename std::vector< OctreeIntermediateNode<PointType>* >::iterator iter;
        for (size_t i = 0; i < m_intermediateLeafNodeList.size(); i ++)
        {
            for( iter = m_intermediateLeafNodeList[i].begin(); iter != m_intermediateLeafNodeList[i].end();  )
            {
                if( *iter == parentPtr )
                {
                    //remove old 'temp' file
                    Filesystem::remove( parentPtr->m_fileName );
                    //reset file name
                    parentPtr->m_fileName = L"";
                    //remove from leaf list
                    iter = m_intermediateLeafNodeList[i].erase( iter );
                }
                else
                    iter++;
            }
        }
        //done
        return true;
    }
}

template<typename PointType>
bool PointCloudIndexer<PointType>::getNormalizeIntensity() const
{
    return mNormalizeIntensity;
}

template<typename PointType>
void PointCloudIndexer<PointType>::setNormalizeIntensity( bool val )
{
    mNormalizeIntensity = val;
}

template<typename PointType>
void PointCloudIndexer<PointType>::setHasRGB( bool hasRGB )
{
    mHasRGB = hasRGB;
}

template<typename PointType>
bool PointCloudIndexer<PointType>::hasRGB() const
{
    return mHasRGB;
}

template<typename PointType>
void PointCloudIndexer<PointType>::setHasNormals( bool hasNormals )
{
    mHasNormals = hasNormals;
}

template<typename PointType>
bool PointCloudIndexer<PointType>::hasNormals() const
{
    return mHasNormals;
}

template<typename PointType>
void PointCloudIndexer<PointType>::setHasTimeStamp( bool hasTimeStamp )
{
    mHasTimeStamp = hasTimeStamp;
}

template<typename PointType>
bool PointCloudIndexer<PointType>::hasTimeStamp() const
{
    return mHasTimeStamp;
}


template<typename PointType>
void PointCloudIndexer<PointType>::setHasClassification(bool hasClassification)
{
    mHasClassification = hasClassification;
}

template<typename PointType>
bool PointCloudIndexer<PointType>::hasClassification() const
{
    return mHasClassification;
}


template<typename PointType>
void PointCloudIndexer<PointType>::setHasIntensity( bool hasIntensity )
{
    mHasIntensity = hasIntensity;
}

template<typename PointType>
bool PointCloudIndexer<PointType>::hasIntensity() const
{
    return mHasIntensity;
}

template<typename PointType>
void PointCloudIndexer<PointType>::setCalculateNormalsFromUnstructuredData(bool calculateNormals)
{
    mCalculateNormalsFromUnstructuredData = calculateNormals;
}

template<typename PointType>
bool PointCloudIndexer<PointType>::getCalculateNormalsFromUnstructuredData()
{
    return mCalculateNormalsFromUnstructuredData;
}

template<typename PointType>
void PointCloudIndexer<PointType>::setEnableRangeClipping(bool enableRangeClipping)
{
    mRangeClippingEnabled = enableRangeClipping;
}

template<typename PointType>
void PointCloudIndexer<PointType>::setRangeFilter(const RangeFilter& rangeFilter)
{
    mRangeFilter = rangeFilter;
}

template<typename PointType>
void PointCloudIndexer<PointType>::setIntensityFilter(const IntensityFilter& intensityFilter)
{
    mIntensityFilter = intensityFilter;
}

template<typename PointType>
int PointCloudIndexer<PointType>::getNumScans() const
{
    if(mScanIdToTempFilesMap.size() == 0)
        return 1;

    return static_cast<int>(mScanIdToTempFilesMap.size());
}

template<typename PointType>
int PointCloudIndexer<PointType>::getCurrentScanId() const
{
    return mCurrentScanId;
}

template<typename PointType>
void PointCloudIndexer<PointType>::setCurrentScanId( int scanId )
{
    if( scanId == 0)
    {
        mCurrentScanId = scanId;
        // change temp files
        if(mScanIdToTempFilesMap.count(scanId) > 0)
            m_tempFileNames = mScanIdToTempFilesMap[scanId];
    }
    else if( scanId > 0 && scanId < static_cast<int>(mScanIdToTempFilesMap.size()) )
    {
        mCurrentScanId = scanId;
        // change temp files
        if(mScanIdToTempFilesMap.count(scanId) > 0)
            m_tempFileNames = mScanIdToTempFilesMap[scanId];
        else
            RCLog::getLogFile()->addMessage(L"Can't find scan id from ScanIdToTempFilesMap");
    }
}

template<typename PointType>
void PointCloudIndexer<PointType>::updateRegistrationInfo( RCVector3d rotation, RCVector3d translation )
{
    m_rotation = rotation;
    m_translation = translation;
}

template<typename PointType>
void PointCloudIndexer<PointType>::getRegistrationInfo(RCVector3d& rotation, RCVector3d& translation) const {
    rotation = m_rotation;
    translation = m_translation;
}


template<typename PointType>
void PointCloudIndexer<PointType>::convertLeafNodeToLeafSaveData(std::vector<OctreeLeafSaveData>& leafSaveList)
{
	for (int i = 0; i < static_cast<int>(m_intermediateLeafNodeList.size()); i++)
	{
		for (int j = 0; j < static_cast<int>(m_intermediateLeafNodeList[i].size()); j++)
		{
			OctreeLeafSaveData saveLeaf;
			saveLeaf.setFromIntermediateNode<PointType>(m_intermediateLeafNodeList[i][j]);
			m_intermediateLeafNodeList[i][j]->m_indexInToLeafArray = i * UMBRELLA_LEAF_NODE_WIDTH + j; //assign index
			leafSaveList.push_back(saveLeaf);
		}
	}
}

template<typename PointType>
ambergris::RealityComputing::Common::RCCode PointCloudIndexer<PointType>::getErrorCode( )
{
    return mErrorCode;
}

template<typename PointType>
bool PointCloudIndexer<PointType>::isEmptyScan() const
{
    if(m_totalAmountOfPoints==0)
    {
        mErrorCode = rcEmptyFile;
        return true;
    }
    return false;
}

template<typename PointType>
bool PointCloudIndexer<PointType>::isBadBounds() const
{
    auto boundsLength = m_scanBounds.getMax().distanceSqrd(m_scanBounds.getMin());
    if(isnan(boundsLength)||isinf(boundsLength))
    {
        mErrorCode = rcIndexFailed;
        return true;
    }

    return false;
}

namespace ambergris {
	namespace RealityComputing {
		namespace Import
		{
			template class PointCloudIndexer<BasicOctreeImportPoint>;
			template class PointCloudIndexer<ExtendedOctreeImportPoint>;
		}
	}
}
