#include <common/RCUserDataSource.h>
#include <common/RCMath.h>

#include "DataSourceIndexer.h"

#include <utility/RCAssert.h>
#include <utility/RCFilesystem.h>

#include "OctreeDefs.h"

#include <sstream>

using namespace ambergris::RealityComputing;
using namespace ambergris::RealityComputing::Import;

template< typename PointType >
struct DataSourceIndexer< PointType >::Impl
{
    Impl( Common::IRCUserDataSource& dataSource, const std::wstring& sourceName ) 
        : mDataSource( dataSource )
        , mSourceName( sourceName )
    {}

    Common::IRCUserDataSource& mDataSource;
    std::wstring mSourceName;

private:
    // Disable assignment operator
    Impl( const Impl& );
    Impl& operator=( Impl& );
};




template< typename PointType >
DataSourceIndexer<PointType>::DataSourceIndexer( Common::IRCUserDataSource& dataSource, 
    const std::wstring& sourceName,
    bool isTerrestrial)
    : mImpl( new Impl( dataSource, sourceName ) )
{
    this->m_isTerrestrial = isTerrestrial;
    if (this->m_isTerrestrial)
        this->m_maxOctreeLeafBounds = MAX_SIZE_TERRESTRIAL_UMBRELLA_LEAFNODE;
    else
        this->m_maxOctreeLeafBounds = MAX_SIZE_LIDAR_UMBRELLA_LEAFNODE;
}

template< typename PointType >
DataSourceIndexer<PointType>::~DataSourceIndexer()
{
    // Do nothing. This is to allow mImpl to be deleted.
}

namespace{

    bool readBasicPoint( Common::IRCUserDataSource& source, BasicOctreeImportPoint& pointOut )
    {
        if( ! source.advancePoint() )
            return false;

        source.getPosition(pointOut.m_pos);
        source.getColor(pointOut.m_rgba);
        source.getIntensity(pointOut.m_intensity);
        source.getNormal(pointOut.m_normal);
        source.getSegmentId(pointOut.m_segmentId);
        source.getClassification(pointOut.m_misc[0]);

        return true;
    }

    bool readPoint( Common::IRCUserDataSource& source, BasicOctreeImportPoint& pointOut )
    {
        return readBasicPoint(source,pointOut);
    }

    bool readPoint( Common::IRCUserDataSource& source, ExtendedOctreeImportPoint& pointOut )
    {
        if(!readBasicPoint(source,pointOut)) 
            return false;

        source.getTimestamp(pointOut.m_reserved);
        return true;
    }
}


template< typename PointType >
Common::IRCUserDataSource::ErrorCode DataSourceIndexer<PointType>::parsePoints( std::wstring& errorLog )
{
    this->m_totalAmountOfPoints = 0;
    this->m_amountOfPointInTempFile = 0;
    this->m_estimatedAmountOfPoints = mImpl->mDataSource.getPointCt(); //get an estimate of the number of points before parsing for progress reporting

    PointType point;
    while (readNextPoint(point))
    {
        if (this->addPointToTempFile(point) == AddPointResult::FATAL_ERROR)
        {
            errorLog += L"DataSourceIndexer::parsePoints() --> Internal error";
            return Common::IRCUserDataSource::ErrorCode::UnknownError;
        }

        if (this->getCancelled())
        {
            errorLog += L"DataSourceIndexer::parsePoints() --> Cancelled";
            return Common::IRCUserDataSource::ErrorCode::Cancelled;
        }
    }
	auto errorCode = mImpl->mDataSource.getLastError();
    this->writeNewTempFile();
    this->m_tempPoints.clear(); //clear out temp points
    this->m_tempPoints.shrink_to_fit();
   

    this->mScanIdToTempFilesMap[0] = this->m_tempFileNames;

    setScanFormat();

    return errorCode;
}


template< typename PointType >
void DataSourceIndexer<PointType>::setScanFormat()
{
    const auto& scanMeta = mImpl->mDataSource.getMetaData();

    scanMeta.rotation.getAsEulerAngles(this->m_rotation);
    this->m_translation = scanMeta.translation;

    // Set format properties
    auto format = scanMeta.format;
    this->setHasClassification(format.hasClassification);
    this->setHasIntensity(format.hasIntensity);
    this->setHasNormals(format.hasNormals);
    this->setHasRGB(format.hasRGB);
    this->setHasTimeStamp(format.hasTimestamp);
}

template< typename PointType >
bool DataSourceIndexer<PointType>::convertToNativeFormat( std::wstring& errorLog )
{
    // change node name for multiple scans
    if( this->getNumScans() > 1 )
    {
        this->m_nodeName = Utility::Filesystem::basename( this->mImpl->mSourceName );
    }

    // clear nodes
    for (auto& iln : this->m_intermediateLeafNodeList)
    {
        iln.clear();
    }
    this->m_intermediateLeafNodeList.clear();

    //m_translation = mImpl->mDataSource.getRegistration().translation;
    //mImpl->mDataSource.getRegistration().rotation.getAsEulerAngles( m_rotation );

    if( !this->generateUmbrellaOctree( errorLog ) )
    {
        errorLog += L"DataSourceIndexer::convertToNativeFormat() Error Generating";
        return false;
    }
    if( !this->convertUmbrellaOctreeLeafNodes( errorLog, 0.003 ) )
    {
        errorLog += L"DataSourceIndexer::convertToNativeFormat() Error Converting";
        return false;
    }

    return true;
}

template< typename PointType >
Common::IRCUserDataSource::ErrorCode DataSourceIndexer<PointType>::postParse( std::wstring& /*errorLog*/ )
{
    mImpl->mDataSource.close();
    return mImpl->mDataSource.getLastError();
}

template< typename PointType >
bool DataSourceIndexer<PointType>::readNextPoint( PointType& pointOut )
{
    return readPoint( mImpl->mDataSource, pointOut );
}

namespace ambergris { namespace RealityComputing { namespace Import {

    // Instanciate the template with common types, so that other modules can link against them
    template class DataSourceIndexer<BasicOctreeImportPoint>;
    template class DataSourceIndexer<ExtendedOctreeImportPoint>;
	
}}} // ns
