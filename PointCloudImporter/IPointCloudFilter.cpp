#include <cmath>
#include <sstream>
#include <algorithm>

#include <common/RCString.h>
#include <utility/RCLog.h>
#include <utility/RCAssert.h>

#include "IPointCloudFilter.h"
#include "OctreeImportPoint.h"

using namespace ambergris::RealityComputing::Import;
using namespace ambergris::RealityComputing::Utility::Log;
using namespace ambergris::Infrastructure::SpatialReference;
using namespace ambergris::RealityComputing::Common;

IPointCloudFilter::IPointCloudFilter() {}
IPointCloudFilter::~IPointCloudFilter() {}

bool IPointCloudFilter::applyFilter(const BasicOctreeImportPoint& /*inputPoint*/, BasicOctreeImportPoint& /*outPoint*/) const
{
    return false;
}
//////////////////////////////////////////////////////////////////////////
// RangeFilter
//////////////////////////////////////////////////////////////////////////
int RangeFilter::mFilteredPoints = 0;
RangeFilter::RangeFilter( double minRange, double maxRange )
{
    mMinRange = minRange;
    mMaxRange = std::max(minRange, maxRange);
    mFilteredPoints = 0;
}

RangeFilter::~RangeFilter()
{
    if( mFilteredPoints > 0)
    {
        std::wstringstream ws;
        ws << "Points filtered by RangeFilter:";
        ws << mFilteredPoints;
        RCLog::getLogFile()->addMessage(ws.str().c_str());
    }
}

bool RangeFilter::applyFilter( const BasicOctreeImportPoint& inputPoint, BasicOctreeImportPoint& outPoint ) const
{
    RCVector3d vec( inputPoint.m_pos[0], inputPoint.m_pos[1], inputPoint.m_pos[2] );
    double length = vec.length();
    if( length >= mMinRange && length <= mMaxRange )
    {
        outPoint = inputPoint;
        return true;
    }

    mFilteredPoints++;
    return false;
}

std::wstring RangeFilter::getFilterName() const
{
    return L"RangeImportFilter";
}

void RangeFilter::updateRange( double minRange, double maxRange )
{
    RCASSERTCORE( minRange >= 0,        "Invalid filter range" );
    RCASSERTCORE( minRange < maxRange,  "Invalid filter range" );

    if(minRange < 0)
        minRange = 0.0;

    if( maxRange < minRange )
        maxRange = minRange;

    mMinRange = minRange;
    mMaxRange = maxRange;
}


//////////////////////////////////////////////////////////////////////////
// IntensityFilter
//////////////////////////////////////////////////////////////////////////
int IntensityFilter::mFilteredPoints = 0;
IntensityFilter::IntensityFilter( double minRange, double maxRange )
{
    mMinRange = minRange;
    mMaxRange = std::max(minRange, maxRange);
    mFilteredPoints = 0;
}

IntensityFilter::~IntensityFilter()
{
    if( mFilteredPoints > 0)
    {
        std::wstringstream ws;
        ws << "Points filtered by IntensityFilter:";
        ws << mFilteredPoints;
        RCLog::getLogFile()->addMessage(ws.str().c_str());
    }

}

bool IntensityFilter::applyFilter( const BasicOctreeImportPoint& inputPoint, BasicOctreeImportPoint& outPoint ) const
{
    // convert to percentage
    double percentage = inputPoint.m_rgba[3] * 100.0 / 255.0;
    
    // if intensity value is in range
    if((percentage >= mMinRange) && (percentage <= mMaxRange))
    {
        outPoint = inputPoint;
        return true;
    }

    mFilteredPoints++;
    return false;
}

std::wstring IntensityFilter::getFilterName() const
{
    return L"IntensityImportFilter";
}

void IntensityFilter::updateRange( double minRange, double maxRange )
{
    RCASSERTCORE( minRange < maxRange, "Invalid intensity range" );

    if(minRange < 0)
        minRange = 0.0;

    if( maxRange < minRange )
        maxRange = minRange;

    mMinRange = minRange;
    mMaxRange = maxRange;
}

//////////////////////////////////////////////////////////////////////////
//Class NullFilter
//////////////////////////////////////////////////////////////////////////
bool NullFilter::applyFilter( const BasicOctreeImportPoint& inputPoint, BasicOctreeImportPoint& outPoint ) const
{
    outPoint = inputPoint;

    return true;
}

std::wstring NullFilter::getFilterName() const
{
    return L"NullImportFilter";
}
