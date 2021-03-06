#pragma once

#include <memory>
#include <vector>

#include <common/RCCode.h>
#include <common/RCScanProperties.h>
#include <common/RCUserDataSource.h>
#include <common/RCVectorFwd.h>

#include "OctreeDefs.h"

namespace ambergris { namespace RealityComputing {
 namespace Common { struct RCBox; class RCOrientedBoundingBox; }
namespace Import {
class IPointCloudFilter;
class IntensityFilter;
class RangeFilter;

class PointCloudIndexerInterface
{
public:
    // \brief: Destructor
    //////////////////////////////////////////////////////////////////////////
    virtual                                             ~PointCloudIndexerInterface();

    //////////////////////////////////////////////////////////////////////////
    // \brief: get/set the total bounding box of this scan
    //////////////////////////////////////////////////////////////////////////
    virtual const ambergris::RealityComputing::Common::RCOrientedBoundingBox&       getOrientedBoundingBox() const = 0;
    virtual void  setOrientedBoundingBox(const ambergris::RealityComputing::Common::RCOrientedBoundingBox& obb) = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: Returns the total bounding box of this scan
    //////////////////////////////////////////////////////////////////////////
    virtual const ambergris::RealityComputing::Common::RCBox&                       getBoundingBox() const = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: Returns the translation of this scan
    //////////////////////////////////////////////////////////////////////////
    virtual const ambergris::RealityComputing::Common::RCVector3d&                   getTranslation() const = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: Returns the rotation of this scan
    //////////////////////////////////////////////////////////////////////////
    virtual const ambergris::RealityComputing::Common::RCVector3d&                   getRotation() const = 0;

    // \brief: Returns if this a structured scan
    //////////////////////////////////////////////////////////////////////////

    virtual bool                                        hasTimeStamp() const = 0;
    virtual bool                                        hasRGB() const = 0;
    virtual bool                                        hasIntensity() const = 0;
    virtual bool                                        hasClassification() const = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: Returns the output folder 
    //////////////////////////////////////////////////////////////////////////
    virtual std::wstring                                getOutputFolder() const = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: Sets the output filename
    //////////////////////////////////////////////////////////////////////////
    virtual void                                        setOutputFolder(const std::wstring& val) = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: Sets the input scan's name and (parent) filepath
    //////////////////////////////////////////////////////////////////////////
    virtual void                                        setInput(const std::wstring& name, const std::wstring& filepath) = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: Sets the input scans up directoin
    //////////////////////////////////////////////////////////////////////////
    virtual void                                        setUpDirection(UP_DIRECTION val) = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: Sets the temp. folder
    //////////////////////////////////////////////////////////////////////////
    virtual void                                        setTempFolder(const std::wstring& val) = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: get the temp. folder
    //////////////////////////////////////////////////////////////////////////
    virtual std::wstring                                getTempFolder() const = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: Returns the node name( file name without path & extension );
    //////////////////////////////////////////////////////////////////////////
    virtual const std::wstring&                         getNodeName() const = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: Returns if this a terrestrial scan
    //////////////////////////////////////////////////////////////////////////
    virtual void                                        setTerrestrial(bool val) = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: Returns the installed point cloud filter, can be NULL
    //////////////////////////////////////////////////////////////////////////
    virtual IPointCloudFilter*                          getPointCloudFilter() = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: Sets the point cloud filter
    //////////////////////////////////////////////////////////////////////////
    virtual void                                        setPointCloudFilter(std::unique_ptr<IPointCloudFilter> filter) = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: Returns if the indexing is canceled
    //////////////////////////////////////////////////////////////////////////
    virtual bool                                        getCancelled() const = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: Sets the indexing process to be canceled
    //////////////////////////////////////////////////////////////////////////
    virtual void                                        setCancelled(bool val) = 0;
    //////////////////////////////////////////////////////////////////////////
    // \brief: Removes all the temp files in the temp folder
    //////////////////////////////////////////////////////////////////////////
    virtual void                                        removeTempFiles() = 0;

    //////////////////////////////////////////////////////////////////////////
    //\brief: Sets if we should normalize intensity values 
    //////////////////////////////////////////////////////////////////////////
    virtual void                                        setNormalizeIntensity(bool val)  = 0;

    virtual void                                        setFilterType(NOISE_FILTER filterType) = 0;
    virtual NOISE_FILTER                                getFilterType() = 0;

    virtual void                                        setDensity(int density) = 0;

    virtual void                                        setHasNormals(bool hasNormals) = 0;
    virtual bool                                        hasNormals() const = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief:This should be called before the actual parsing of points
    //          to set up members/variables, error message can be specified
    //////////////////////////////////////////////////////////////////////////
    virtual Common::IRCUserDataSource::ErrorCode        parsePoints(std::wstring& errorLog) = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief:This should be called after parsing of points
    //          to set up members/variables, error message can be specified
    //////////////////////////////////////////////////////////////////////////
    virtual Common::IRCUserDataSource::ErrorCode        postParse(std::wstring& errorLog) = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: Call this directly after 'parsePoints()' to convert the 
    //           data set
    //////////////////////////////////////////////////////////////////////////
    virtual bool                                        convertToNativeFormat(std::wstring& errorLog) = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: set whether calculate normals from unstructured data
    //////////////////////////////////////////////////////////////////////////
    virtual void                                        setCalculateNormalsFromUnstructuredData(bool calculateNormals) = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: get whether calculate normals from unstructured data
    //////////////////////////////////////////////////////////////////////////
    virtual bool                                        getCalculateNormalsFromUnstructuredData() = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: set intensity filter
    //////////////////////////////////////////////////////////////////////////
    virtual void                                        setIntensityFilter(const IntensityFilter& intensityFilter) = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: enable / disable range filter
    //////////////////////////////////////////////////////////////////////////
    virtual void                                        setEnableRangeClipping(bool enableRangeClipping) = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: set range filter
    //////////////////////////////////////////////////////////////////////////
    virtual void                                        setRangeFilter(const RangeFilter& rangeFilter) = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: get the scan number as one file can have multiple scans
    //////////////////////////////////////////////////////////////////////////
    virtual int                                         getNumScans() const = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: get the scan number as one file can have multiple scans, scan index start with 0
    //////////////////////////////////////////////////////////////////////////
    virtual void                                        setCurrentScanId(int scanId) = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: update registration information
    //////////////////////////////////////////////////////////////////////////
    virtual void                                        updateRegistrationInfo(ambergris::RealityComputing::Common::RCVector3d rotation, ambergris::RealityComputing::Common::RCVector3d translation) = 0;

    virtual void                                        getRegistrationInfo(ambergris::RealityComputing::Common::RCVector3d& rotation, ambergris::RealityComputing::Common::RCVector3d& translation) const = 0;

    //////////////////////////////////////////////////////////////////////////
    // \brief: get error code
    //////////////////////////////////////////////////////////////////////////
    virtual ambergris::RealityComputing::Common::RCCode  getErrorCode() = 0;

    virtual bool isEmptyScan() const = 0;
    virtual bool isBadBounds() const = 0;

};
}}}
