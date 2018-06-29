#include <sstream>

#include <common/RCString.h>
#include <common/RCScanProperties.h>
#include "LasUtils.h"
#include "LidarFileManager.h"
#include <utility/RCFilesystem.h>
#include <utility/RCSpatialReference.h>
#include <utility/RCStringUtils.h>

#ifdef _WIN32
#include <utility/RCSpatialReference.h>
#endif

using namespace ambergris::RealityComputing::Import;
using namespace ambergris::RealityComputing::Common;
using namespace ambergris::RealityComputing::Utility;

void LasUtils::interpretKeyEntry(const RCLASvariable_header_key_entry& variable_header_key_entry,
    int& epsgCode, LinearUnitsGeoKey& linearUnit, GeogAngularUnitsGeoKey& angularUnit)
{
    // 2048: for Geographic CS Type Codes
    // 2048: This key may be used to specify the code for 
    // the geographic coordinate system used to map lat-long 
    // to a specific ellipsoid over the earth.
    if (epsgCode == -1 && variable_header_key_entry.key_id == 2048)
    {
        epsgCode = variable_header_key_entry.value_offset;
    }
    // 3072 for Projected CS Type Codes, overwrite previous epsgCode anyway
    else if (variable_header_key_entry.key_id == 3072)
    {
        epsgCode = variable_header_key_entry.value_offset;
    }

    //2054: Allows the definition of geocentric CS Linear units for user-defined GCS and for ellipsoids.
    if (variable_header_key_entry.key_id == 2054)
        angularUnit = (GeogAngularUnitsGeoKey)variable_header_key_entry.value_offset;

    //2052: Allows the definition of geocentric CS linear units for user-defined GCS.
    //3076: Defines linear units used by this projection
    if (variable_header_key_entry.key_id == 2052 || variable_header_key_entry.key_id == 3076)
        linearUnit = (LinearUnitsGeoKey)variable_header_key_entry.value_offset;
}

RCString LasUtils::getCoordinateSystemFromFile(const RCString& fileName)
{
    std::wstring csCode = L"";

    if (!Filesystem::exists(fileName.w_str()))
    {
        return RCString(csCode.c_str());
    }

#ifdef _WIN32
    int epsgCode = -1;
    LinearUnitsGeoKey linearUnit;
    GeogAngularUnitsGeoKey angularUnit;

    const std::wstring fileExtention = Filesystem::lowercaseExtension(fileName.w_str());
    if (String::RCStringUtils::wequal(fileExtention, LAS_FILE_EXTENSION, true) || String::RCStringUtils::wequal(fileExtention, LAZ_FILE_EXTENSION, true))
    {
        LidarFileManager lidarFileMgr;
        bool success = lidarFileMgr.open(fileName.w_str());
        if (success)
        {
            lidarFileMgr.ReadCoordinateSystemInfo(epsgCode, linearUnit, angularUnit);
            lidarFileMgr.close();
        }
    }

    if (epsgCode > 0)
    {
        std::wstringstream strEpsgCode;
        strEpsgCode << epsgCode;

        //csCode = RCSpatialReferenceUtils::getADSKCoordinateSystemCode(strEpsgCode.str().c_str()).w_str();
    }

#endif

    return RCString(csCode.c_str());
}


#ifdef _WIN32
double LasUtils::checkUnitScale(int epsgCode, int linearUnitsGeoKey, const std::wstring& userSetOrigCS)
{
    double unitScale = 1.0;

    if (userSetOrigCS != L"")
    {
        std::wstringstream strEpsgCode;
        strEpsgCode << epsgCode;
		/*std::wstring adskCS = RCSpatialReferenceUtils::getADSKCoordinateSystemCode(strEpsgCode.str().c_str()).w_str();
		if (userSetOrigCS != adskCS)
			adskCS = userSetOrigCS;*/

        bool isGeographics = false;
        bool isLinearUnit = false;
        RCCode result = RCSpatialReferenceUtils::getCoordinateSystemInfo(RCString(userSetOrigCS.c_str()), isGeographics, isLinearUnit, unitScale);
        if (rcOK == result && isLinearUnit)
            return unitScale;
    }

    unitScale = RCSpatialReferenceUtils::getGeoUnitScale(linearUnitsGeoKey);

    return unitScale;
}
#endif
