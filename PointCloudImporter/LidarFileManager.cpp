#ifdef _WIN32

#include <sstream>
#include <common/RCString.h>
#include "LidarFileManager.h"
#include "LasUtils.h"
#include <utility/RCLog.h>

#include <laszip_api.h>

using namespace ambergris::RealityComputing::Common;
using namespace ambergris::RealityComputing::Import;
using namespace ambergris::RealityComputing::Utility;

LidarFileManager::LidarFileManager()
    :mLaszipReader(nullptr),
    mLasZipHeader(nullptr),
    mLasPoint(nullptr)
{

}

LidarFileManager::~LidarFileManager()
{
    if (mLaszipReader)
        close();
}

bool LidarFileManager::open(const std::wstring& fileName)
{
    std::stringstream ss;

    if (laszip_create(&mLaszipReader))
    {
        Log::RCLog::getLogFile()->addMessage("DLL ERROR: creating laszip reader\n");
        return false;
    }

    laszip_BOOL is_compressed = 0;
    if (laszip_open_reader(mLaszipReader, RCString(fileName.c_str()).c_str(), &is_compressed))
    {
        ss.str("");
        ss << "DLL ERROR: opening laszip reader for " << fileName.c_str() << std::endl;
        Log::RCLog::getLogFile()->addMessage(ss.str().c_str());
        return false;
    }

    if (laszip_get_header_pointer(mLaszipReader, &mLasZipHeader))
    {
        Log::RCLog::getLogFile()->addMessage("DLL ERROR: getting header pointer from laszip reader\n");
        return false;
    }

    if (laszip_get_point_pointer(mLaszipReader, &mLasPoint))
    {
        Log::RCLog::getLogFile()->addMessage("DLL ERROR: getting point pointer from laszip reader\n");
        return false;
    }

    return true;
}

laszip_header* LidarFileManager::getLasHeader() const
{
    return mLasZipHeader;
}

laszip_POINTER LidarFileManager::getLasReader() const
{
    return mLaszipReader;
}

laszip_point*  LidarFileManager::getLasPoint() const
{
    return mLasPoint;
}

bool LidarFileManager::ReadCoordinateSystemInfo(int& epsgCode, LinearUnitsGeoKey& linearUnit, GeogAngularUnitsGeoKey& angularUnit)
{
    for (int i = 0; i < (int)mLasZipHeader->number_of_variable_length_records; i++)
    {
        auto& variable_header = mLasZipHeader->vlrs[i];
        if (strcmp(variable_header.user_id, "LASF_Projection") == 0 && variable_header.record_id == 34735)
        {
            laszip_geokey_struct geo_keys;
            memcpy(&geo_keys, variable_header.data, sizeof(geo_keys));
            //Laszip uses the same structure for geo_keys and key_entry
            RCLASvariable_header_geo_keys variable_header_geo_keys;
            variable_header_geo_keys.key_directory_version = geo_keys.key_id;
            variable_header_geo_keys.key_revision = geo_keys.tiff_tag_location;
            variable_header_geo_keys.minor_revision = geo_keys.count;
            variable_header_geo_keys.number_of_keys = geo_keys.value_offset;

            for (int j = 0; j < variable_header_geo_keys.number_of_keys; ++j)
            {
                memcpy(&geo_keys, variable_header.data + (j + 1) * sizeof(geo_keys), sizeof(geo_keys));
                RCLASvariable_header_key_entry variable_header_key_entry;
                variable_header_key_entry.key_id = geo_keys.key_id;
                variable_header_key_entry.tiff_tag_location = geo_keys.tiff_tag_location;
                variable_header_key_entry.count = geo_keys.count;
                variable_header_key_entry.value_offset = geo_keys.value_offset;

                LasUtils::interpretKeyEntry(variable_header_key_entry, epsgCode, linearUnit, angularUnit);
            }
        }
    }

    return true;
}

void LidarFileManager::close()
{
    if (laszip_close_reader(mLaszipReader))
    {
        Log::RCLog::getLogFile()->addMessage("DLL ERROR: closing laszip reader\n");
        return;
    }

    // destroy the reader
    if (laszip_destroy(mLaszipReader))
    {
        Log::RCLog::getLogFile()->addMessage("DLL ERROR: destroying laszip reader\n");
        return;
    }

    mLaszipReader = nullptr;
    mLasZipHeader = nullptr;
    mLasPoint = nullptr;
}

#endif