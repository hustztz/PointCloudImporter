#pragma once

#ifdef _WIN32

#include <string>

#include <laszip_api.h>
#include "RCLasDefs.h"

namespace ambergris {
    namespace RealityComputing {
        namespace Import {

            class LidarFileManager
            {
            public:
                LidarFileManager();
                ~LidarFileManager();
                bool open(const std::wstring& fileName);
                void close();

                laszip_header* getLasHeader() const;
                laszip_POINTER getLasReader() const;
                laszip_point*  getLasPoint() const;

                bool ReadCoordinateSystemInfo(int& epsgCode, LinearUnitsGeoKey& linearUnit, GeogAngularUnitsGeoKey& angularUnit);


            private:
                laszip_POINTER mLaszipReader;
                laszip_header* mLasZipHeader;
                laszip_point* mLasPoint;
            };

        }
    }
}

#endif
