#pragma once

#include <string>

#include "RCLasDefs.h"

namespace ambergris { namespace RealityComputing { namespace Common {
    class RCString;
}}}

namespace ambergris { namespace RealityComputing { namespace Import {

namespace LasUtils
{
    void interpretKeyEntry(const RCLASvariable_header_key_entry& variable_header_key_entry,
        int& epsgCode, LinearUnitsGeoKey& linearUnit, GeogAngularUnitsGeoKey& angularUnit);

	ambergris::RealityComputing::Common::RCString getCoordinateSystemFromFile(const ambergris::RealityComputing::Common::RCString& fileName);


#ifdef _WIN32
    double checkUnitScale(int epsgCode, int linearUnitsGeoKey, const std::wstring& userSetOrigCS);
#endif
}

}}}
