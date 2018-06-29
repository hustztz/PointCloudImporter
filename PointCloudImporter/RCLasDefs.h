#pragma once

namespace ambergris { namespace RealityComputing { namespace Import {

class RCLASvariable_header_geo_keys
{
public:
    unsigned short key_directory_version;
    unsigned short key_revision;
    unsigned short minor_revision;
    unsigned short number_of_keys;
};

class RCLASvariable_header_key_entry
{
public:
    unsigned short key_id;
    unsigned short tiff_tag_location;
    unsigned short count;
    unsigned short value_offset;
};

enum LinearUnitsGeoKey
{
    Linear_Meter = 9001,
    Linear_Foot = 9002,
    Linear_Foot_US_Survey = 9003,
    Linear_Foot_Modified_American = 9004,
    Linear_Foot_Clarke = 9005,
    Linear_Foot_Indian = 9006,
    Linear_Link = 9007,
    Linear_Link_Benoit = 9008,
    Linear_Link_Sears = 9009,
    Linear_Chain_Benoit = 9010,
    Linear_Chain_Sears = 9011,
    Linear_Yard_Sears = 9012,
    Linear_Yard_Indian = 9013,
    Linear_Fathom = 9014,
    Linear_Mile_International_Nautical = 9015,
    Linear_Unknown = 9000
};

enum GeogAngularUnitsGeoKey
{
    Angular_Radian = 9101,
    Angular_Degree = 9102,
    Angular_Arc_Minute = 9103,
    Angular_Arc_Second = 9104,
    Angular_Grad = 9105,
    Angular_Gon = 9106,
    Angular_DMS = 9107,
    Angular_DMS_Hemisphere = 9108,
    Angular_Unknown = 9100
};

}}} // ns
