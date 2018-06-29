#pragma once

#include <fstream>
#include <cstdint>
#include <vector>
#include <cmath>
#include <cfloat>

#include <common/RCVector.h>

#ifndef ALIGNAS
#ifndef _MSC_VER
#define ALIGNAS(x) alignas(x)
#else
#define ALIGNAS(x)
#endif
#endif

namespace ambergris { namespace RealityComputing { namespace Import {

//////////////////////////////////////////////////////////////////////////
// \brief: Single Point of a Scan File During Import
//////////////////////////////////////////////////////////////////////////
struct ALIGNAS(8) BasicOctreeImportPoint
{
    BasicOctreeImportPoint() : m_normal(0), m_segmentId(0), m_intensity(0), m_misc { 0 } {}

    BasicOctreeImportPoint(const BasicOctreeImportPoint&) = default;
    BasicOctreeImportPoint& operator=(const BasicOctreeImportPoint&) = default;

    ~BasicOctreeImportPoint(){
        m_misc[0] = m_misc[1] = m_misc[2] = m_misc[3] = 0;
    }

    Common::RCVector3d                                      m_pos;                       //xyz coordinate
    std::uint16_t                                           m_normal;                    //normal id( see alMath.cpp )
    std::uint16_t                                           m_segmentId;                 //segment id
    Common::RCVector4ub                                     m_rgba;                      //color & reflectance
    float                                                   m_intensity;                 //reflectance
    std::uint8_t                                            m_misc[4];                   //LIDAR classification, tree's buildings etc,  detected PRIMITIVES
                                                                                         //also used for angular offset for range image points
};
static_assert(alignof(BasicOctreeImportPoint) == 8, "alignof(BasicOctreeImportPoint) is incorrect");
static_assert(sizeof(BasicOctreeImportPoint) == 40, "sizeof(BasicOctreeImportPoint) is incorrect");

struct ALIGNAS(8) ExtendedOctreeImportPoint : public BasicOctreeImportPoint
{
    ExtendedOctreeImportPoint() : m_reserved(0) { }
    double                                                  m_reserved;                     //reserved, can be used for gps time or other information
};
static_assert(alignof(ExtendedOctreeImportPoint) == 8, "alignof(ExtendedOctreeImportPoint) is incorrect");
static_assert(sizeof(ExtendedOctreeImportPoint) == 48, "sizeof(BasicOctreeImportPoint) is incorrect");

}}}