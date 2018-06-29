#pragma once

#include <string>
#include <limits>

namespace ambergris { namespace Infrastructure { namespace SpatialReference {
    class CoordinateReferenceSystemOperation;
    class CoordinateReferenceSystemTransformation;
}}}

namespace ambergris { namespace RealityComputing { namespace Import {

    struct BasicOctreeImportPoint;

    //////////////////////////////////////////////////////////////////////////
    // \brief: Interface filter points during import (e.g. Mentor library )
    //////////////////////////////////////////////////////////////////////////
    class IPointCloudFilter
    {
    public:
        IPointCloudFilter();
        IPointCloudFilter& operator=(const IPointCloudFilter&) = default;
        virtual ~IPointCloudFilter();

        //////////////////////////////////////////////////////////////////////////
        // \brief: Returns a new point based on user defined filter
        //////////////////////////////////////////////////////////////////////////
        virtual bool                                       applyFilter( const BasicOctreeImportPoint& /*inputPoint*/, BasicOctreeImportPoint& /*outPoint*/) const;
        //////////////////////////////////////////////////////////////////////////
        // \brief: Returns the filter name 
        //////////////////////////////////////////////////////////////////////////
        virtual std::wstring                                getFilterName() const = 0;
    };

    //////////////////////////////////////////////////////////////////////////
    // \brief: Default Filter
    //////////////////////////////////////////////////////////////////////////
    class NullFilter : public IPointCloudFilter
    {
        //////////////////////////////////////////////////////////////////////////
        // \brief: Default returns the point unmodified
        //////////////////////////////////////////////////////////////////////////
        bool                                                applyFilter( const BasicOctreeImportPoint& /*inputPoint*/, BasicOctreeImportPoint& /*outPoint*/ ) const; 

        std::wstring                                        getFilterName() const;
    };

    //////////////////////////////////////////////////////////////////////////
    // \brief: filter points not in the range
    //////////////////////////////////////////////////////////////////////////
    class RangeFilter    : public IPointCloudFilter
    {
    public:
        RangeFilter(double minRange=0, double maxRange = (std::numeric_limits<double>::max)());
        RangeFilter& operator=(const RangeFilter&) = default;
        ~RangeFilter();

        bool                                                applyFilter( const BasicOctreeImportPoint& /*inputPoint*/, BasicOctreeImportPoint& /*outPoint*/ ) const;

        std::wstring                                        getFilterName() const;

        void                                                updateRange( double minRange, double maxRange );

        double getMinRange() { return mMinRange; }
        double getMaxRange() { return mMaxRange; }

    private:
        double            mMinRange;
        double            mMaxRange;
        static int        mFilteredPoints;
    };

    //////////////////////////////////////////////////////////////////////////
    // \brief: filter points not fit the intensity range
    //////////////////////////////////////////////////////////////////////////
    class IntensityFilter    : public IPointCloudFilter
    {
    public:
        IntensityFilter(double minRange = 0, double maxRange = (std::numeric_limits<double>::max)());
        IntensityFilter& operator=(const IntensityFilter&) = default;
        ~IntensityFilter();

        bool                                                applyFilter( const BasicOctreeImportPoint& /*inputPoint*/, BasicOctreeImportPoint& /*outPoint*/ ) const;

        std::wstring                                        getFilterName() const;

        void                                                updateRange( double minRange, double maxRange );

    private:
        double              mMinRange;
        double              mMaxRange;
        static int          mFilteredPoints;
    };

}}}
