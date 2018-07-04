#pragma once

#include <memory>

#include <common/RCUserDataSource.h>

#include "PointCloudIndexer.h"

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4251)
#endif

namespace ambergris { namespace RealityComputing { namespace Import {


    // A generic indexer that deals with unstructured point types
    template< typename PointType >
    class DataSourceIndexer : public PointCloudIndexer<PointType>
    {
    public:
        DataSourceIndexer( Common::IRCUserDataSource& dataSource, 
            const std::wstring& sourceName,
            bool isTerrestrial = false );
        ~DataSourceIndexer();

        virtual Common::IRCUserDataSource::ErrorCode parsePoints( std::wstring& errorLog ) override;
        virtual bool                                 convertToNativeFormat( std::wstring& errorLog ) override;
        virtual Common::IRCUserDataSource::ErrorCode postParse( std::wstring& errorLog ) override;
    protected:
        virtual void                            setScanFormat();
    private:
        virtual bool                            readNextPoint( PointType& pointOut );
        struct Impl;
        std::unique_ptr< Impl > mImpl;
    };

}}} // ns

#if defined(_MSC_VER)
#pragma warning(pop)
#endif
