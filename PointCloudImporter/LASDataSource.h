#pragma once

#include <vector>

#include <common/RCUserDataSource.h>

#include "RCLasDefs.h"

namespace ambergris { namespace RealityComputing { namespace Import {
    class FileReaderFactory;

    class LASDataSource : public Common::IRCUserDataSource
    {
    public:
        LASDataSource(const std::wstring& filename = L"");
        LASDataSource(LASDataSource&&);
        ~LASDataSource();

        virtual size_t getPointCt() const override;
        virtual double getStatusProgress() const override;
        virtual void getStatusMessage(std::wstring& statusMessageOut) const override;
        virtual bool open() override;
        virtual bool close() override;

        virtual bool advancePoint() override;

        virtual void getPosition(ambergris::RealityComputing::Common::RCVector3d& pt) const override;
        virtual void getColor(ambergris::RealityComputing::Common::RCVector4ub& color) const override;
        virtual void getIntensity(float& intensity) const override;

#ifdef _WIN32
        virtual void getClassification(uint8_t&) const override;
        virtual void getTimestamp(double&) const override;
#endif

        virtual const Common::RCScanMetaData& getMetaData() const override;

        //
        virtual int getEPGSCode() const;
        virtual LinearUnitsGeoKey getLinearUnit() const;
        virtual void updateUnitScale(const std::wstring& userSetOriginalCS);

    private:
        LASDataSource(const LASDataSource&);
        void operator=(const LASDataSource&);

        struct Impl;
        Impl* mImpl;
    };

}}}
