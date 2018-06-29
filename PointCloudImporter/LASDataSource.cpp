#include <sstream>
#include <string>
#include <map>
#include <algorithm>

#include <common/RCString.h>
#include <common/RCVector.h>

#include "LASDataSource.h"

#ifdef _WIN32
#include "LidarFileManager.h"
#endif

//#include <import/RCFileReaderFactory.h>
#include "PointCloudIndexer.h"
#include "LasUtils.h"
#include "DataSourceIndexer.h"
//#include <import/PointCloudCSFilter.h>

#include <utility/RCFilesystem.h>
#include <utility/RCLog.h>

#include <laszip_api.h>

using namespace ambergris::RealityComputing::Common;
using namespace ambergris::RealityComputing::Import;
using namespace ambergris::RealityComputing::Utility;


struct LASDataSource::Impl
{
    std::wstring mFilename;

    RCVector3d mPosition;
    RCVector4ub mColor;
    float mIntensity;

    RCScanMetaData mMetaData;

    size_t mCurrentPtCnt, mTotalPntCnt;
    int                    mEPSGCode;
    double                 mUnitScale;
    LinearUnitsGeoKey      mLinearUnit;
    GeogAngularUnitsGeoKey mAngularUnit;

#ifdef _WIN32
    LidarFileManager mLidarFileManager;
    laszip_POINTER mLaszipReader;
    laszip_header* mLasZipHeader;
    laszip_point* mLasPoint;
#endif

    std::wstring mUserSetOriginalCS;
    bool          mIsOpen;

    bool open();
    void updateUnitScale(const std::wstring& userSetOriginalCS);
    void setScanFormat();
};

bool LASDataSource::Impl::open()
{
#ifdef _WIN32
    if (mIsOpen)
        return true;

    if (!Filesystem::exists(mFilename))
        return false;

    bool success = mLidarFileManager.open(mFilename);
    if (!success)
        return false;

    mIsOpen = true;
    mLaszipReader = mLidarFileManager.getLasReader();
    mLasZipHeader = mLidarFileManager.getLasHeader();
    mLasPoint = mLidarFileManager.getLasPoint();

    mTotalPntCnt = static_cast<size_t>((mLasZipHeader->number_of_point_records ? 
                                        mLasZipHeader->number_of_point_records : mLasZipHeader->extended_number_of_point_records));

    mLidarFileManager.ReadCoordinateSystemInfo(mEPSGCode, mLinearUnit, mAngularUnit);

    setScanFormat();
    return true;
#else
    return false;
#endif
}

void LASDataSource::Impl::setScanFormat()
{
#ifdef _WIN32
    bool have_rgb = false;
    bool have_gps_time = false;
    bool have_point14 = false;
    bool have_wavepacket = false;
    bool have_nir = false;

    // switch over the point types we know
    switch (mLasZipHeader->point_data_format)
    {
    case 0:
        break;
    case 1:
        have_gps_time = true;
        break;
    case 2:
        have_rgb = true;
        break;
    case 3:
        have_gps_time = true;
        have_rgb = true;
        break;
    case 4:
        have_gps_time = true;
        have_wavepacket = true;
        break;
    case 5:
        have_gps_time = true;
        have_rgb = true;
        have_wavepacket = true;
        break;
    case 6:
        have_point14 = true;
        break;
    case 7:
        have_point14 = true;
        have_rgb = true;
        break;
    case 8:
        have_point14 = true;
        have_rgb = true;
        have_nir = true;
        break;
    case 9:
        have_point14 = true;
        have_wavepacket = true;
        break;
    case 10:
        have_point14 = true;
        have_rgb = true;
        have_nir = true;
        have_wavepacket = true;
        break;
    default:
        {
            std::stringstream ss;
            ss << "Unknown Point type "<< mLasZipHeader->point_data_format <<std::endl;
            Log::RCLog::getLogFile()->addMessage(ss.str().c_str());
        }
    }

    auto& format = mMetaData.format;
    format.hasRGB = have_rgb;
    format.hasTimestamp = have_gps_time;

#endif
}

void LASDataSource::Impl::updateUnitScale(const std::wstring& userSetOriginalCS)
{
#ifdef _WIN32
    mUserSetOriginalCS = userSetOriginalCS;
    mUnitScale = LasUtils::checkUnitScale(mEPSGCode, mLinearUnit, userSetOriginalCS);
#endif
}

LASDataSource::LASDataSource(const std::wstring& filename) : mImpl(nullptr)
{
    mImpl = new Impl;
    mImpl->mFilename = filename;

    mImpl->mIntensity = 0;

    mImpl->mIsOpen = false;

    mImpl->mMetaData.provider = RCScanProvider::PROVIDER_LAS_DATA;

    mImpl->mCurrentPtCnt = 0;
    mImpl->mTotalPntCnt = 0;

#ifdef _WIN32
    mImpl->mLasZipHeader = nullptr;
    mImpl->mLaszipReader = nullptr;
    mImpl->mLasPoint = nullptr;
#endif

    mImpl->mEPSGCode = -1;
    mImpl->mUnitScale = 1.0;
    mImpl->mLinearUnit = Linear_Meter;
    mImpl->mAngularUnit = Angular_Radian;
}

LASDataSource::LASDataSource(LASDataSource&& that)
{
    std::swap(mImpl, that.mImpl);
}

LASDataSource::~LASDataSource()
{
    delete mImpl;
    mImpl = nullptr;
}

size_t LASDataSource::getPointCt() const
{
    return mImpl->mTotalPntCnt;
}

double LASDataSource::getStatusProgress() const
{
    return static_cast<double>(mImpl->mCurrentPtCnt) / mImpl->mTotalPntCnt;
}

void LASDataSource::getStatusMessage(std::wstring& statusMessageOut) const
{
    std::wstringstream ss;
    if (mImpl->mCurrentPtCnt == 0)
    {
        ss << "Initializing \"" << mImpl->mFilename << "\"";
    }
    else
    {
        ss << "Reading \"" << mImpl->mFilename << "\"";
    }
    ss << std::ends;
    statusMessageOut = ss.str();
}

bool LASDataSource::open()
{
    if (!mImpl)
    {
        setLastError(IRCUserDataSource::ErrorCode::UnknownError);
        return false;
    }

    if (mImpl->open())
    {
        setLastError(IRCUserDataSource::ErrorCode::OK);
        return true;
    }
    else
    {
        setLastError(IRCUserDataSource::ErrorCode::UnknownError);
        return false;
    }
}

void LASDataSource::updateUnitScale(const std::wstring& userSetOriginalCS)
{
    mImpl->updateUnitScale(userSetOriginalCS);
}

bool LASDataSource::close()
{
#ifdef _WIN32
    if (mImpl)
        mImpl->mLidarFileManager.close();
#endif
    setLastError(IRCUserDataSource::ErrorCode::OK);
    return true;
}

bool LASDataSource::advancePoint()
{
    setLastError(IRCUserDataSource::ErrorCode::OK);
#ifdef _WIN32
    if (mImpl->mCurrentPtCnt >= mImpl->mTotalPntCnt)
        return false;

    if (laszip_read_point(mImpl->mLaszipReader))
    {
        std::stringstream ss;
        ss << "DLL ERROR: reading point " << mImpl->mCurrentPtCnt << std::endl;
        Log::RCLog::getLogFile()->addMessage( ss.str().c_str() );
        return false;
    }

    mImpl->mCurrentPtCnt++;
    if (mImpl->mLasPoint->classification != 0)
        mImpl->mMetaData.format.hasClassification = true;
    if (mImpl->mLasPoint->intensity!=0)
        mImpl->mMetaData.format.hasIntensity = true;

    return true;
#else 
    return false;
#endif
}

void LASDataSource::getPosition(ambergris::RealityComputing::Common::RCVector3d& pt) const
{
#ifdef _WIN32
    if (!mImpl->mLasPoint || !mImpl->mLasZipHeader)
        return;
    
    pt.x = mImpl->mLasZipHeader->x_offset + mImpl->mLasZipHeader->x_scale_factor * mImpl->mLasPoint->X;
    pt.y = mImpl->mLasZipHeader->y_offset + mImpl->mLasZipHeader->y_scale_factor * mImpl->mLasPoint->Y;
    pt.z = mImpl->mLasZipHeader->z_offset + mImpl->mLasZipHeader->z_scale_factor * mImpl->mLasPoint->Z;

    pt = pt * mImpl->mUnitScale;
#endif
}

void LASDataSource::getColor(ambergris::RealityComputing::Common::RCVector4ub& color) const
{
#ifdef _WIN32
    if (!mImpl->mLasPoint)
        return;

    int r, g, b;

    if (mImpl->mMetaData.format.hasRGB)
    {
        bool isRGB16Bit = false;

        r = mImpl->mLasPoint->rgb[0];
        g = mImpl->mLasPoint->rgb[1];
        b = mImpl->mLasPoint->rgb[2];

        if (!isRGB16Bit && (r > 255 || g > 255 || b > 255))
        {
            isRGB16Bit = true;
        }

        if (isRGB16Bit)
        {
            r = (r >> 8) & 0xff;
            g = (g >> 8) & 0xff;
            b = (b >> 8) & 0xff;
        }
    }
    else
    {
        // Set r=g=b=intensity just to have a sane value.
        float intensity = 0.0;
        getIntensity(intensity);
        r = g = b = static_cast<int>(255.0 * (static_cast<float>(intensity) / std::numeric_limits<unsigned short>::max()));
    }

    color[0] = static_cast<std::uint8_t>(r);
    color[1] = static_cast<std::uint8_t>(g);
    color[2] = static_cast<std::uint8_t>(b);
#endif
}

void LASDataSource::getIntensity(float& intensity) const
{
#ifdef _WIN32
    if (!mImpl->mLasPoint)
        return;

    intensity = mImpl->mLasPoint->intensity;
#endif
}

#ifdef _WIN32
void LASDataSource::getClassification(uint8_t& classification) const
{
    if (!mImpl->mLasPoint)
        return;

    classification = mImpl->mLasPoint->classification;
}

void LASDataSource::getTimestamp(double& timeStamp) const
{
    if (!mImpl->mLasPoint)
        return;

    timeStamp = mImpl->mLasPoint->gps_time;
}
#endif

const RCScanMetaData& LASDataSource::getMetaData() const 
{
    return mImpl->mMetaData;
}

int LASDataSource::getEPGSCode() const
{
    return mImpl->mEPSGCode;
}

LinearUnitsGeoKey LASDataSource::getLinearUnit() const
{
    return mImpl->mLinearUnit;
}
