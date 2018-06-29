#pragma once

#include <cstdint>
#include <vector>
#include <map>
#include <queue>
#include <memory>

#include <common/RCBox.h>
#include <common/RCPlane.h>
#include <common/RCCode.h>
#include <common/RCUserDataSource.h>
#include <common/RCMemoryHelper.h>
#include <common/RCOrientedBoundingBox.h>
#include <common/RCUUID.h>

#include <utility/RCTimer.h>
#include <utility/RCIThreadWorker.h>
#include <utility/RCMutex.h>

#include <config/RCDebugConfig.h>

#include "RCFileReaderBase.h"
#include "OctreeImportPoint.h"
#include "OctreeIntermediateNode.h"
#include "IPointCloudFilter.h"

namespace ambergris
{
	namespace Scene
	{
		struct OctreeFileHeader;
		struct OctreeFileChunk;
	}

    namespace RealityStudio
    {
        namespace SDK
        {
            namespace Common 
            {
                class SyncThread;
            }
        }
    }
}

namespace ambergris { namespace RealityComputing {
    namespace Asset { struct RCAssetInfo; }
    namespace Common { struct RCScanFormat;  }
    namespace Formats { class SphericalModel; }
}}

/*
NEW FILE FORMAT Explanation

SVO Interior Node ( 32 bits total )
[0.....18]            Index to first child, if this is a leaf points to leaf array
[19....26]            Child Information
[27....27]            Is Leaf
[28....31]            Reserved


SVO Leaf Node ( 128 bits total )
//ULong #1
[0.....53]            XYZ Coordinates( 18 bits per axis )
[54....61]            Classification LIDAR
[62....63]            K2 Native Bits Reserved( e.g. Point/Voxel is removed )

//ULong #2
[64....95]            RGBA    
[96...109]            Normal ( QSplat Paper )        --> http://graphics.stanford.edu/papers/qsplat/
[110..113]            Classification(PRIMITIVES)    --> Noise, Plane, Cylinder, Sphere //1, 2, 3, 4
[114..127]            Reserved    

std::uint64_t                m_data[2];
*/ 


//#define                    MAX_POINTS_PER_OCTREE_LEAFNODE            (  512 * 1024 - 1 )    
//#define                    MAX_INTERMEDIATE_TREE_DEPTH                ( 6 )
//#define                    MIN_SVO_LEAF_POINTS                        1
//#define                    MAJOR_VERSION                            3
//#define                    MINOR_VERSION                            0
//
//#define                    MAX_SIZE_TERRESTRIAL_UMBRELLA_LEAFNODE    1.0        //we can use 12 bytes per point    
//#define                    MAX_SIZE_LIDAR_UMBRELLA_LEAFNODE        262.0    //lidar/aerial data

//future expansion of file format
#define                    MAX_FILE_CHUNK_ENTRIES                    4096

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4251)
#endif

namespace ambergris { namespace RealityComputing { namespace Import {

        /*
        PointCloudIndexer: Base class of point cloud indexing operations
        */
        template<typename PointType>
        class PointCloudIndexer : public PointCloudIndexerInterface
        {
        public:
            //////////////////////////////////////////////////////////////////////////
            // \brief: Default Constructor, if argument is not NULL, this class
            //           will update the current progress to the converter
            //////////////////////////////////////////////////////////////////////////

            //////////////////////////////////////////////////////////////////////////
            PointCloudIndexer();
            // \brief: Destructor
            //////////////////////////////////////////////////////////////////////////
            virtual                                             ~PointCloudIndexer();

            //////////////////////////////////////////////////////////////////////////
            // \brief: set / get the oriented bounding box of this scan
            //////////////////////////////////////////////////////////////////////////
            virtual void       setOrientedBoundingBox(const ambergris::RealityComputing::Common::RCOrientedBoundingBox& obb)  override;
            virtual const ambergris::RealityComputing::Common::RCOrientedBoundingBox&       getOrientedBoundingBox() const override;


            //////////////////////////////////////////////////////////////////////////
            // \brief: Returns the total bounding box of this scan
            //////////////////////////////////////////////////////////////////////////
            virtual const ambergris::RealityComputing::Common::RCBox&                       getBoundingBox() const override;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Returns the translation of this scan
            //////////////////////////////////////////////////////////////////////////
            virtual const ambergris::RealityComputing::Common::RCVector3d&                   getTranslation() const override;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Returns the rotation of this scan
            //////////////////////////////////////////////////////////////////////////
            virtual const ambergris::RealityComputing::Common::RCVector3d&                   getRotation() const override;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Returns the scale of this scan
            //////////////////////////////////////////////////////////////////////////
            virtual const ambergris::RealityComputing::Common::RCVector3d&                   getScale() const;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Returns the scale of this scan
            //////////////////////////////////////////////////////////////////////////
            ambergris::RealityComputing::Common::RCVector3d                                   getScannerOrigin() const;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Adds a point to temp file, IMPORTANT if this indexer has an
            //         import filter point will be filtered/transformed according
            //           to the filter
            //////////////////////////////////////////////////////////////////////////
            AddPointResult                                      addPointToTempFile( const PointType& point);

            //////////////////////////////////////////////////////////////////////////
            // \brief: Sets the number of threads for point cloud creation
            //////////////////////////////////////////////////////////////////////////
            void                                                setNumberOfThreads( int val );

            //////////////////////////////////////////////////////////////////////////
            // \brief: Returns the number of threads for svo creation
            //////////////////////////////////////////////////////////////////////////
            int                                                 getNumberOfThreads() const;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Returns the output folder 
            //////////////////////////////////////////////////////////////////////////
            virtual std::wstring                                getOutputFolder() const override;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Returns the temp. folder
            //////////////////////////////////////////////////////////////////////////
            virtual std::wstring                                getTempFolder() const override;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Returns the node name( file name without path & extension );
            //////////////////////////////////////////////////////////////////////////
            virtual const std::wstring&                         getNodeName() const override;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Returns if this a terrestrial scan
            //////////////////////////////////////////////////////////////////////////
            bool                                                getTerrestrial() const;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Returns if this a terrestrial scan
            //////////////////////////////////////////////////////////////////////////
            virtual void                                        setTerrestrial( bool val ) override;


            //////////////////////////////////////////////////////////////////////////
            // \brief: Sets the output filename
            //////////////////////////////////////////////////////////////////////////
            virtual void                                        setOutputFolder( const std::wstring& val ) override;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Sets the input scans
            //////////////////////////////////////////////////////////////////////////
            virtual void                                        setInput(const std::wstring& name, const std::wstring& filepath) override;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Sets the up direction of the scan
            //////////////////////////////////////////////////////////////////////////
            virtual void                                        setUpDirection( UP_DIRECTION val) override { m_upDirection = val; }
            

            //////////////////////////////////////////////////////////////////////////
            // \brief: Sets the temp. folder
            //////////////////////////////////////////////////////////////////////////
            virtual void                                                setTempFolder( const std::wstring& val ) override;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Returns the installed point cloud filter, can be NULL
            //////////////////////////////////////////////////////////////////////////
            virtual IPointCloudFilter*                                  getPointCloudFilter() override;

            void                                                resetToDefault();

            //////////////////////////////////////////////////////////////////////////
            // \brief: Sets the point cloud filter
            //////////////////////////////////////////////////////////////////////////
            virtual void setPointCloudFilter( std::unique_ptr<IPointCloudFilter> filter) override;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Set the minimum amount of points a svo-node should contain, before
            //           converting it to a leaf node
            //////////////////////////////////////////////////////////////////////////
            void                                                setMinimumAmountOfPointsSVOLeaf( int val );

            ///////////////////////////////////////////////////////////////////////////
            // \brief: Set the maximum amount of points before treating it as 
            //           octree leaf node
            //////////////////////////////////////////////////////////////////////////
            void                                                setMaximumAmountOfPointsOctreeLeaf( int val );

            //////////////////////////////////////////////////////////////////////////
            // \brief: Returns the minimum amount of points contained inside
            //           an svo leaf node
            //////////////////////////////////////////////////////////////////////////
            int                                                 getMinimumAmountOfPointsSVOLeaf() const;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Sets the maximum size of the octree leaf bounds in millimeters
            //////////////////////////////////////////////////////////////////////////
            void                                                setMaxOctreeLeafBounds( double val );

            //////////////////////////////////////////////////////////////////////////
            // \brief: Sets the indexing process to be canceled
            //////////////////////////////////////////////////////////////////////////
            virtual void                                        setCancelled( bool val ) override;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Returns if the indexing is canceled
            //////////////////////////////////////////////////////////////////////////
            virtual bool                                        getCancelled( ) const override;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Removes all the temp files in the temp folder
            //////////////////////////////////////////////////////////////////////////
            virtual void                                        removeTempFiles() override;

            //////////////////////////////////////////////////////////////////////////
            //\brief: Returns if we should normalize intensity values in the range[0..255]
            //////////////////////////////////////////////////////////////////////////
            bool                                                getNormalizeIntensity() const;

            //////////////////////////////////////////////////////////////////////////
            //\brief: Sets if we should normalize intensity values 
            //////////////////////////////////////////////////////////////////////////
            virtual void                                        setNormalizeIntensity( bool val ) override;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Returns the normalized[0..255] intensity value
            //////////////////////////////////////////////////////////////////////////
            std::uint8_t                                        getNormalizedIntensityValue( float minVal, float maxVal, float val );

            virtual void                                        setFilterType(NOISE_FILTER filterType) override;
            virtual NOISE_FILTER                                getFilterType() override;

            virtual void                                        setDensity(int density) override;

            PointType                                           updateWithBasicOctreeImportPoint( const PointType& inputPoint, const BasicOctreeImportPoint& update );

            void                                                addPointToUmbrellaTreeNode(OctreeIntermediateNode<PointType>* childNode, const PointType& point);

            //////////////////////////////////////////////////////////////////////////
            // \brief: set whether has RGB information
            //////////////////////////////////////////////////////////////////////////
            void                                                setHasRGB( bool hasRGB );
            virtual bool                                        hasRGB() const override;
            //////////////////////////////////////////////////////////////////////////
            // \brief: set whether has normals information
            //////////////////////////////////////////////////////////////////////////
            virtual void                                        setHasNormals( bool hasNormals ) override;
            virtual bool                                        hasNormals() const override;

            //////////////////////////////////////////////////////////////////////////
            // \brief: set whether has intensity information
            //////////////////////////////////////////////////////////////////////////
            void                                                setHasIntensity( bool hasIntensity );
            virtual bool                                        hasIntensity() const override;


            //////////////////////////////////////////////////////////////////////////
            // \brief: set whether has time stamp information
            //////////////////////////////////////////////////////////////////////////
            void                                                setHasTimeStamp( bool hasTimeStamp );
            virtual bool                                        hasTimeStamp() const override;

            //////////////////////////////////////////////////////////////////////////
            // \brief: set whether has classification information
            //////////////////////////////////////////////////////////////////////////
            void                                                setHasClassification( bool hasTimeStamp );
            virtual bool                                        hasClassification() const override;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Linearizes the umbrella nodes, also re-arranged the 
            //           Intermediate nodes, so that they form 'islands' of files
            //           that are in close proximity with each other, returns the amount of nodes found
            //////////////////////////////////////////////////////////////////////////
            std::int64_t                                        linearizeUmbrellaOctree();

			std::uint64_t										getTotalAmountOfPoints() const { return m_totalAmountOfPoints; }

			RealityComputing::Common::RCBox						getSVOBounds() const;
			RealityComputing::Common::RCBox						getScanBounds() const;

			virtual void                                        convertLeafNodeToLeafSaveData(std::vector<OctreeLeafSaveData>& leafSaveList);

            //////////////////////////////////////////////////////////////////////////
            // \brief: set whether calculate normals from unstructured data
            //////////////////////////////////////////////////////////////////////////
            virtual void                                        setCalculateNormalsFromUnstructuredData(bool calculateNormals) override;

            //////////////////////////////////////////////////////////////////////////
            // \brief: get whether calculate normals from unstructured data
            //////////////////////////////////////////////////////////////////////////
            virtual bool                                        getCalculateNormalsFromUnstructuredData() override;

            //////////////////////////////////////////////////////////////////////////
            // \brief: set intensity filter
            //////////////////////////////////////////////////////////////////////////
            virtual void                                        setIntensityFilter(const IntensityFilter& intensityFilter) override;

            virtual void                                        setEnableRangeClipping(bool enableRangeClipping) override;

            //////////////////////////////////////////////////////////////////////////
            // \brief: set range filter
            //////////////////////////////////////////////////////////////////////////
            virtual void                                        setRangeFilter(const RangeFilter& rangeFilter) override;

            //////////////////////////////////////////////////////////////////////////
            // \brief: get the scan number as one file can have multiple scans
            //////////////////////////////////////////////////////////////////////////
            virtual int                                         getNumScans() const override;

            //////////////////////////////////////////////////////////////////////////
            // \brief: get current scan id being indexed, scan index start with 0
            //////////////////////////////////////////////////////////////////////////
            int                                                 getCurrentScanId() const;

            //////////////////////////////////////////////////////////////////////////
            // \brief: get the scan number as one file can have multiple scans, scan index start with 0
            //////////////////////////////////////////////////////////////////////////
            virtual void                                        setCurrentScanId( int scanId ) override;    

            //////////////////////////////////////////////////////////////////////////
            // \brief: update registration information
            //////////////////////////////////////////////////////////////////////////
            virtual void                                        updateRegistrationInfo( ambergris::RealityComputing::Common::RCVector3d rotation, ambergris::RealityComputing::Common::RCVector3d translation ) override;

            virtual void                                        getRegistrationInfo(ambergris::RealityComputing::Common::RCVector3d& rotation, ambergris::RealityComputing::Common::RCVector3d& translation) const override;

            //////////////////////////////////////////////////////////////////////////
            // \brief: get error code
            //////////////////////////////////////////////////////////////////////////
            virtual ambergris::RealityComputing::Common::RCCode  getErrorCode() override;

            virtual bool isEmptyScan() const override;
            virtual bool isBadBounds() const override;

        protected:

            //////////////////////////////////////////////////////////////////////////
            // \brief: total bounding box of this scan
            //////////////////////////////////////////////////////////////////////////
            ambergris::RealityComputing::Common::RCBox           m_scanBounds;

            //////////////////////////////////////////////////////////////////////////
            // \brief: oriented bounding box of this scan
            //////////////////////////////////////////////////////////////////////////
            ambergris::RealityComputing::Common::RCOrientedBoundingBox           m_scanOrientedBoundingBox;

            //////////////////////////////////////////////////////////////////////////
            //// \brief: total amounts of points
            //////////////////////////////////////////////////////////////////////////
            std::uint64_t                                       m_totalAmountOfPoints;

            //////////////////////////////////////////////////////////////////////////
            // \brief: this will make up the transform of the scan
            //////////////////////////////////////////////////////////////////////////
            ambergris::RealityComputing::Common::RCVector3d      m_translation,
                m_rotation,
                m_scale;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Original position of scanner( untransformed )
            //////////////////////////////////////////////////////////////////////////
            ambergris::RealityComputing::Common::RCVector3d      m_scannerOrigin;

            //////////////////////////////////////////////////////////////////////////
            // \brief: the input file (scan file )
            //////////////////////////////////////////////////////////////////////////
            std::wstring                                    m_inputFilePath;

            //////////////////////////////////////////////////////////////////////////
            // \brief: up direction of the  input file
            //////////////////////////////////////////////////////////////////////////
            UP_DIRECTION m_upDirection;

            //////////////////////////////////////////////////////////////////////////
            // \brief: the temp folder 
            //////////////////////////////////////////////////////////////////////////
            std::wstring                                        m_tempFolder;

            //////////////////////////////////////////////////////////////////////////
            // \brief: the output folder
            //////////////////////////////////////////////////////////////////////////
            std::wstring                                        m_outputFolder;

            //////////////////////////////////////////////////////////////////////////
            // \brief:
            //////////////////////////////////////////////////////////////////////////
            std::wstring                                        m_nodeName;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Number of threads used to create svo
            //////////////////////////////////////////////////////////////////////////
            int                                                 m_numThreads;

            //////////////////////////////////////////////////////////////////////////
            // \brief: # of points before we treated this as (intermediate) leaf node
            //         default is (MAX_POINTS_PER_OCTREE_LEAFNODE)
            //////////////////////////////////////////////////////////////////////////
            int                                                 m_maximiumAmountOfPointsPerLeaf;

            //////////////////////////////////////////////////////////////////////////
            // \brief: # of points before we treated this as SVO leaf node
            //         default is ( MIN_SVO_LEAF_POINTS ), meaning we recurse creating 
            //         svo tree until there is only ( MIN_SVO_LEAF_POINTS )  point in the bounding box
            //////////////////////////////////////////////////////////////////////////
            int                                                 m_minimumAmountOfPointsPerSVOLeaf; 

            //////////////////////////////////////////////////////////////////////////
            // \brief: # of points before we treated this as (intermediate) leaf node
            //        default is (MAX_NODE_BOUNDS_MILIMETER)
            //////////////////////////////////////////////////////////////////////////
            double                                              m_maxOctreeLeafBounds;

            //////////////////////////////////////////////////////////////////////////
            // \brief: estimated amount of points for progress dialog 
            //////////////////////////////////////////////////////////////////////////
            std::int64_t                                        m_estimatedAmountOfPoints;

            //////////////////////////////////////////////////////////////////////////
            // \brief: (Optional) Point cloud filter
            //////////////////////////////////////////////////////////////////////////
            std::unique_ptr<IPointCloudFilter>                  m_pointCloudFilterPtr;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Total amount of temp files written
            //////////////////////////////////////////////////////////////////////////
            int                                                 m_amountOfTempFiles;

            //////////////////////////////////////////////////////////////////////////
            // \brief: ZFS/Faro/Riegl etc.
            //////////////////////////////////////////////////////////////////////////
            bool                                                m_isTerrestrial;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Current amount of point in temp file
            //////////////////////////////////////////////////////////////////////////
            int                                                 m_amountOfPointInTempFile;

            //////////////////////////////////////////////////////////////////////////
            // \brief: The indexing is canceled from outside
            //////////////////////////////////////////////////////////////////////////
            bool                                                m_isCancelled;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Temp filenames, basically the input file converted into
            //           binary file data
            //////////////////////////////////////////////////////////////////////////
            std::vector<std::wstring>                           m_tempFileNames;

            //////////////////////////////////////////////////////////////////////////
            // \brief: temporary points
            //////////////////////////////////////////////////////////////////////////
            std::vector<PointType>                                      m_tempPoints;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Temp points are converted and stored in the intermediate leafnode
            //           in order to speed up the process
            //////////////////////////////////////////////////////////////////////////
            std::vector< std::vector<OctreeIntermediateNode<PointType>*> >   m_intermediateLeafNodeList;

            //////////////////////////////////////////////////////////////////////////
            // \brief: Parent node for intermediate points
            //////////////////////////////////////////////////////////////////////////
            OctreeIntermediateNode<PointType>*                             m_rootNodePtr;

            //////////////////////////////////////////////////////////////////////////
            // \brief: amount of split nodes created
            //////////////////////////////////////////////////////////////////////////
            int                                                 m_amountOfSplitNodes;

            virtual bool                                        getScanTempFileNames();
            virtual void                                        cleanupDataSource(Common::IRCStructuredDataSource*& structureDataSource);

            //////////////////////////////////////////////////////////////////////////
            // \brief: Write a new temp file
            //////////////////////////////////////////////////////////////////////////
            bool                                                writeNewTempFile();

            //////////////////////////////////////////////////////////////////////////
            // \brief: Start to convert temp files into native format
            //////////////////////////////////////////////////////////////////////////

            bool                                                generateUmbrellaOctree( std::wstring& errorLog );
            bool                                                generateUmbrellaOctreeFromUnstructuredData();

            //////////////////////////////////////////////////////////////////////////
            // \brief: Generate a SVO from the octree's LeafNodes
            //////////////////////////////////////////////////////////////////////////
            bool                                                convertUmbrellaOctreeLeafNodes( std::wstring& errorLog, double minDistanceBetweenPoints = 0.003 );

            //////////////////////////////////////////////////////////////////////////
            // \brief: Add a point to the umbrella tree, scanCenter should be 
            //           specified if data is unstructured ( lidar/pts etc )
            //////////////////////////////////////////////////////////////////////////
            //bool                                                addPointToUmbrellaTree( const OctreeImportPointIter* point );

            bool                                                addPointToUmbrellaTree( const PointType& point );
            //////////////////////////////////////////////////////////////////////////
            // \brief: Evaluates the intermediate leaf nodes, returns the leaf nodes
            //           that need to be split into new children
            //////////////////////////////////////////////////////////////////////////
            std::vector<OctreeIntermediateNode<PointType>*>     evaluateUmbrellaLeafNodes( std::wstring& errorLog );

            //////////////////////////////////////////////////////////////////////////
            // \brief: Split leaf node if required, return true if successfully split
            //////////////////////////////////////////////////////////////////////////
            virtual bool                                        splitUmbrellaLeafNode( OctreeIntermediateNode<PointType>* parentPtr );

			void updatePointByUpDirection(const ambergris::RealityComputing::Common::RCVector3d& pointIn, ambergris::RealityComputing::Common::RCVector3d& pointOut, UP_DIRECTION upDirection);

            //////////////////////////////////////////////////////////////////////////
            // \brief: Intermediate Node information, will be filled in after
            //           call to 'linearizeUmbrellaOctree()'
            //////////////////////////////////////////////////////////////////////////
            OctreeIntermediateInformation<PointType>            mUmbrellaNodeInformation[MAX_UMBRELLA_NODE_LEVELS];            

            //////////////////////////////////////////////////////////////////////////
            // \brief: basic information of the scan
            //////////////////////////////////////////////////////////////////////////
            bool                                                mHasRGB;
            bool                                                mHasNormals;
            bool                                                mHasIntensity;
            bool                                                mHasTimeStamp;
            bool                                                mHasClassification;

            // splitting stops if the leaf node's size is smaller than this value
            double                                              mLeafSplitStopSize;
			
            float                                               mIntensityMinValue,
                mIntensityMaxValue;
            bool                                                mNormalizeIntensity;

            bool                                                mCalculateNormalsFromUnstructuredData;

            RangeFilter                                         mRangeFilter;
            IntensityFilter                                     mIntensityFilter;
            NOISE_FILTER                                        mFilterType;

            bool                                                mRangeClippingEnabled;

            /// if there are multiple scans, then the relation between scan id and temp files should be saved
            std::map<int, std::vector<std::wstring>>            mScanIdToTempFilesMap;

            std::map<int, ambergris::RealityComputing::Common::RCVector3d>                   mScanIdToScanTranslations;
            int                                                                             mCurrentScanId;
            mutable ambergris::RealityComputing::Common::RCCode                              mErrorCode;
        };

    }}} // ns

#if defined( _MSC_VER )
#pragma warning(pop)
#endif

