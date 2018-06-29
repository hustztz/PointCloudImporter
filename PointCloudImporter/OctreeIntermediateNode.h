#pragma once

#include <vector>
#include <common/RCBox.h>
#include <common/RCVector.h>

#ifndef ALIGNAS
#ifndef _MSC_VER
#define ALIGNAS(x) alignas(x)
#else
#define ALIGNAS(x)
#endif
#endif

//////////////////////////////////////////////////////////////////////////
//OctreeIntermediateNode: Helper struct for umbrella octree
//////////////////////////////////////////////////////////////////////////
namespace ambergris { namespace RealityComputing { namespace Import {

    template<typename PointType>
    struct OctreeIntermediateNode
    {
         OctreeIntermediateNode( OctreeIntermediateNode* parentPtr);
        OctreeIntermediateNode();
        virtual                                                 ~OctreeIntermediateNode();
        //////////////////////////////////////////////////////////////////////////
        // \brief: Pointer to ancestor
        //////////////////////////////////////////////////////////////////////////
        OctreeIntermediateNode<PointType>*                                 m_parentPtr;

        //////////////////////////////////////////////////////////////////////////
        // \brief: Children List
        //////////////////////////////////////////////////////////////////////////
        OctreeIntermediateNode<PointType>*                                 m_childList[8];


        //////////////////////////////////////////////////////////////////////////
        // \brief: Bounding box of this node
        //////////////////////////////////////////////////////////////////////////
        RealityComputing::Common::RCBox               m_nodeBounds;

        //////////////////////////////////////////////////////////////////////////
        // \brief: SVO Bounding box of this node(bounding box with equal sides)
        //////////////////////////////////////////////////////////////////////////
        RealityComputing::Common::RCBox               m_svoBounds;

        //////////////////////////////////////////////////////////////////////////
        // \brief: The depth of the current node
        //////////////////////////////////////////////////////////////////////////
        int                                                     m_octreeDepth;

        //////////////////////////////////////////////////////////////////////////
        // \brief: Is this node considered a leaf node
        //////////////////////////////////////////////////////////////////////////
        bool                                                    m_isOctreeLeaf;

        //////////////////////////////////////////////////////////////////////////
        // \brief: After splitting/merging leaf nodes, some original leaf nodes
        //           should be removed from PointCloudIndexer::m_intermediateLeafNodeList
        //////////////////////////////////////////////////////////////////////////
        bool                                                    m_removeFromLeafList;

        //////////////////////////////////////////////////////////////////////////
        // \brief: Unique filename of this node where the points are stored
        //////////////////////////////////////////////////////////////////////////
        std::wstring                                            m_fileName;

        //////////////////////////////////////////////////////////////////////////
        // \brief: Unique filename of this node where the points are stored
        //////////////////////////////////////////////////////////////////////////
        std::vector<std::vector<PointType> >                    m_currentPointList;

        //////////////////////////////////////////////////////////////////////////
        // \brief: Amount of non-leaf nodes per octree level
        //////////////////////////////////////////////////////////////////////////
        int                                                     m_numNonLeafNodes[32];

        //////////////////////////////////////////////////////////////////////////
        // \brief: Amount of leaf nodes per octree level
        //////////////////////////////////////////////////////////////////////////
        int                                                     m_numLeafNodes[32];

        //////////////////////////////////////////////////////////////////////////
        // \brief: Amount of points written to .svo file
        //////////////////////////////////////////////////////////////////////////
        int                                                     m_amountOfPointsInFile;
        //////////////////////////////////////////////////////////////////////////
        // \brief: File size of the svo data
        //////////////////////////////////////////////////////////////////////////
        int                                                     m_fileSize;

        //////////////////////////////////////////////////////////////////////////
        // \brief: Max Tree Depth of svo file
        //////////////////////////////////////////////////////////////////////////
        int                                                     m_maxTreeDepth;

        //////////////////////////////////////////////////////////////////////////
        // \brief: Total Amount of points this node contains, or passed through
        //////////////////////////////////////////////////////////////////////////
        std::uint64_t                                           m_totalAmountOfPoints;

        //////////////////////////////////////////////////////////////////////////
        // \brief: RGBA
        //////////////////////////////////////////////////////////////////////////
        RealityComputing::Common::RCVector4ub         m_rgba;

        //////////////////////////////////////////////////////////////////////////
        // \brief: Normal
        //////////////////////////////////////////////////////////////////////////
        std::uint16_t                                           m_normal;

        //////////////////////////////////////////////////////////////////////////
        // \brief: Child Masks etc.
        //////////////////////////////////////////////////////////////////////////
        std::uint16_t                                           m_childInformation;

        //////////////////////////////////////////////////////////////////////////
        // \brief: Misc Data, lidar classification, segmentation
        //////////////////////////////////////////////////////////////////////////
        std::uint8_t                                            m_misc[4];

        //////////////////////////////////////////////////////////////////////////
        // \brief: Index for first child index, into the linearized node list
        //////////////////////////////////////////////////////////////////////////
        int                                                     m_firstChildIndex;

        //////////////////////////////////////////////////////////////////////////
        // \brief: This instance it's index into the leaf array.
        //////////////////////////////////////////////////////////////////////////
        int                                                     m_indexInToLeafArray;
        //////////////////////////////////////////////////////////////////////////
        // \brief: Bit fields( classification, normals  )
        //////////////////////////////////////////////////////////////////////////
        void                                                    writeTemporaryPointsToFile();

        //////////////////////////////////////////////////////////////////////////
        // \brief: Update this node with the current point information
        //////////////////////////////////////////////////////////////////////////
        void                                                    updatePointInformation(const PointType& point );

        //////////////////////////////////////////////////////////////////////////
        // \brief: Reset this node
        //////////////////////////////////////////////////////////////////////////
        void                                                    reset();


    private:
        OctreeIntermediateNode( const OctreeIntermediateNode& ){}

    };


    //////////////////////////////////////////////////////////////////////////
    //OctreeChildInformation: Helper struct for octree child info
    //////////////////////////////////////////////////////////////////////////
    struct OctreeChildInformation
    {
		ambergris::RealityComputing::Common::RCBox           m_childBounds;
        int                                                 m_childId;
    };

    //////////////////////////////////////////////////////////////////////////
    // \brief: This data will be written to disk, these will hold the actual point data
    //////////////////////////////////////////////////////////////////////////
    struct OctreeLeafSaveData
    {
        OctreeLeafSaveData()
            : m_maxDepth(0), m_amountOfPoints(0), m_fileSize(0), m_numNonLeafNodes{0}, m_numLeafNodes{0}, dummy{0}
        { }

        template<typename PointType>
        void setFromIntermediateNode( const OctreeIntermediateNode<PointType>* node )
        {
            m_svoBounds = node->m_svoBounds;
            m_nodeBounds = node->m_nodeBounds;
            m_maxDepth = node->m_maxTreeDepth;
            m_amountOfPoints = node->m_amountOfPointsInFile;
            m_fileSize = node->m_fileSize;

            memcpy( &m_numNonLeafNodes[0], &node->m_numNonLeafNodes[0], sizeof( int ) * 32 );
            memcpy( &m_numLeafNodes[0], &node->m_numLeafNodes[0], sizeof( int ) * 32 );
        }

        RealityComputing::Common::RCBox            m_svoBounds;    // 48
        RealityComputing::Common::RCBox            m_nodeBounds;   // 48 (96)
        int                            m_maxDepth;                           // 4 (100)
        int                            m_amountOfPoints;                     // 4 (104)
        int                            m_fileSize;                           // 4 (108)
        int                            m_numNonLeafNodes[32];                // 128 (236)
        int                            m_numLeafNodes[32];                   // 128 (364)
        char                           dummy[4];                             // 4   (368)
    };
    static_assert(sizeof(OctreeLeafSaveData) == 368, "sizeof(OctreeLeafSaveData) is incorrect"); // 364 + 4 pad

    //////////////////////////////////////////////////////////////////////////
    // \brief: This data will be written to disk, its an SVO
    //           where the leaf nodes contain less then 512k Points
    //////////////////////////////////////////////////////////////////////////
    struct OctreeLeafIntermediateNodeSaveData
    {
        OctreeLeafIntermediateNodeSaveData()
            : m_normalIndex(0), m_childInfo(0), m_misc{0}, m_indexIntoLeafArray(0), m_firstChildIndex(0), dummy{0}
        { }
        
        template<typename PointType>
        void setFromIntermediateNode( const OctreeIntermediateNode<PointType>& node )
        {
            m_rgba = node.m_rgba;
            m_normalIndex = node.m_normal;
            m_childInfo = node.m_childInformation;
            m_svoBounds = node.m_svoBounds;
            m_firstChildIndex = node.m_firstChildIndex;
            m_indexIntoLeafArray = node.m_indexInToLeafArray;

            memcpy( m_misc, node.m_misc, sizeof( std::uint8_t ) * 4 );
        }

        RealityComputing::Common::RCVector4ub     m_rgba;               //average RGBA                    (4)
        std::uint16_t                                       m_normalIndex;        //index into normal lookup table  (2 (6))
        std::uint16_t                                       m_childInfo;          //child information               (2 (8))
        RealityComputing::Common::RCBox           m_svoBounds;          //bounding box of this node       (48 (56))
        std::uint8_t                                        m_misc[4];            //lidar, segments etc             (4 (60))
        int                                                 m_indexIntoLeafArray; //leaf found if this is not -1    (4 (64))
        int                                                 m_firstChildIndex;    //index to first child            (4 (68))
        char                                                dummy[4];             // explicit padding to 8-byte bnd (4 (72))
    };
    static_assert(sizeof(OctreeLeafIntermediateNodeSaveData) == 72, "sizeof(OctreeLeafIntermediateNodeSaveData) is incorrect"); // 68 + 4 pad

    template<typename PointType>
    struct OctreeIntermediateInformation
    {
        OctreeIntermediateInformation(){ }
        ~OctreeIntermediateInformation(){}

        std::vector<OctreeIntermediateNode<PointType>* >            m_umbrellaNodeList;                    //the nodes itself
    };

    //////////////////////////////////////////////////////////////////////////
    // \brief: returns bounding box & id of child based on input point 
    //           & parent bounding box
    //////////////////////////////////////////////////////////////////////////
    void getChildInformation( const RealityComputing::Common::RCBox& parentBounds, const RealityComputing::Common::RCVector3d& point, OctreeChildInformation& childInfoOut );

    static const int        UMBRELLA_LEAF_NODE_WIDTH    = 10000;
    static const int        MAX_UMBRELLA_NODE_LEVELS    = 32;

    enum VoxelContainerStatus
    {
        UNCHANGED,
        REMOVED,
        REINDEXED
    };

}}}
