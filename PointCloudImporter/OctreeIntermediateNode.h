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

	template<typename PointType>
	struct OctreeIntermediateInformation
	{
		OctreeIntermediateInformation() { }
		~OctreeIntermediateInformation() {}

		std::vector<OctreeIntermediateNode<PointType>* >            m_umbrellaNodeList;                    //the nodes itself
	};

	//////////////////////////////////////////////////////////////////////////
	// \brief: This data will be written to disk, these will hold the actual point data
	//////////////////////////////////////////////////////////////////////////
	struct OctreeLeafSaveData
	{
		OctreeLeafSaveData()
			: m_maxDepth(0), m_amountOfPoints(0)
		{ }

		template<typename PointType>
		void setFromIntermediateNode(const OctreeIntermediateNode<PointType>* node)
		{
			m_svoBounds = node->m_svoBounds;
			m_nodeBounds = node->m_nodeBounds;
			m_maxDepth = node->m_maxTreeDepth;
			m_amountOfPoints = node->m_amountOfPointsInFile;
			m_fileName = node->m_fileName;
		}

		RealityComputing::Common::RCBox            m_svoBounds;    // 48
		RealityComputing::Common::RCBox            m_nodeBounds;   // 48 (96)
		int                            m_maxDepth;                           // 4 (100)
		int                            m_amountOfPoints;                     // 4 (104)
		std::wstring                   m_fileName;
	};

    //////////////////////////////////////////////////////////////////////////
    // \brief: returns bounding box & id of child based on input point 
    //           & parent bounding box
    //////////////////////////////////////////////////////////////////////////
    void getChildInformation( const RealityComputing::Common::RCBox& parentBounds, const RealityComputing::Common::RCVector3d& point, OctreeChildInformation& childInfoOut );

    static const int        UMBRELLA_LEAF_NODE_WIDTH    = 10000;
    static const int        MAX_UMBRELLA_NODE_LEVELS    = 32;

}}}
