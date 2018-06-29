#pragma once

#include <memory>
#include <vector>
#include <functional>

#include "OctreeIntermediateNode.h"

namespace ambergris { namespace RealityComputing { namespace Import {

    class PointCloudIndexerInterface;
    struct VoxelHelperNode;

    //////////////////////////////////////////////////////////////////////////
    //VoxelOctreeCreator Creates a SVO from an umbrella leaf node
    //////////////////////////////////////////////////////////////////////////
    template<typename PointType>
    class VoxelOctreeCreator
    {
    public:

        static const int            MAX_SVO_NODE_LEVELS     = 32;
        static const int            POINTS_PER_SVO_LEAF     =  1;    //one point per leaf node

        enum EVALUATE_METHOD
        {
            ENUM_EVALUATE_METHOD_GET_SPLIT_NODES            = 1,    //nodes that must be split
            ENUM_EVALUATE_METHOD_GET_MERGE_NODES            = 2,    //nodes that can be merged
            ENUM_EVALUATE_METHOD_GET_ALL_LEAF_NODES         = 3,    //return all leaf nodes
        };

        typedef std::function<bool(void)> CancelCallback;
        static bool dummy_callback() { return false; } //Simple dummy callback

        VoxelOctreeCreator(std::vector<PointType>& points, const std::wstring& voxelName, const std::wstring& tempFolder, const ambergris::RealityComputing::Common::RCBox& svoBounds,
            int maxDepth = 8, double stopSize = 0.001, double minPointDistance = 0.002, bool hasNormals = false, bool hasTimestamps = false, bool logOutput = false, const CancelCallback& cb=dummy_callback);

        virtual    ~VoxelOctreeCreator();

        //////////////////////////////////////////////////////////////////////////
        // \brief: Begin creation of Voxel Octree
        //////////////////////////////////////////////////////////////////////////
        virtual bool                                        createOctree();

        //////////////////////////////////////////////////////////////////////////
        // \brief: save voxel node information into disk and update metadata in an OctreeIntermediateNode
        //////////////////////////////////////////////////////////////////////////
        bool                                                saveToDisk(OctreeIntermediateNode<PointType>* leafPtr);

        bool                                                getCalculateNormalsFromUnstructuredData();
        void                                                setCalculateNormalsFromUnstructuredData(bool calculateNormals);

        bool                                                getHasNormal() const {return mHasNormals; }
        void                                                setHasNormal( bool hasNormal ) { mHasNormals = hasNormal; }

    protected:

    private:

        struct VoxelInformation
        {
            VoxelInformation(){ m_numLeafNodes = m_numNonLeafNodes = m_numCumulativeLeafNodes = 0; }
            ~VoxelInformation(){}

            std::vector<VoxelHelperNode*>                   m_interiorNodes;                    //the nodes itself

            int                                             m_numNonLeafNodes;                    //amount of non-leaf nodes in this depth level
            int                                             m_numLeafNodes;                        //amount of leaf nodes in this depth level
            int                                             m_numCumulativeLeafNodes;            //amount of cumulative leaf nodes in this depth level
        };

        //////////////////////////////////////////////////////////////////////////
        // \brief: Add a new point to the tree stack based
        //////////////////////////////////////////////////////////////////////////
        bool                                                addPointToTree( int index );

        //////////////////////////////////////////////////////////////////////////
        // \brief: Remove a node from the (potential) leaf list
        //////////////////////////////////////////////////////////////////////////
        bool                                                removeLeafNodeFromList( VoxelHelperNode* nodePtr );

        //////////////////////////////////////////////////////////////////////////
        // \brief: Evaluates current octree, returns the nodes that qualify 
        //           the evaluation argument
        //////////////////////////////////////////////////////////////////////////
        std::vector<VoxelHelperNode*>                       evaluateOctree( int evaluateMethod ) const;


        //////////////////////////////////////////////////////////////////////////
        // \brief: split voxel node if it contains more than 'm_maxAmountOfPointsPerLeaf'
        //           returns true if this one is split, false if no split is needed
        //           this assumes there are no duplicate points
        //////////////////////////////////////////////////////////////////////////
        bool                                                splitVoxelNode( VoxelHelperNode* nodePtr );

        //////////////////////////////////////////////////////////////////////////
        // \brief: determine if this node can be merged with its siblings
        //           if so the parent node becomes the leaf node
        //////////////////////////////////////////////////////////////////////////
        bool                                                canMergeVoxelNode( VoxelHelperNode* nodePtr ) const;

        //////////////////////////////////////////////////////////////////////////
        // \brief: Merge voxel node if all of its siblings contains less or equal then
        //           'm_maxAmountOfPointsPerLeaf' returns true if successfully merged
        //           false if can't merge
        //////////////////////////////////////////////////////////////////////////
        bool                                                mergeVoxelNode( VoxelHelperNode* parentPtr );

        //////////////////////////////////////////////////////////////////////////
        // \brief: Updates octree node data, rgba, normal etc
        //////////////////////////////////////////////////////////////////////////
        bool                                                updateOctreeLeafNodeData( int pointIndex );

        //////////////////////////////////////////////////////////////////////////
        // \brief: Merge all voxel nodes
        //////////////////////////////////////////////////////////////////////////
        bool                                                mergeVoxelTree();

        //////////////////////////////////////////////////////////////////////////
        // \brief: Split voxel leaf nodes until they contain less or equal then 'm_maxAmountOfPointsPerLeaf'
        //////////////////////////////////////////////////////////////////////////
        bool                                                splitVoxelTree();

        //////////////////////////////////////////////////////////////////////////
        // \brief: Linearize the svo tree(store in an array like data structure)
        //////////////////////////////////////////////////////////////////////////
        bool                                                linearizeSvoTree();

        bool                                                saveTimeStampToDisk();

        //////////////////////////////////////////////////////////////////////////
        // \brief: Calculates normals from unstructured data sets,
        //////////////////////////////////////////////////////////////////////////
        void                                                calculateNormalsFromUnstructuredData( int numNeighbours = 9 );

        ////////////////////////////////////////////////////////////////////////////
        //// \brief: Returns nearest neigbours inside a bounding box
        ////////////////////////////////////////////////////////////////////////////
        std::vector<int>                                    getNearestNeighbours( const ambergris::RealityComputing::Common::RCBox& bounds ) const;

        int                                                 m_startDepth;
        double                                              m_LeafStopSplitSize;
        double                                              m_minPointDistance;

        CancelCallback  m_cancelCb;

        std::wstring                                        m_fileName;
        std::wstring                                        m_timeStampFileName;

        //////////////////////////////////////////////////////////////////////////
        // \brief: the points that going to make up the svo
        //////////////////////////////////////////////////////////////////////////
        std::vector<PointType>&                             m_octreePoints;
        int                                                 m_amountOfPoints;

        //////////////////////////////////////////////////////////////////////////
        // \brief: Root node of svo
        //////////////////////////////////////////////////////////////////////////
        std::unique_ptr< VoxelHelperNode >                  m_rootPtr;

        //////////////////////////////////////////////////////////////////////////
        // \brief: List of leaf nodes
        //////////////////////////////////////////////////////////////////////////
        std::vector<VoxelHelperNode*>                       m_leafList;

        bool                                                m_outputToLog;

        int                                                 m_maxLeafDepth;
        VoxelInformation                                    m_voxelInfoList[MAX_SVO_NODE_LEVELS];

        bool                                                mCalculateNormalsFromUnstructuredData;
        bool                                                mHasNormals;
        bool                                                mHasTimestamps;
    };

}}}
