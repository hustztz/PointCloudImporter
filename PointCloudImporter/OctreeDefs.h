#pragma once

#include <cstdint>

namespace ambergris {
	namespace RealityComputing {
		namespace Import {

#define NODE_FILE_NAME                           L"inter2_0"   // L"node_0"
#define TEMP_POINT_FILE_NAME                     L"inter7_0"  // L"tempfile_0"
#define SPLIT_NODE_FILE_NAME                     L"inter5_0"    // L"split_node_0"

#define TEMP_POINT_EXTENSION                     L".temp05" //L".pointtemp"
#define TEMP_UMBRELLA_LEAF_NODE_EXTENSION        L".temp11" //L".umbnode"
#define TEMP_VOXEL_OCTREE_EXTENSION              L".temp07" //L".svonode"
#define TEMP_VOXEL_TIMESTAMP_EXTENSION           L".temp013" //L".svogpstime"

#define                 MAX_SIZE_TERRESTRIAL_UMBRELLA_LEAFNODE  2.5     //we can use 12 bytes per point
#define                 MAX_SIZE_LIDAR_UMBRELLA_LEAFNODE        262.0   //lidar/aerial data
#define                 MAX_POINTS_PER_OCTREE_LEAFNODE          (  512 * 1024 - 1 )
#define                 MIN_SVO_LEAF_POINTS                     1

			enum class AddPointResult
			{
				OK,
				FILTERED,
				FATAL_ERROR
			};

			enum NOISE_FILTER
			{
				NOISE_FILTER_MINIMAL,
				NOISE_FILTER_MEDIUM,
				NOISE_FILTER_AGGRESSIVE
			};

			enum UP_DIRECTION
			{
				UP_DIRECTION_X,
				UP_DIRECTION_Y,
				UP_DIRECTION_Z,
				UP_DIRECTION_NEGATIVE_X,
				UP_DIRECTION_NEGATIVE_Y,
				UP_DIRECTION_NEGATIVE_Z
			};


			//////////////////////////////////////////////////////////////////////////
			// \brief: Stores the spatial information of a umbrella leaf node
			//////////////////////////////////////////////////////////////////////////
			class AgVoxelInteriorNode
			{
			public:

				/*
				SVO Interior Node ( 32 bits total )
				[0.....18]          Index to first child, if this is a leaf points to leaf array
				[19....26]          Child Information
				[27....27]          Is Leaf
				[28....31]          Reserved
				*/
				AgVoxelInteriorNode();
				AgVoxelInteriorNode(const AgVoxelInteriorNode& cpy);
				~AgVoxelInteriorNode();

				//////////////////////////////////////////////////////////////////////////
				// \brief: Sets the index to the first child, or points to the leaf index
				// array if it is a leaf
				//////////////////////////////////////////////////////////////////////////
				void                    setIndexToChild(int childIndex);

				//////////////////////////////////////////////////////////////////////////
				// \brief: Sets the child information( all 8 of them )
				//////////////////////////////////////////////////////////////////////////
				void                    setChildInfo(int val);

				//////////////////////////////////////////////////////////////////////////
				// \brief: Sets whether of not this node is a leaf, thus its
				//         child offset pointing to the leaf index
				//////////////////////////////////////////////////////////////////////////
				void                    setLeafInfo(int val);

				//////////////////////////////////////////////////////////////////////////
				// \brief: Returns the index to the first child
				//////////////////////////////////////////////////////////////////////////
				int                     getChildIndex() const;

				//////////////////////////////////////////////////////////////////////////
				// \brief: Return the bitfield info about this node's sibbling
				//////////////////////////////////////////////////////////////////////////
				int                     getChildInfo() const;

				//////////////////////////////////////////////////////////////////////////
				// \brief: Return if this node is a leaf or not
				//////////////////////////////////////////////////////////////////////////
				int                     getLeafInfo();

				void                    testData();


			private:

				std::uint32_t                               m_data;

			};
			static_assert(alignof(AgVoxelInteriorNode) == 4, "alignof(AgVoxelInteriorNode) is incorrect");
			static_assert(sizeof(AgVoxelInteriorNode) == 4, "sizeof(AgVoxelInteriorNode) is incorrect");

		}
	}
}// ns
