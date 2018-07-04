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
#define VOXEL_RESOURCE_EXTENSION           L".vxl" //L".resouce"

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

		}
	}
}// ns
