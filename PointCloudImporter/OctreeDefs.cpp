#include "OctreeDefs.h"

#include <common/RCMath.h>
#include <common/RCString.h>
#include <utility/RCLog.h>

#include <sstream>

using namespace ambergris::RealityComputing::Import;
using namespace ambergris::RealityComputing::Common;
using namespace ambergris::RealityComputing::Utility::Log;

//////////////////////////////////////////////////////////////////////////
// VoxelInteriorNode
//////////////////////////////////////////////////////////////////////////

AgVoxelInteriorNode::AgVoxelInteriorNode()
{
	m_data = 0;
}

AgVoxelInteriorNode::AgVoxelInteriorNode(const AgVoxelInteriorNode& cpy)
{
	m_data = cpy.m_data;
}

AgVoxelInteriorNode::~AgVoxelInteriorNode()
{

}

void AgVoxelInteriorNode::setIndexToChild(int childIndex)
{
	m_data &= ~(0x7FFFF);
	m_data |= childIndex;
}

void AgVoxelInteriorNode::setChildInfo(int val)
{
	m_data &= ~(0xFF << 19);
	m_data |= (0xFF & val) << 19;
}

void AgVoxelInteriorNode::setLeafInfo(int val)
{
	m_data &= ~(0x1 << 27);
	m_data |= (0x1 & val) << 27;
}


void AgVoxelInteriorNode::testData()
{
	for (int i = 0; i < 1000; i++)
	{
		int indexToChild = Math::random(1, 512000);
		int childInfo = Math::random(0, 255);
		int isLeaf = Math::random(0, 1);

		setIndexToChild(indexToChild);
		setChildInfo(childInfo);
		setLeafInfo(isLeaf);

		{
			RCLog::getLogFile()->addMessage(L"================================================");
			RCLog::getLogFile()->addMessage(L"Before");

			std::wstringstream ss;
			ss << indexToChild << ", " << childInfo << ", " << isLeaf;
			RCLog::getLogFile()->addMessage(ss.str().c_str());
		}

		indexToChild = getChildIndex();
		childInfo = getChildInfo();
		isLeaf = getLeafInfo();

		{
			RCLog::getLogFile()->addMessage(L"After");

			std::wstringstream ss;
			ss << indexToChild << ", " << childInfo << ", " << isLeaf;
			RCLog::getLogFile()->addMessage(ss.str().c_str());
		}


	}
}

int AgVoxelInteriorNode::getChildIndex() const
{
	return int(m_data & 0x7FFFF);
}

int AgVoxelInteriorNode::getChildInfo() const
{
	return int((m_data >> 19) & 0xFF);
}

int AgVoxelInteriorNode::getLeafInfo()
{
	return int((m_data >> 27) & 0x1);
}