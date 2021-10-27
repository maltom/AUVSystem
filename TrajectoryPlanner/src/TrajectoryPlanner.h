#pragma once

#include <memory>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "CommonEnums.h"
#include "NodeBase.h"

using wayPoint = geometry_msgs::Twist;

class TrajectoryPlanner final : public NodeBase
{
public:
	TrajectoryPlanner( std::shared_ptr< ros::NodeHandle >& node, configFiles::fileID configID, AUVROS::NodeIDs nID )
	    : NodeBase( node, configID, nID )
	{
		subscribeTopics();
		advertiseTopics();
	}
	~TrajectoryPlanner() {}

protected:
private:
	void processInLoop() override;
	void subscribeTopics() override;
	void advertiseTopics() override;
	void connectServices() override;

	std::vector< wayPoint > plannedPath;
	wayPoint currentPosition;
};