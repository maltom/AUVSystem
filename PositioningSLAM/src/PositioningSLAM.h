#pragma once

#include <memory>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "CommonEnums.h"
#include "NodeBase.h"

class PositioningSLAM final : public NodeBase
{
public:
	PositioningSLAM( std::shared_ptr< ros::NodeHandle >& node, configFiles::fileID configID, AUVROS::NodeIDs nID )
	    : NodeBase( node, configID, nID )
	{
		subscribeTopics();
		advertiseTopics();
	}
	~PositioningSLAM() {}

protected:
private:
	void processInMainLoop() override;
	void subscribeTopics() override;
	void advertiseTopics() override;
	void connectServices() override;


	void update
};