#pragma once

#include <memory>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "CommonEnums.h"
#include "NodeBase.h"

class Debug final : public NodeBase
{
public:
	Debug( std::shared_ptr< ros::NodeHandle >& node, configFiles::fileID configID, AUVROS::NodeIDs nID )
	    : NodeBase( node, configID, nID )
	{
		subscribeTopics();
		advertiseTopics();
	}
	~Debug() {}

protected:
private:
	void processInLoop() override;
	void subscribeTopics() override;
	void advertiseTopics() override;
	void connectServices() override;
};