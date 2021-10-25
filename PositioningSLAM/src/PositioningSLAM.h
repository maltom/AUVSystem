#pragma once

#include <memory>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "CommonEnums.h"
#include "NodeBase.h"

using position = geometry_msgs::Twist;

class PositioningSLAM final : public NodeBase
{
public:
	PositioningSLAM( std::shared_ptr< ros::NodeHandle >& node, configFiles::fileID configID )
	    : NodeBase( node, configID )
	{
		subscribeTopics();
	}
	~PositioningSLAM() {}

	void startMainLoop() override;

protected:
private:
	void subscribeTopics() override;
	void advertiseTopics() override;
	void connectServices() override;

};