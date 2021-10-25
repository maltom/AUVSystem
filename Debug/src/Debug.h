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
	Debug( std::shared_ptr< ros::NodeHandle >& node, configFiles::fileID configID ) : NodeBase( node, configID )
	{
		subscribeTopics();
	}
	~Debug() {}

	void startMainLoop() override;

protected:
private:
	void subscribeTopics() override;
	void advertiseTopics() override;
	void connectServices() override;
};