#pragma once

#include <fstream>
#include <memory>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "NodeBase.h"

class Debug final : public NodeBase
{
public:
	Debug( std::shared_ptr< ros::NodeHandle >& node, std::fstream& config ) : NodeBase( node, config )
	{
		subscribeTopics();
	}
	~Debug() {}

	void startMainLoop() override;

protected:
private:

	void subscribeTopics() override;
	void advertiseTopics() const override;
	void connectServices() const override;
};