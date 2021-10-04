#pragma once

#include <fstream>
#include <memory>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "jsonxx/jsonxx.h"

#include "NodeBase.h"

class TrajectoryPlanner final : public NodeBase
{
public:
	TrajectoryPlanner( std::shared_ptr< ros::NodeHandle >& node, std::fstream& config ) : NodeBase( node, config )
	{
		subscribeTopics();
	}
	~TrajectoryPlanner() {}

	void startMainLoop() const;

protected:
private:

	void subscribeTopics();
	void advertiseTopics() const;
	void connectServices() const;
};