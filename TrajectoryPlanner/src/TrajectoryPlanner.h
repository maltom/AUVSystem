#pragma once

#include <fstream>
#include <memory>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "CommonEnums.h"
#include "NodeBase.h"

class TrajectoryPlanner final : public NodeBase
{
public:
	TrajectoryPlanner( std::shared_ptr< ros::NodeHandle >& node, configFiles::fileID configID ) : NodeBase( node, configID )
	{
		subscribeTopics();
	}
	~TrajectoryPlanner() {}

	void startMainLoop() override;

protected:
private:

	void subscribeTopics() override;
	void advertiseTopics() const override;
	void connectServices() const override;
};