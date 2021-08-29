#pragma once

#include <fstream>
#include <memory>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "jsonxx/jsonxx.h"


class TrajectoryPlanner final
{
public:
	TrajectoryPlanner( std::shared_ptr< ros::NodeHandle >& node, std::fstream& config ) : rosNode( node ), rawConfigFile( config )
	{
		this->configFile.parse(rawConfigFile);
		
        subscribeTopics();
	}
	~TrajectoryPlanner() {}

	void startMainLoop() const;

protected:
private:
	std::shared_ptr< ros::NodeHandle >& rosNode;

	std::fstream& rawConfigFile;
	jsonxx::Object configFile;

	void subscribeTopics();
	void advertiseTopics() const;
	void connectServices() const;
};