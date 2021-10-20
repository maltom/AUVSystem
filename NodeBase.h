#pragma once

#include <fstream>
#include <memory>

#include <ros/ros.h>

#include "jsonCommonFunctions.h"

class NodeBase
{
public:
	NodeBase( std::shared_ptr< ros::NodeHandle >& node, std::fstream& config ) : rosNode( node ), configFile( config )
	{
		rosLoopRate = std::make_unique< ros::Rate >( jsonFunctions::ROS::readRosRate( config ) );
	}
	~NodeBase() = default;

	virtual void startMainLoop() = 0;

protected:
	std::shared_ptr< ros::NodeHandle >& rosNode;

	std::fstream& configFile;
	std::unique_ptr< ros::Rate > rosLoopRate;

	virtual void subscribeTopics()       = 0;
	virtual void advertiseTopics() const = 0;
	virtual void connectServices() const = 0;
};