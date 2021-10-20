#pragma once

#include <iostream>
#include <fstream>
#include <memory>

#include <ros/ros.h>

#include "jsonCommonFunctions.h"

class NodeBase
{
public:
	NodeBase( std::shared_ptr< ros::NodeHandle >& node, std::fstream& config ) : rosNode( node ), configFile( config )
	{
		auto rate   = jsonFunctions::ROS::readRosRate( config );
		rosLoopRate = std::make_unique< ros::Rate >( rate );
		
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