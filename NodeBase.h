#pragma once

#include <memory>
#include <vector>

#include <ros/ros.h>

#include "CommonEnums.h"
#include "jsonCommonFunctions.h"

class NodeBase
{
public:
	NodeBase( std::shared_ptr< ros::NodeHandle >& node, configFiles::fileID configID )
	    : rosNode( node ), configFileID( configID )
	{
		auto rate   = jsonFunctions::ROS::readRosRate( configFileID );
		rosLoopRate = std::make_unique< ros::Rate >( rate );
	}
	~NodeBase() = default;

	virtual void startMainLoop() = 0;

protected:
	std::vector< std::unique_ptr< ros::Publisher > > rosPublishers;
	std::vector< ros::Subscriber > rosSubscribers;
	std::shared_ptr< ros::NodeHandle >& rosNode;
	configFiles::fileID configFileID;
	std::unique_ptr< ros::Rate > rosLoopRate;

	virtual void subscribeTopics() = 0;
	virtual void advertiseTopics() = 0;
	virtual void connectServices() = 0;
};