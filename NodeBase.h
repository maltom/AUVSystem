#pragma once

#include <fstream>
#include <memory>

#include <ros/ros.h>

#include "jsonxx/jsonxx.h"

class NodeBase
{
public:
	NodeBase( std::shared_ptr< ros::NodeHandle >& node, std::fstream& config ) : rosNode( node ), rawConfigFile( config )
	{
		this->configFile.parse( rawConfigFile );
	}
	~NodeBase() = default;

	virtual void startMainLoop() = 0;

protected:
	std::shared_ptr< ros::NodeHandle >& rosNode;

	std::fstream& rawConfigFile;
	jsonxx::Object configFile;

	virtual void subscribeTopics() = 0;
	virtual void advertiseTopics() const = 0;
	virtual void connectServices() const = 0;
};