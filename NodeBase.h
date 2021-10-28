#pragma once

#include <memory>
#include <vector>

#include <ros/ros.h>

#include "CommonEnums.h"
#include "jsonCommonFunctions.h"
#include "ROSEnums.h"

class NodeBase
{
public:
	NodeBase( std::shared_ptr< ros::NodeHandle >& node, configFiles::fileID configID, AUVROS::NodeIDs nID )
	    : rosNode( node ), configFileID( configID ), nodeID( nID )
	{
		rosRate     = jsonFunctions::ROS::readRosRate( configFileID );
		rosLoopRate = std::make_unique< ros::Rate >( rosRate );

		auto healthReportRate = jsonFunctions::ROS::readHealthReportRate( this->configFileID );
		healthReportTickSpan  = static_cast< unsigned >( rosRate / healthReportRate );

		healthReportPublisher = this->rosNode->advertise< AUVROS::MessageTypes::HealthReport >(
		    AUVROS::Topics::Health::healthReportSingleNode, AUVROS::QueueSize::HealthQueueSize, true );
	}
	~NodeBase() = default;

	virtual void startMainLoop();

protected:
	virtual void processInMainLoop()   = 0;
	virtual void subscribeTopics() = 0;
	virtual void advertiseTopics() = 0;
	virtual void connectServices() = 0;
	virtual void reportHealth();

	std::vector< std::unique_ptr< ros::Publisher > > rosPublishers;
	std::vector< ros::Subscriber > rosSubscribers;
	ros::Publisher healthReportPublisher;
	std::shared_ptr< ros::NodeHandle >& rosNode;
	configFiles::fileID configFileID;
	std::unique_ptr< ros::Rate > rosLoopRate;
	AUVROS::NodeIDs nodeID;

	double rosRate{ 0.0 };
	unsigned long long ticks{ 0ull };
	
private:
	unsigned healthReportTickSpan{ 0u };
};