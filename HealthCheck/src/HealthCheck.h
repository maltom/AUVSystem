#pragma once

#include <bitset>
#include <chrono>
#include <memory>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "CommonEnums.h"
#include "jsonCommonFunctions.h"
#include "NodeBase.h"
#include "ROSEnums.h"

class HealthCheck final : public NodeBase
{
public:
	HealthCheck( std::shared_ptr< ros::NodeHandle >& node, configFiles::fileID configID, AUVROS::NodeIDs nID )
	    : NodeBase( node, configID, nID )
	{
		auto healthCheckRate = jsonFunctions::ROS::readHealthCheckRate( this->configFileID );
		healthCheckTickSpan  = static_cast< unsigned >( rosRate / healthCheckRate );
		clearDump();
		subscribeTopics();
		advertiseTopics();
		std::this_thread::sleep_for( std::chrono::seconds( 5 ) );
	}
	~HealthCheck() {}

protected:
private:
	enum PublishersCodes
	{
		healthSummary = 0
	};
	void processInMainLoop() override;
	void subscribeTopics() override;
	void advertiseTopics() override;
	void connectServices() override;
	void reportHealth() override;

	void reportGlobalHealth();

	void registerNodeHealthStatus( const AUVROS::MessageTypes::HealthReport& report );
	void clearDump();
	void writeDeadNodesToDump( std::bitset< AUVROS::NodeIDs::Count > nodesHealthSet );
	std_msgs::Int32 globalHealthStatus;
	unsigned healthCheckTickSpan{ 0u };
};