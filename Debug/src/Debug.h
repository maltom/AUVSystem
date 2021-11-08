#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "CommonEnums.h"
#include "Displayer.h"
#include "jsonCommonFunctions.h"
#include "NodeBase.h"

class Debug final : public NodeBase
{
public:
	Debug( std::shared_ptr< ros::NodeHandle >& node, configFiles::fileID configID, AUVROS::NodeIDs nID )
	    : NodeBase( node, configID, nID )
	{
		auto debugRate = jsonFunctions::ROS::readDebugRate( this->configFileID );
		debugTickSpan  = static_cast< unsigned >( rosRate / debugRate );

		subscribeTopics();
		advertiseTopics();
	}
	~Debug() = default;

protected:
private:
	enum DisplayerDataPositions
	{
		Health
	};

	void processInMainLoop() override;
	void subscribeTopics() override;
	void advertiseTopics() override;
	void connectServices() override;

	bool countAndCompareNumberOfTopics() const;

	void displayNodeHealthStatus( const AUVROS::MessageTypes::HealthReport& report );

	Displayer displayer;

	ros::master::V_TopicInfo listOfTopics;
	unsigned debugTickSpan{ 0u };
	unsigned numberofIgnoredTopics{ 0u };
};