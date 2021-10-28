#pragma once

#include <memory>

#include <geometry_msgs/Twist.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "CommonEnums.h"
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
	void processInMainLoop() override;
	void subscribeTopics() override;
	void advertiseTopics() override;
	void connectServices() override;

	void displayDebugInfo() const;

	unsigned debugTickSpan{0u};
	ros::master::V_TopicInfo listOfTopics;
};