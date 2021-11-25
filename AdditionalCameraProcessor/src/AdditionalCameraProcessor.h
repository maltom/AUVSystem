#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "CameraReceiver.h"
#include "CommonEnums.h"
#include "jsonCommonFunctions.h"
#include "NodeBase.h"

class AdditionalCameraProcessor final : public NodeBase
{
public:
	AdditionalCameraProcessor( std::shared_ptr< ros::NodeHandle >& node,
	                           configFiles::fileID configID,
	                           AUVROS::NodeIDs nID )
	    : NodeBase( node, configID, nID )
	{
		subscribeTopics();
		advertiseTopics();
	}
	~AdditionalCameraProcessor() = default;

protected:
private:
	void processInMainLoop() override;
	void subscribeTopics() override;
	void advertiseTopics() override;
	void connectServices() override;
};