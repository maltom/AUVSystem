#pragma once

#include <fstream>
#include <memory>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "CommonEnums.h"
#include "NodeBase.h"
#include "ROSEnums.h"
#include "StateMachine/StateMachine.h"

class MainLogic final : public NodeBase
{
public:
	MainLogic( std::shared_ptr< ros::NodeHandle >& node, configFiles::fileID configID, AUVROS::NodeIDs nID )
	    : NodeBase( node, configID, nID )
	{
		this->stateMachine = std::make_unique< StateMachine >();

		subscribeTopics();
		advertiseTopics();
	}
	~MainLogic() = default;

protected:
	void processInLoop() override;
	void subscribeTopics() override;
	void advertiseTopics() override;
	void connectServices() override;

private:
	std::unique_ptr< StateMachine > stateMachine;
	ros::Publisher globalEstimatedPositionPublisher;
	ros::Subscriber globalEstimatedPositionSubscriber;

	void globalEstimatedPositionObtained( const geometry_msgs::Twist& position );
};