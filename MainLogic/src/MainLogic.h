#pragma once

#include <fstream>
#include <memory>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "jsonxx/jsonxx.h"

#include "NodeBase.h"
#include "StateMachine/StateMachine.h"

class MainLogic final : public NodeBase
{
public:
	MainLogic( std::shared_ptr< ros::NodeHandle >& node, std::fstream& config ) : NodeBase( node, config )
	{
		this->stateMachine = std::make_unique< StateMachine >();

		subscribeTopics();
	}
	~MainLogic() = default;

	void startMainLoop()  override;

protected:
	void subscribeTopics() override;
	void advertiseTopics() const override;
	void connectServices() const override;

private:
	std::unique_ptr< StateMachine > stateMachine;
	ros::Publisher globalEstimatedPositionPublisher;
	ros::Subscriber globalEstimatedPositionSubscriber;

	void globalEstimatedPositionObtained( const geometry_msgs::Twist& position );
};