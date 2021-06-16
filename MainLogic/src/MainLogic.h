#pragma once

#include <memory>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "jsonxx/jsonxx.h"

#include "StateMachine/StateMachine.h"

class MainLogic final
{
public:
	MainLogic( std::shared_ptr< ros::NodeHandle >& node, std::fstream& config ) : rosNode( node ), configFile( config )
	{
		this->stateMachine = std::make_unique< StateMachine >();
		file.parse( configFile );

        subscribeTopics();
	}
	~MainLogic() {}

	void startMainLoop() const;

protected:
private:
	std::unique_ptr< StateMachine > stateMachine;
	std::shared_ptr< ros::NodeHandle >& rosNode;
	ros::Publisher globalEstimatedPositionPublisher;
	ros::Subscriber globalEstimatedPositionSubscriber;

	std::fstream& configFile;
	jsonxx::Object file;

	void subscribeTopics();
	void advertiseTopics() const;
	void connectServices() const;

	void globalEstimatedPositionObtained( const geometry_msgs::Twist& position );
};