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
	MainLogic( std::shared_ptr< ros::NodeHandle >& node, std::fstream& config ) : rosNode( node ), rawConfigFile( config )
	{
		this->stateMachine = std::make_unique< StateMachine >();
		this->configFile.parse( rawConfigFile );
		
        subscribeTopics();
		readRosRate();
	}
	~MainLogic() {}

	void startMainLoop() const;

protected:
private:
	std::unique_ptr< StateMachine > stateMachine;
	std::shared_ptr< ros::NodeHandle >& rosNode;
	ros::Publisher globalEstimatedPositionPublisher;
	ros::Subscriber globalEstimatedPositionSubscriber;
	double rosContactRate{0.0};

	std::fstream& rawConfigFile;
	jsonxx::Object configFile;

	void subscribeTopics();
	void advertiseTopics() const;
	void connectServices() const;
	void readRosRate();

	void globalEstimatedPositionObtained( const geometry_msgs::Twist& position );
};