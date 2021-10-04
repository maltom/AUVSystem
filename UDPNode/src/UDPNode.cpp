#include "UDPNode.h"
#include "../jsonCommonFunctions.h"
#include "../ROSEnums.h"

void UDPNode::startMainLoop() const
{
	ros::Rate rosRate( jsonFunctions::readRosRate( configFile ) );
	while( ros::ok() )
	{
		ros::spinOnce();
		rosRate.sleep();
	}
}

void UDPNode::subscribeTopics()
{
	// this->globalEstimatedPositionPublisher =  this->rosNode->advertise< std_msgs::Float32 >(
	// TopicsAndServicesNames::Topics::globalEstimatedPosition, 1000 );
}
void UDPNode::advertiseTopics() const {}
void UDPNode::connectServices() const {}

void UDPNode::loadNetworkConfig()
{
	
}