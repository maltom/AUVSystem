#include "TCPNode.h"
#include "../jsonCommonFunctions.h"
#include "../ROSEnums.h"

void TCPNode::startMainLoop() const
{
	ros::Rate rosRate( jsonFunctions::readRosRate( configFile ) );
	while( ros::ok() )
	{
		ros::spinOnce();
		rosRate.sleep();
	}
}

void TCPNode::subscribeTopics()
{
	// this->globalEstimatedPositionPublisher =  this->rosNode->advertise< std_msgs::Float32 >(
	// TopicsAndServicesNames::Topics::globalEstimatedPosition, 1000 );
}
void TCPNode::advertiseTopics() const {}
void TCPNode::connectServices() const {}
