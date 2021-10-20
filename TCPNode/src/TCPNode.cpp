#include "TCPNode.h"
#include "ROSEnums.h"

void TCPNode::startMainLoop()
{
	while( ros::ok() )
	{
		ros::spinOnce();
		this->rosLoopRate->sleep();
	}
}

void TCPNode::subscribeTopics()
{
	// this->globalEstimatedPositionPublisher =  this->rosNode->advertise< std_msgs::Float32 >(
	// TopicsAndServicesNames::Topics::globalEstimatedPosition, 1000 );
}
void TCPNode::advertiseTopics() const {}
void TCPNode::connectServices() const {}
