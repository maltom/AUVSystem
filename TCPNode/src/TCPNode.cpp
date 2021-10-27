#include "TCPNode.h"
#include "ROSEnums.h"

void TCPNode::processInLoop() {}

void TCPNode::subscribeTopics()
{
	// this->globalEstimatedPositionPublisher =  this->rosNode->advertise< std_msgs::Float32 >(
	// TopicsAndServicesNames::Topics::globalEstimatedPosition, 1000 );
}
void TCPNode::advertiseTopics() {}
void TCPNode::connectServices() {}
