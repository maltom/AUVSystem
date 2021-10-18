#include "UDPNode.h"

#include <exception>
#include <iostream>

#include "CommonEnums.h"
#include "jsonCommonFunctions.h"
#include "ROSEnums.h"

void UDPNode::startMainLoop()
{
	ros::Rate rosRate( jsonFunctions::ROS::readRosRate( configFile ) );
	udpServer->startServer();

	while( ros::ok() )
	{
		ros::spinOnce();

		udpServer->sendOutgoingMessages( outgoingMessages );
		udpServer->getIncomingMessages( incomingMessages );
		this->processIncomingMessages();
		rosRate.sleep();
		// std::cout << "zyje \n\n";
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
	try
	{
		this->serverPort = jsonFunctions::network::readDevicePortNumber( configFile, network::Device::jetson );
		this->clientPort = jsonFunctions::network::readDevicePortNumber( configFile, network::Device::microcontroller );
	}
	catch( const std::exception& e )
	{
		std::cerr << e.what() << '\n';
	}
}

void UDPNode::processIncomingMessages()
{
	while( !incomingMessages.empty() )
	{
		incomingMessages.front();
		incomingMessages.pop();
	}
}