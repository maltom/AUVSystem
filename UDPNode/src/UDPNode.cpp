#include "UDPNode.h"

#include <exception>

#include "CommonEnums.h"
#include "jsonCommonFunctions.h"
#include "ROSEnums.h"

void UDPNode::startMainLoop()
{
	ros::Rate rosRate( jsonFunctions::ROS::readRosRate( configFile ) );
	std::cout << "wstalem";
	this->udpServer = std::make_unique< UDPServer >(this->serverPort );

	while( ros::ok() )
	{
		ros::spinOnce();
		rosRate.sleep();
		std::cout << "zyje ";
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

