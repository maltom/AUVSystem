#include "TCPNode.h"

#include <array>
#include <cstdlib>
#include <exception>
#include <vector>

#include "CommonEnums.h"
#include "jsonCommonFunctions.h"
#include "MessageProcessing.h"
#include "ROSEnums.h"

void TCPNode::processInMainLoop()
{
	tcpClient->sendOutgoingMessages( outgoingMessages );
	tcpClient->getIncomingMessages( incomingMessages );
	this->processIncomingMessages();
}

void TCPNode::subscribeTopics() {}

void TCPNode::advertiseTopics()
{
	this->rosPublishers.emplace_back(
	    std::make_unique< ros::Publisher >( this->rosNode->advertise< AUVROS::MessageTypes::ThrustersSignal >(
	        AUVROS::Topics::DevPC::arbitrarlySetThrusters, AUVROS::QueueSize::StandardQueueSize ) ) );

	this->rosPublishers.emplace_back(
	    std::make_unique< ros::Publisher >( this->rosNode->advertise< AUVROS::MessageTypes::ServosSignal >(
	        AUVROS::Topics::DevPC::arbitrarlySetServos, AUVROS::QueueSize::StandardQueueSize ) ) );

	this->rosPublishers.emplace_back(
	    std::make_unique< ros::Publisher >( this->rosNode->advertise< AUVROS::MessageTypes::Position >(
	        AUVROS::Topics::DevPC::arbitrarlySetGlobalPosition, AUVROS::QueueSize::StandardQueueSize ) ) );

	this->rosPublishers.emplace_back(
	    std::make_unique< ros::Publisher >( this->rosNode->advertise< AUVROS::MessageTypes::ServosSignal >(
	        AUVROS::Topics::DevPC::arbitrarlySetRelativePosition, AUVROS::QueueSize::StandardQueueSize ) ) );
}

void TCPNode::connectServices() {}

void TCPNode::loadNetworkConfig()
{
	try
	{
		this->serverPort   = jsonFunctions::network::readDevicePortNumber( configFileID, network::Device::DVL );
		this->clientPort   = jsonFunctions::network::readDevicePortNumber( configFileID, network::Device::jetson );
		this->serverAdress = jsonFunctions::network::readDeviceIPNumber( configFileID, network::Device::DVL );
		this->clientAdress = jsonFunctions::network::readDeviceIPNumber( configFileID, network::Device::jetson );
	}
	catch( const std::exception& e )
	{
		std::cerr << e.what() << '\n';
	}
}

void TCPNode::processIncomingMessages()
{
	while( !incomingMessages.empty() )
	{
		auto income = incomingMessages.front();
		processCommand( decomposeFrame( income ) );
		incomingMessages.pop();
	}
}

Frame TCPNode::decomposeFrame( const network::TCPunstickedMessage& incMsg )
{
	Frame result( incMsg );
	result.processMe();
	return result;
}

void TCPNode::processCommand( const Frame& frame )
{
	switch( frame.currentType )
	{
	case Frame::Type::velocity:
		std::cout << "Pryndkosc: " << std::get< ProcessedVelocityFrame >( frame.content ).figureOfMerit << "\n\n"
		          << std::endl;
		break;
	case Frame::Type::deadReckoning:
		std::cout << "DeadReck: " << std::get< ProcessedDeadReckoningFrame >( frame.content ).pitch << "\n\n"
		          << std::endl;
		break;
	default:
		break;
	}
	if( frame.currentType == Frame::Type::velocity )
	{
	}
}
