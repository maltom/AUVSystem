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
	    std::make_unique< ros::Publisher >( this->rosNode->advertise< AUVROS::MessageTypes::DVLDeadReckoning >(
	        AUVROS::Topics::HardwareSignals::DVLDeadReckoningData, AUVROS::QueueSize::DVLQueueSize ) ) );
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
		break;
	case Frame::Type::deadReckoning:
	{
		AUVROS::MessageTypes::DVLDeadReckoning rosDeadReckMsg;
		auto& data          = std::get< ProcessedDeadReckoningFrame >( frame.content );
		rosDeadReckMsg.data = { static_cast< float >( data.timeStamp ), static_cast< float >( data.x ),
			                    static_cast< float >( data.y ),         static_cast< float >( data.z ),
			                    static_cast< float >( data.roll ),      static_cast< float >( data.pitch ),
			                    static_cast< float >( data.yaw ) };
		this->rosPublishers.at( PublishersCodes::DVLDeadReckoning )->publish( rosDeadReckMsg );
	}
	break;
	default:
		break;
	}
}
