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

	this->rosPublishers.emplace_back(
	    std::make_unique< ros::Publisher >( this->rosNode->advertise< AUVROS::MessageTypes::DVLVelocity >(
	        AUVROS::Topics::HardwareSignals::DVLVelocityData, AUVROS::QueueSize::DVLQueueSize ) ) );

	this->rosPublishers.emplace_back(
	    std::make_unique< ros::Publisher >( this->rosNode->advertise< AUVROS::MessageTypes::DVLDRTimeStamp >(
	        AUVROS::Topics::HardwareSignals::DVLDRTimeStamp, AUVROS::QueueSize::DVLQueueSize ) ) );
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
	{
		AUVROS::MessageTypes::DVLVelocity rosVelocityMsg;
		auto& data              = std::get< ProcessedVelocityFrame >( frame.content );
		rosVelocityMsg.linear.x = static_cast< float >( data.vx );
		rosVelocityMsg.linear.y = static_cast< float >( data.vy );
		rosVelocityMsg.linear.z = static_cast< float >( data.vz );

		this->rosPublishers.at( PublishersCodes::DVLVelocity )->publish( rosVelocityMsg );
	}
	break;
	case Frame::Type::deadReckoning:
	{
		auto& data = std::get< ProcessedDeadReckoningFrame >( frame.content );

		AUVROS::MessageTypes::DVLDeadReckoning rosDeadReckMsg;
		rosDeadReckMsg.data = { static_cast< float >( data.x ),        static_cast< float >( data.y ),
			                    static_cast< float >( data.z ),        static_cast< float >( data.roll ),
			                    static_cast< float >( data.pitch ),    static_cast< float >( data.yaw ),
			                    static_cast< float >( data.timeStamp ) };

		std_msgs::Float64 timeStamp;
		timeStamp.data = data.timeStamp;
		this->rosPublishers.at( PublishersCodes::DVLDeadReckoning )->publish( rosDeadReckMsg );
		this->rosPublishers.at( PublishersCodes::DVLDRTimeStamp )->publish( timeStamp );
	}
	break;
	default:
		break;
	}
}

void TCPNode::resetDeadReckoning()
{
	this->outgoingMessages.emplace( "{\"command\": \"reset_dead_reckoning\"}" );
}