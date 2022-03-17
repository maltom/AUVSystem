#include "UDPNode.h"
#include <iostream>
#include <array>
#include <cstdlib>
#include <exception>
#include <vector>

#include "CommonEnums.h"
#include "jsonCommonFunctions.h"
#include "MessageProcessing.h"
#include "ROSEnums.h"

void UDPNode::processInMainLoop()
{
	udpServer->sendOutgoingMessages( outgoingMessages );
	udpServer->getIncomingMessages( incomingMessages );
	this->processIncomingMessages();
}

void UDPNode::subscribeTopics()
{
	this->rosSubscribers.emplace_back( this->rosNode->subscribe( AUVROS::Topics::HardwareSignals::signalToThrusters,
	                                                             AUVROS::QueueSize::StandardQueueSize,
	                                                             &UDPNode::sendThrustersSignalToMicroController,
	                                                             this ) );
	this->rosSubscribers.emplace_back( this->rosNode->subscribe( AUVROS::Topics::HardwareSignals::signalToServos,
	                                                             AUVROS::QueueSize::StandardQueueSize,
	                                                             &UDPNode::sendServosSignalToMicroController,
	                                                             this ) );
	this->rosSubscribers.emplace_back( this->rosNode->subscribe( AUVROS::Topics::DevPC::arbitrarlySetThrusters,
	                                                             AUVROS::QueueSize::StandardQueueSize,
	                                                             &UDPNode::sendThrustersSignalToMicroController,
	                                                             this ) );
	this->rosSubscribers.emplace_back( this->rosNode->subscribe( AUVROS::Topics::DevPC::arbitrarlySetServos,
	                                                             AUVROS::QueueSize::StandardQueueSize,
	                                                             &UDPNode::sendServosSignalToMicroController,
	                                                             this ) );
}
void UDPNode::advertiseTopics()
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
void UDPNode::connectServices() {}

void UDPNode::loadNetworkConfig()
{
	try
	{
		this->serverPort = jsonFunctions::network::readDevicePortNumber( configFileID, network::Device::jetson );
		this->clientPort
		    = jsonFunctions::network::readDevicePortNumber( configFileID, network::Device::microcontroller );
		this->serverAdress = jsonFunctions::network::readDeviceIPNumber( configFileID, network::Device::jetson );
		this->clientAdress
		    = jsonFunctions::network::readDeviceIPNumber( configFileID, network::Device::microcontroller );
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
		auto income = incomingMessages.front();
		processCommand( decomposeFrame( income ) );
		incomingMessages.pop();
	}
}

UDPNode::Frame UDPNode::decomposeFrame( const network::UDPincomingMessage& incMsg )
{
	Frame result;
	result.commandCode = static_cast< Command >( incMsg.at( network::UDPcommandPositionFrame ) );
	result.payloadSize = static_cast< uint8_t >( incMsg.at( network::UDPpayloadSizePositionFrame ) );

	auto resultPayloadIndex = 0;
	for( auto i = network::UDPpayloadStartPositionFrame;
	     i < ( network::UDPpayloadStartPositionFrame + result.payloadSize );
	     i += network::UDPonePayloadWordByteSize )
	{
		result.payload.at( resultPayloadIndex ) = incMsg.at( i );
		++resultPayloadIndex;
	}
	return result;
}

void UDPNode::processCommand( const Frame& frame )
{
	switch( frame.commandCode )
	{
	case Command::HEARTBEAT:
		break;
	default:
		break;
	}
}

void UDPNode::processOutgoingMessages( const Frame& frame )
{
	network::UDPoutgoingMessage message;

	message.push_back( static_cast< char >( frame.commandCode ) );
	message.push_back( static_cast< char >( frame.payloadSize ) );

	for( auto i = 0u; i < frame.payloadSize; ++i )
	{
		const char* bytePointer = reinterpret_cast< const char* >( &( frame.payload[ i ] ) );
		std::array< char, network::UDPonePayloadWordByteSize > byteArray;

		for( auto j = 0u; j < network::UDPonePayloadWordByteSize; ++j )
		{
			byteArray.at( j ) = *bytePointer;
			++bytePointer;
		}

		for( auto it = byteArray.begin(); it != byteArray.end(); ++it )
		{
			message.push_back( *it );
		}
	}
	this->outgoingMessages.push( message );
}

void UDPNode::sendThrustersSignalToMicroController( const AUVROS::MessageTypes::ThrustersSignal& message )
{
	auto length = message.layout.dim.begin()->size;
	Frame frame;
	frame.commandCode = NORESPREQ_SET_THRUSTERS;
	frame.payloadSize = length;
	for( auto i = 0u; i < length; ++i )
	{
		frame.payload[ i ] = adjustThrusterValues( message.data[ i ] );
	}

	this->processOutgoingMessages( frame );
}

void UDPNode::sendServosSignalToMicroController( const AUVROS::MessageTypes::ServosSignal& message )
{
	auto length = message.layout.dim.begin()->size;
	if( length != 2 )
	{
		throw std::runtime_error( "Too many servos." );
	}
	Frame frame;
	Frame frame2;
	frame.commandCode  = NORESPREQ_SET_SERVOS;
	frame2.commandCode = NORESPREQ_SET_SERVOS;
	frame.payloadSize  = length;
	frame2.payloadSize = length;
	// for( auto i = 0u; i < length; ++i )
	// {
	// 	frame.payload[ i ] = adjustServoValues( message.data[ i ] );
	// 	std::cout<<frame.payload[i]<<std::endl;
	// }
	frame.payload[ 0 ]  = 0;
	frame.payload[ 1 ]  = hardware::servoMinMax.second - adjustServoValues( message.data[ 0 ] );
	frame2.payload[ 0 ] = 1;
	frame2.payload[ 1 ] = adjustServoValues( message.data[ 1 ] );

	std::cout << frame.payload[ 0 ] << std::endl;
	std::cout << frame.payload[ 1 ] << std::endl;

	this->processOutgoingMessages( frame );
	this->processOutgoingMessages( frame2 );
}