#include "UDPNode.h"

#include <array>
#include <cstdlib>
#include <exception>
#include <iostream>

#include "CommonEnums.h"
#include "ROSEnums.h"

void UDPNode::startMainLoop()
{
	udpServer->startServer();

	while( ros::ok() )
	{
		ros::spinOnce();

		udpServer->sendOutgoingMessages( outgoingMessages );
		udpServer->getIncomingMessages( incomingMessages );
		this->processIncomingMessages();
		rosLoopRate->sleep();
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
	// std::cout << "KOMENDA NR: " << frame.commandCode << "\n";
	// std::cout << "ROZMIAR: " << frame.payloadSize << "\n";
	// for( int i = 0; i < frame.payloadSize; ++i )
	// {
	// 	std::cout << "|" << frame.payload[ i ];
	// }
	// std::cout << "\n";
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
		// going from end to beginning because of little endian
		for( auto it = byteArray.rbegin(); it != byteArray.rend(); ++it )
		{
			message.push_back( *it );
		}
	}
	this->outgoingMessages.push( message );
}