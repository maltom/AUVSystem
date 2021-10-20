#pragma once

#include <fstream>
#include <memory>
#include <queue>
#include <string>

#include <boost/asio.hpp>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "CommonEnums.h"
#include "auvConfig/CommunicationCodes.h"
#include "NodeBase.h"

class TCPNode final : public NodeBase
{
struct Frame
{
		Command commandCode;
		uint8_t payloadSize;
		std::array< float, network::UDPpayloadMaxSize > payload;
};

public:
	TCPNode( std::shared_ptr< ros::NodeHandle >& node, configFiles::fileID configID ) : NodeBase( node, configID )
	{
		// loadNetworkConfig();
		// subscribeTopics();
		// this->udpServer = std::make_unique< TCPServer >( this->serverPort, 53060 );
	}
	~TCPNode() = default;

	void startMainLoop() override;

// protected:
// private:
// 	std::queue< network::UDPincomingMessage > incomingMessages;
// 	std::queue< network::UDPoutgoingMessage > outgoingMessages;

// 	std::unique_ptr< TCPServer > udpServer;

// 	// jetson is the udp server
// 	uint16_t serverPort;
// 	uint16_t clientPort;

	void subscribeTopics() override;
	void advertiseTopics() const override;
	void connectServices() const override;

// 	void loadNetworkConfig();
// 	void processIncomingMessages();
// 	void processOutgoingMessages( const Frame& frameToSend );
// 	Frame decomposeFrame( const network::UDPincomingMessage& incMsg );
// 	void processCommand( const Frame& frame );
};