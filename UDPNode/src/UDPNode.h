#pragma once

#include <fstream>
#include <memory>
#include <queue>
#include <string>

#include <boost/asio.hpp>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "jsonxx/jsonxx.h"

#include "auvConfig/CommunicationCodes.h"
#include "NodeBase.h"
#include "UDPServer.h"

class UDPNode final : public NodeBase
{
	struct Frame
	{
		Command commandCode;
		uint8_t payloadSize;
		std::array< float, network::UDPpayloadMaxSize > payload;
	};

public:
	UDPNode( std::shared_ptr< ros::NodeHandle >& node, std::fstream& config ) : NodeBase( node, config )
	{
		loadNetworkConfig();
		subscribeTopics();
		this->udpServer = std::make_unique< UDPServer >( this->serverPort, 53060 );
	}
	~UDPNode() = default;

	void startMainLoop() override;

protected:
private:
	std::queue< network::UDPincomingMessage > incomingMessages;
	std::queue< network::UDPoutgoingMessage > outgoingMessages;

	std::unique_ptr< UDPServer > udpServer;

	// jetson is the udp server
	uint16_t serverPort;
	uint16_t clientPort;

	void subscribeTopics() override;
	void advertiseTopics() const override;
	void connectServices() const override;

	void loadNetworkConfig();
	void processIncomingMessages();
	Frame decomposeFrame( const network::UDPincomingMessage& incMsg );
	void processFrame( const Frame& frame );
};