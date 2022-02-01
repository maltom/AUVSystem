#pragma once

#include <memory>
#include <queue>
#include <string>

#include <boost/asio.hpp>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "auvConfig/CommunicationCodes.h"
#include "external/jsonxx/jsonxx.h"
#include "CommonEnums.h"
#include "NodeBase.h"
#include "ROSEnums.h"
#include "TCPClient.h"

class TCPNode final : public NodeBase
{
	struct Frame
	{
		
	};

public:
	TCPNode( std::shared_ptr< ros::NodeHandle >& node, configFiles::fileID configID, AUVROS::NodeIDs nID )
	    : NodeBase( node, configID, nID )
	{
		loadNetworkConfig();
		this->tcpClient = std::make_unique< TCPClient >( serverPort, clientPort, serverAdress, clientAdress );

		subscribeTopics();
		advertiseTopics();
	}

private:
	std::queue< network::UDPincomingMessage > incomingMessages;
	std::queue< network::UDPoutgoingMessage > outgoingMessages;

	std::unique_ptr< TCPClient > tcpClient;

	// system is the tcp client
	uint16_t serverPort;
	uint16_t clientPort;
	std::string serverAdress;
	std::string clientAdress;

	void processInMainLoop() override;
	void subscribeTopics() override;
	void advertiseTopics() override;
	void connectServices() override;

	void loadNetworkConfig();
	void processIncomingMessages();
	void processOutgoingMessages( const Frame& frameToSend );
	Frame decomposeFrame( const network::UDPincomingMessage& incMsg );
	void processCommand( const Frame& frame );
};