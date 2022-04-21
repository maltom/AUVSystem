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
#include "jsonTCPFunctions.h"
#include "NodeBase.h"
#include "ROSEnums.h"
#include "TCPClient.h"

class TCPNode final : public NodeBase
{
public:
	TCPNode( std::shared_ptr< ros::NodeHandle >& node, configFiles::fileID configID, AUVROS::NodeIDs nID )
	    : NodeBase( node, configID, nID )
	{
		loadNetworkConfig();
		this->tcpClient = std::make_unique< TCPClient >( serverPort, clientPort, serverAdress, clientAdress );

		subscribeTopics();
		advertiseTopics();
#ifndef NODVL
		tcpClient->startClient();
		this->resetDeadReckoning();

#endif
	}

private:
	enum PublishersCodes
	{
		DVLDeadReckoning = 0,
		DVLVelocity,
		DVLDRTimeStamp
	};

	std::queue< network::TCPunstickedMessage > incomingMessages;
	std::queue< network::TCPoutgoingMessage > outgoingMessages;

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
	Frame decomposeFrame( const network::TCPunstickedMessage& incMsg );
	void processCommand( const Frame& frame );
	void resetDeadReckoning();
};