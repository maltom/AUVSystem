#pragma once

#include <memory>
#include <queue>
#include <string>

#include <boost/asio.hpp>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "auvConfig/CommunicationCodes.h"
#include "CommonEnums.h"
#include "NodeBase.h"
#include "ROSEnums.h"
#include "UDPServer.h"

class UDPNode final : public NodeBase
{
	struct Frame
	{
		uint8_t commandCode;
		uint8_t payloadSize;
		std::array< network::payloadWordType, network::UDPpayloadMaxSize > payload;
	};

public:
	UDPNode( std::shared_ptr< ros::NodeHandle >& node, configFiles::fileID configID, AUVROS::NodeIDs nID )
	    : NodeBase( node, configID, nID )
	{
		loadNetworkConfig();
		this->udpServer = std::make_unique< UDPServer >( serverPort, clientPort, serverAdress, clientAdress );

		subscribeTopics();
		advertiseTopics();
		this->udpServer->startServer();
	}
	~UDPNode() = default;

private:
	std::queue< network::UDPincomingMessage > incomingMessages;
	std::queue< network::UDPoutgoingMessage > outgoingMessages;

	std::unique_ptr< UDPServer > udpServer;

	// system is the udp server
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

	void sendThrustersSignalToMicroController( const AUVROS::MessageTypes::ThrustersSignal& message );
	void sendServosSignalToMicroController( const AUVROS::MessageTypes::ServosSignal& mesagge );
	void sendLaunchTorpedoSignalToMicroController( const AUVROS::MessageTypes::Torpedo& mesagge );
};