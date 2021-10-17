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

#include "NodeBase.h"
#include "UDPServer.h"

class UDPNode final : public NodeBase
{

public:
	UDPNode( std::shared_ptr< ros::NodeHandle >& node, std::fstream& config ) : NodeBase( node, config )
	{
		loadNetworkConfig();
		subscribeTopics();
	}
	~UDPNode() = default;

	void startMainLoop() override;

protected:
private:
	std::queue< std::string > incomingMessages;
	std::queue< std::string > outgoingMessages;

	std::unique_ptr< UDPServer> udpServer;


	//std::thread receivingThread;
	// jetson is the udp server
	uint16_t serverPort;
	uint16_t clientPort;

	void subscribeTopics() override;
	void advertiseTopics() const override;
	void connectServices() const override;

	void loadNetworkConfig();
};