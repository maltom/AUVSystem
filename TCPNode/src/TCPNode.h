#pragma once

#include <fstream>
#include <memory>

#include <boost/asio.hpp>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "jsonxx/jsonxx.h"

#include "CommonEnums.h"
#include "NodeBase.h"

using boost::asio::ip::udp;

class TCPNode final : public NodeBase
{
public:
	TCPNode( std::shared_ptr< ros::NodeHandle >& node, std::fstream& config ) : NodeBase( node, config )
	{
		socket = std::make_shared< udp::socket >( io_context );

		subscribeTopics();
	}
	~TCPNode() {}

	void startMainLoop() const override;

protected:
private:
	boost::asio::io_context io_context;
	std::shared_ptr< udp::socket > socket;

	void subscribeTopics() override;
	void advertiseTopics() const override;
	void connectServices() const override;

	void loadNetworkConfig();
};