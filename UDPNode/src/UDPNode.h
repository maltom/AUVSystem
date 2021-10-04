#pragma once

#include <fstream>
#include <memory>

#include <boost/asio.hpp>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "jsonxx/jsonxx.h"

#include "NodeBase.h"

using boost::asio::ip::udp;

class UDPNode final : public NodeBase
{
	enum Targets
	{
		jetson = 0,
		dvl,
	};
public:
	UDPNode( std::shared_ptr< ros::NodeHandle >& node, std::fstream& config ) : NodeBase(node , config )
	{
		socket = std::make_shared< udp::socket >( io_context );

		subscribeTopics();
	}
	~UDPNode() {}

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