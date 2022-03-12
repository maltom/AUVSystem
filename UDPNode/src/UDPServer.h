#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>

#include "CommonEnums.h"

using boost::asio::ip::address;
using boost::asio::ip::udp;

class UDPServer final
{
private:
	std::queue< network::UDPincomingMessage > incomingMessages;
	boost::asio::io_context ioContext;
	std::unique_ptr< udp::socket > socket;
	udp::endpoint clientEndpoint;
	network::UDPincomingMessage receivedMessage;
	std::thread UDPrunningThread;
	std::mutex incomingMessageBlock;
	void startReceiving();
	void handleReceive( const boost::system::error_code& error, std::size_t bytes_transferred );
	void handleSend( std::string, const boost::system::error_code&, std::size_t ) {}

protected:
public:
	UDPServer( const uint16_t serverPort,
	           const uint16_t clientPort,
	           const std::string& serverIpAdress,
	           const std::string& clientIpAdress );

	~UDPServer();
	void startServer();
	bool sendOutgoingMessages( std::queue< network::UDPoutgoingMessage >& msgsToSend );
	void getIncomingMessages( std::queue< network::UDPincomingMessage >& targetContainer );
};