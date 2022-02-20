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
using boost::asio::ip::tcp;

class TCPClient final
{
private:
	std::queue< network::TCPincomingMessage > incomingMessages; 
	boost::asio::io_context ioContext;
	std::unique_ptr< tcp::socket > socket;
	tcp::endpoint clientEndpoint;
	std::thread TCPrunningThread;
	std::mutex incomingMessageBlock;
	void startReceiving();
	void handleReceive( const boost::system::error_code& error,
	                    std::size_t bytes_transferred,
	                    network::UDPincomingMessage receivedMessage );

protected:
public:
	TCPClient( const uint16_t serverPort,
	           const uint16_t clientPort,
	           const std::string& serverIpAdress,
	           const std::string& clientIpAdress );

	~TCPClient();
	void startServer();
	bool sendOutgoingMessages( std::queue< network::UDPoutgoingMessage >& msgsToSend );
	void getIncomingMessages( std::queue< network::UDPincomingMessage >& targerContainer );
};