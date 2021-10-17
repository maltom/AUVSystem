#pragma once

#include <array>
#include <memory>
#include <thread>

#include <boost/asio.hpp>

using boost::asio::ip::udp;

class UDPServer final
{
private:
	boost::asio::io_context ioContext;
	std::unique_ptr< udp::socket > socket;
	udp::endpoint clientEndpoint;
	std::array< char, 100 > receivedMessage;
	std::thread UDPrunningThread;

	void startReceiving();
	void handleReceive( const boost::system::error_code& error,
	                    std::size_t bytes_transferred,
	                    std::array< char, 100 > receivedMessage );

protected:
public:
	UDPServer( const uint16_t serverPort )
	{
		socket = std::make_unique< udp::socket >( ioContext, udp::endpoint( udp::v4(), serverPort ) );
		this->startReceiving();

		UDPrunningThread = std::thread( [ & ] { this->ioContext.run(); } );
	}

	~UDPServer();
};