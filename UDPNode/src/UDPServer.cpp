#include "UDPServer.h"

#include <iostream>

#include <boost/bind.hpp>

void UDPServer::startReceiving()
{
	socket->async_receive_from( boost::asio::buffer( receivedMessage ),
	                           clientEndpoint,
	                           boost::bind( &UDPServer::handleReceive,
	                                        this,
	                                        boost::asio::placeholders::error,
	                                        boost::asio::placeholders::bytes_transferred,
	                                        receivedMessage ) );
}
void UDPServer::handleReceive( const boost::system::error_code& error,
                               std::size_t bytes_transferred,
                               std::array< char, 100 > receivedMessage )
{
	std::cout << "DOSTALEM, PROSZE: ";
	for( const auto& in : receivedMessage )
	{
		std::cout << in;
	}
	std::cout << std::endl;

	this->startReceiving();
}
UDPServer::~UDPServer()
{
    this->ioContext.stop();
    UDPrunningThread.join();
}