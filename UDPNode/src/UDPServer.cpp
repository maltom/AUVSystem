#include "UDPServer.h"

#include <algorithm>
#include <boost/bind.hpp>

#include "jsonCommonFunctions.h"

UDPServer::UDPServer( const uint16_t serverPort,
                      const uint16_t clientPort,
                      const std::string& serverIpAdress,
                      const std::string& clientIpAdress )
{
	// socket         = std::make_unique< udp::socket >( ioContext,
	//   udp::endpoint( address::from_string( serverIpAdress ), serverPort ) );
	socket         = std::make_unique< udp::socket >( ioContext, udp::endpoint( udp::v4(), serverPort ) );
	clientEndpoint = udp::endpoint( address::from_string( clientIpAdress ), clientPort );
}

void UDPServer::startServer()
{
	this->startReceiving();
	UDPrunningThread = std::thread( [ & ] { this->ioContext.run(); } );
}

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
                               network::UDPincomingMessage receivedMessage )
{
	std::thread receiveThread(
	    [ & ]
	    {
		    incomingMessageBlock.lock();

		    incomingMessages.push( receivedMessage );
		    network::UDPincomingMessage empty;

		    std::swap( this->receivedMessage, empty );
		    incomingMessageBlock.unlock();
		    this->startReceiving();
	    } );
	receiveThread.join();
}

bool UDPServer::sendOutgoingMessages( std::queue< network::UDPoutgoingMessage >& msgsToSend )
{
	while( !msgsToSend.empty() )
	{
		auto message = msgsToSend.front();
		socket->async_send_to( boost::asio::buffer( message ),
		                       clientEndpoint,
		                       boost::bind( &UDPServer::handleSend,
		                                    this,
		                                    message,
		                                    boost::asio::placeholders::error,
		                                    boost::asio::placeholders::bytes_transferred ) );

		msgsToSend.pop();
	}
	return true;
}

void UDPServer::getIncomingMessages( std::queue< network::UDPincomingMessage >& targerContainer )
{
	incomingMessageBlock.lock();
	std::swap( targerContainer, this->incomingMessages );

	incomingMessageBlock.unlock();
}

UDPServer::~UDPServer()
{
	this->ioContext.stop();
	UDPrunningThread.join();
}