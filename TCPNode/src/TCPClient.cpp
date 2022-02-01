#include "TCPClient.h"

#include <algorithm>
#include <boost/bind.hpp>

#include "jsonCommonFunctions.h"

TCPClient::TCPClient( const uint16_t serverPort,
                      const uint16_t clientPort,
                      const std::string& serverIpAdress,
                      const std::string& clientIpAdress )
{
	// socket         = std::make_unique< udp::socket >( ioContext,
	//   udp::endpoint( address::from_string( serverIpAdress ), serverPort ) );
	socket         = std::make_unique< tcp::socket >( ioContext, tcp::endpoint( tcp::v4(), serverPort ) );
	clientEndpoint = tcp::endpoint( address::from_string( clientIpAdress ), clientPort );
}

void TCPClient::startServer()
{
	this->startReceiving();
	TCPrunningThread = std::thread( [ & ] { this->ioContext.run(); } );
}

void TCPClient::startReceiving()
{
	socket->( boost::asio::buffer( receivedMessage ),
	                            clientEndpoint,
	                            boost::bind( &TCPClient::handleReceive,
	                                         this,
	                                         boost::asio::placeholders::error,
	                                         boost::asio::placeholders::bytes_transferred,
	                                         receivedMessage ) );
}

void TCPClient::handleReceive( const boost::system::error_code& error,
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

bool TCPClient::sendOutgoingMessages( std::queue< network::UDPoutgoingMessage >& msgsToSend )
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

void TCPClient::getIncomingMessages( std::queue< network::UDPincomingMessage >& targerContainer )
{
	incomingMessageBlock.lock();
	std::swap( targerContainer, this->incomingMessages );

	incomingMessageBlock.unlock();
}

TCPClient::~TCPClient()
{
	this->ioContext.stop();
	UDPrunningThread.join();
}