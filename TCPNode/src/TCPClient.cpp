#include "TCPClient.h"

#include <algorithm>
#include <boost/bind.hpp>
#include <chrono>

#include "jsonCommonFunctions.h"

TCPClient::TCPClient( const uint16_t serverPort,
                      const uint16_t clientPort,
                      const std::string& serverIpAdress,
                      const std::string& clientIpAdress )
{
	// socket = std::make_unique< tcp::socket >( ioContext,
	//   tcp::endpoint( address::from_string( clientIpAdress ), clientPort ) );
	socket = std::make_unique< tcp::socket >( ioContext, tcp::endpoint( tcp::v4(), clientPort ) );
	socket->close();
	serverEndpoint = tcp::endpoint( address::from_string( serverIpAdress ), serverPort );

	this->receivedMessage.fill( TCPClient::bufferEmptySign );
}

void TCPClient::startClient()
{
	boost::system::error_code error = boost::asio::error::host_not_found;
	while( !socket->is_open() || error )
	{
		socket->connect( serverEndpoint, error );
		std::this_thread::sleep_for( std::chrono::milliseconds( 1000 ) );
	}
	this->startReceiving();
	TCPrunningThread = std::thread( [ & ] { this->ioContext.run(); } );
}

void TCPClient::startReceiving()
{

	socket->async_read_some ( boost::asio::buffer( receivedMessage ),
	             boost::bind( &TCPClient::handleReceive,
	                          this,
	                          boost::asio::placeholders::error,
	                          boost::asio::placeholders::bytes_transferred ) );
}

void TCPClient::handleReceive( const boost::system::error_code& error, std::size_t bytes_transferred )
{
	std::thread receiveThread(
	    [ & ]
	    {
		    incomingMessageBlock.lock();
		    std::cout << bytes_transferred << std::endl;

		    std::cout << std::string( this->receivedMessage.begin(), this->receivedMessage.end() ) << std::endl
		              << std::endl
		              << std::endl;
		    incomingMessages.push( this->receivedMessage );
		    network::TCPincomingMessage empty;
		    empty.fill( TCPClient::bufferEmptySign );

		    std::swap( this->receivedMessage, empty );
		    incomingMessageBlock.unlock();
		    this->startReceiving();
	    } );
	receiveThread.join();
}

void TCPClient::handleSend( const boost::system::error_code&, std::size_t ) {}

bool TCPClient::sendOutgoingMessages( std::queue< network::TCPoutgoingMessage >& msgsToSend )
{
	while( !msgsToSend.empty() )
	{
		auto message = msgsToSend.front();
		socket->async_write_some( boost::asio::buffer( message ),
		                          boost::bind( &TCPClient::handleSend,
		                                       this,
		                                       boost::asio::placeholders::error,
		                                       boost::asio::placeholders::bytes_transferred ) );
		msgsToSend.pop();
	}
	return true;
}

void TCPClient::getIncomingMessages( std::queue< network::TCPincomingMessage >& targetContainer )
{
	incomingMessageBlock.lock();
	std::swap( targetContainer, this->incomingMessages );

	incomingMessageBlock.unlock();
}

TCPClient::~TCPClient()
{
	socket->close();
	this->ioContext.stop();
	TCPrunningThread.join();
}