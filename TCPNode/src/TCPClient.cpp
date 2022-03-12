#include "TCPClient.h"

#include <algorithm>
#include <boost/bind.hpp>
#include <chrono>
#include <exception>
#include <iostream>

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

	this->receivedMessageRawBuffer.fill( TCPClient::bufferEmptySign );
	this->partialMessage.reserve( network::TCPincomingBufferMaxLength );
	this->receivedMessageRawBuffer.fill( TCPClient::bufferEmptySign );
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

	socket->async_read_some( boost::asio::buffer( receivedMessageRawBuffer ),
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
		    // std::cout << bytes_transferred << std::endl;

		    // std::cout << std::string( this->receivedMessageRawBuffer.begin(), this->receivedMessageRawBuffer.end() )
		    // << std::endl
		    //           << std::endl
		    //           << std::endl;

		    unstickMessagesFromBuffer();

		    // unstickedMessages.push( this->receivedMessageRawBuffer );
		    network::TCPincomingMessage empty;
		    empty.fill( TCPClient::bufferEmptySign );

		    std::swap( this->receivedMessageRawBuffer, empty );
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

void TCPClient::getIncomingMessages( std::queue< network::TCPunstickedMessage >& targetContainer )
{
	incomingMessageBlock.lock();
	std::swap( targetContainer, this->unstickedMessages );

	incomingMessageBlock.unlock();
}

TCPClient::FrameCandidateStatus TCPClient::checkPartialDataJSONIntegrity( const std::string& dataPart1,
                                                                          const std::string& dataPart2 )
{
	return checkPartialDataJSONIntegrity( dataPart1 + dataPart2 );
}

TCPClient::FrameCandidateStatus TCPClient::checkPartialDataJSONIntegrity( const std::string& data )
{
	auto braceCounter{ 0 };
	for( const auto& in : data )
	{
		if( in == openingBrace )
		{
			++braceCounter;
		}
		else if( in == closingBrace )
		{
			--braceCounter;
		}
	}

	if( braceCounter > 0 )
		return FrameCandidateStatus::incomplete;
	else if( braceCounter == 0 )
		return FrameCandidateStatus::complete;
	else
		return FrameCandidateStatus::complement;
}

void TCPClient::unstickMessagesFromBuffer()
{
	auto startPoint = receivedMessageRawBuffer.begin();

	for( auto it = receivedMessageRawBuffer.begin();; ++it )
	{
		if( it == receivedMessageRawBuffer.end() || *it == bufferEmptySign )
		{
			const std::string frameCandidate( startPoint, it );
			if( frameCandidate.empty() )
			{
				return;
			}

			const auto completness = checkPartialDataJSONIntegrity( frameCandidate );

			if( completness == FrameCandidateStatus::complete )
			{
				unstickedMessages.push( this->partialMessage );
			}
			else if( completness == FrameCandidateStatus::incomplete )
			{
				if( !isPartialMessageWaiting )
				{
					this->partialMessage          = frameCandidate;
					this->isPartialMessageWaiting = true;
				}
				else
				{
					this->partialMessage += frameCandidate;
				}
			}
			else if( completness == FrameCandidateStatus::complement )
			{
				const auto& potentialComplement = frameCandidate;

				if( isPartialMessageWaiting )
				{
					const auto merging = checkPartialDataJSONIntegrity( this->partialMessage, potentialComplement );

					if( merging == FrameCandidateStatus::complete )
					{
						unstickedMessages.push( this->partialMessage );
					}
					else if( merging == FrameCandidateStatus::incomplete )
					{
						this->partialMessage += potentialComplement;
					}
					else if( merging == FrameCandidateStatus::complement )
					{
						throw std::runtime_error( "Overcompleted awaiting frame." );
					}
				}
				else
				{
					throw std::runtime_error( "Partial message lost first half." );
				}
			}
			return;
		}
		else if( *it == '\r' )
		{
			const std::string frameCandidate( startPoint, it );
			startPoint = it + 1;
			if( frameCandidate.empty() )
			{
				continue;
			}

			const auto completness = checkPartialDataJSONIntegrity( frameCandidate );

			if( completness == FrameCandidateStatus::complete )
			{
				unstickedMessages.push( frameCandidate );
			}
			else if( completness == FrameCandidateStatus::incomplete )
			{
				throw std::runtime_error( "Corrupted data." );
			}
			else if( completness == FrameCandidateStatus::complement )
			{
				const auto& potentialComplement = frameCandidate;

				if( isPartialMessageWaiting )
				{
					auto merging = checkPartialDataJSONIntegrity( this->partialMessage, potentialComplement );

					if( merging == FrameCandidateStatus::complete )
					{
						unstickedMessages.push( this->partialMessage + potentialComplement );
						this->partialMessage.clear();
						isPartialMessageWaiting = false;
					}
					else if( merging == FrameCandidateStatus::incomplete )
					{
						throw std::runtime_error( "Corrupted data." );
					}
					else if( merging == FrameCandidateStatus::complement )
					{
						throw std::runtime_error( "Overcompleted awaiting frame" );
					}
				}
				else
				{
					throw std::runtime_error( "Partial message lost first half." );
				}
			}
		}
		else if( *it == '\n' )
		{
			startPoint = it + 1;
		}
	}
}

TCPClient::~TCPClient()
{
	socket->close();
	this->ioContext.stop();
	TCPrunningThread.join();
}