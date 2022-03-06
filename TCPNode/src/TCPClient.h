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
	enum class FrameCandidateStatus
	{
		incomplete,
		complete,
		complement
	};

	std::queue< network::TCPunstickedMessage > unstickedMessages;
	network::TCPincomingMessage receivedMessageRawBuffer;
	network::TCPunstickedMessage partialMessage;
	boost::asio::io_context ioContext;
	std::unique_ptr< tcp::socket > socket;
	tcp::endpoint serverEndpoint;
	std::thread TCPrunningThread;
	std::mutex incomingMessageBlock;
	bool isPartialMessageWaiting{ false };

	void startReceiving();
	void handleReceive( const boost::system::error_code& error, std::size_t bytes_transferred );
	void handleSend( const boost::system::error_code&, std::size_t );
	FrameCandidateStatus checkPartialDataJSONIntegrity( const std::string& dataPart1, const std::string& dataPart2 );
	FrameCandidateStatus checkPartialDataJSONIntegrity( const std::string& data );

	void unstickMessagesFromBuffer();

	static constexpr char bufferEmptySign{ ' ' };
	static constexpr char openingBrace{ '{' };
	static constexpr char closingBrace{ '}' };

protected:
public:
	TCPClient( const uint16_t serverPort,
	           const uint16_t clientPort,
	           const std::string& serverIpAdress,
	           const std::string& clientIpAdress );

	~TCPClient();
	void startClient();
	bool sendOutgoingMessages( std::queue< network::TCPoutgoingMessage >& msgsToSend );
	void getIncomingMessages( std::queue< network::TCPunstickedMessage >& targetContainer );
};