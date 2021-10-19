#pragma once
#define CONFIG_FILE_PATH "../../../auvConfig/auvConfig.json"

#include <array>
#include <string>

namespace network
{
constexpr int UDPbytesForCommand{ 1 };
constexpr int UDPcommandPositionFrame{ 0 };

constexpr int UDPbytesForPayloadSize{ 1 };
constexpr int UDPpayloadSizePositionFrame{ UDPcommandPositionFrame + UDPbytesForCommand };

constexpr int UDPpayloadStartPositionFrame{ UDPpayloadSizePositionFrame + UDPbytesForPayloadSize };
constexpr int UDPonePayloadWordByteSize{ sizeof( float ) };

constexpr int UDPincomingBufferMaxLength{ 256 };
constexpr int UDPpayloadMaxSize{ ( UDPincomingBufferMaxLength - UDPbytesForPayloadSize - UDPbytesForCommand ) / UDPonePayloadWordByteSize };
using UDPincomingMessage = std::array< unsigned char, UDPincomingBufferMaxLength >;
using UDPoutgoingMessage = std::string;
enum Device
{
	jetson          = -1,
	microcontroller = 0,
	DVL,
	devPC
};

} // namespace network