#pragma once
#define MAIN_CONFIG_FILE_PATH "../../auvConfig/auvConfig.json"

#include <array>
#include <string>

namespace configFiles
{
enum fileID
{
	main_file = 0
};
constexpr int configFilesCount                = 1;
static std::array< std::string, 1 > filePaths = { MAIN_CONFIG_FILE_PATH };
} // namespace configFiles

namespace network
{
constexpr int UDPbytesForCommand{ 1 };
constexpr int UDPcommandPositionFrame{ 0 };

constexpr int UDPbytesForPayloadSize{ 1 };
constexpr int UDPpayloadSizePositionFrame{ UDPcommandPositionFrame + UDPbytesForCommand };

constexpr int UDPpayloadStartPositionFrame{ UDPpayloadSizePositionFrame + UDPbytesForPayloadSize };
constexpr int UDPonePayloadWordByteSize{ sizeof( float ) };

constexpr int UDPincomingBufferMaxLength{ 256 };
constexpr int UDPpayloadMaxSize{ ( UDPincomingBufferMaxLength - UDPbytesForPayloadSize - UDPbytesForCommand )
	                             / UDPonePayloadWordByteSize };
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

namespace regulator
{
constexpr auto stateDim   = 12u;
constexpr auto controlDim = 6u;
// As we have 3 axes and 3 rotations - we have 6 space variables and sixDim is to reference 6 in places were it is tied
// i.e. matrices dimensions
constexpr auto sixDim = 6u;
} // namespace regulator