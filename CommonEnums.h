#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <utility>

namespace configFiles
{
enum fileID
{
	main_file = 0,
	mission_planner
};

constexpr std::string_view MAIN_CONFIG_FILE_PATH{ "../../auvConfig/auvConfig.json" };
constexpr std::string_view MISSION_PLANNER_FILE_PATH{ "../../auvConfig/missionPlanner.json" };

constexpr int configFilesCount = 2;
const static std::array< std::string, configFilesCount > filePaths
    = { std::string( MAIN_CONFIG_FILE_PATH ), std::string( MISSION_PLANNER_FILE_PATH ) };
} // namespace configFiles

namespace math
{
constexpr double piNumber{ 3.141592 };
}

namespace network
{
using payloadWordType = uint16_t;

constexpr auto UDPbytesForCommand{ 1u };
constexpr auto UDPcommandPositionFrame{ 0u };

constexpr auto UDPbytesForPayloadSize{ 1u };
constexpr auto UDPpayloadSizePositionFrame{ UDPcommandPositionFrame + UDPbytesForCommand };

constexpr auto UDPpayloadStartPositionFrame{ UDPpayloadSizePositionFrame + UDPbytesForPayloadSize };
constexpr auto UDPonePayloadWordByteSize{ sizeof( payloadWordType ) };

constexpr auto UDPincomingBufferMaxLength{ 256u };
constexpr auto UDPpayloadMaxSize{ ( UDPincomingBufferMaxLength - UDPbytesForPayloadSize - UDPbytesForCommand )
	                              / UDPonePayloadWordByteSize };
using UDPincomingMessage = std::array< unsigned char, UDPincomingBufferMaxLength >;
using UDPoutgoingMessage = std::string;

constexpr auto TCPincomingBufferMaxLength{ 2048u };

using TCPincomingMessage  = std::array< char, TCPincomingBufferMaxLength >;
using TCPoutgoingMessage  = std::string;
using TCPunstickedMessage = std::string;

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
// As we have 3 axes and 3 rotations - we have 6 space variables and sixDim is to reference 6 in places were it is
// tied i.e. matrices dimensions
constexpr auto sixDim = 6u;

constexpr std::pair< float, float > regulatorOutputSignalMinMax{ -1.0f, 1.0f };
constexpr std::pair< float, float > regulatorServoOutputSignalMinMax{ 0.0f, math::piNumber };

} // namespace regulator

enum dimensionsIndex
{
	x = 0,
	y,
	z,
	roll,
	pitch,
	yaw
};

namespace hardware
{
constexpr std::pair< unsigned, unsigned > motorTorqueMinMax{ 0u, 2000u };
constexpr std::pair< unsigned, unsigned > servoMinMax{ 0u, 100u };

}; // namespace hardware