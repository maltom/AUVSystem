#pragma once

#include <array>
#include <string>

namespace network
{

constexpr int UDPincomingBufferMaxLength{ 256 };
using UDPincomingMessage = std::array< char, UDPincomingBufferMaxLength >;
using UDPoutgoingMessage = std::string;
enum Device
{
	jetson          = -1,
	microcontroller = 0,
	DVL,
	devPC
};

} // namespace network