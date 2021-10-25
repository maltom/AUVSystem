#pragma once

#include <string>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>

namespace AUVROS
{
namespace Folders
{
static const std::string MainFolder               = "/AUVInternalSystem";
static const std::string PositionSubFolder        = MainFolder + "/Position";
static const std::string CameraSubFolder          = MainFolder + "/Camera";
static const std::string HardwareSignalsSubFolder = MainFolder + "/HardwareSignals";
static const std::string DevPC                    = MainFolder + "/DevPC";
static const std::string TorpedoSubFolder         = MainFolder + "/Torpedo";
} // namespace Folders
namespace Topics
{
namespace Positions
{
static const std::string globalEstimatedPosition = Folders::PositionSubFolder + "/globalEstimatedPosition";
static const std::string currentPosition         = Folders::PositionSubFolder + "/currentPosition";
} // namespace Positions

namespace HardwareSignals
{
static const std::string signalToThrusters  = Folders::HardwareSignalsSubFolder + "/signalToThrusters";
static const std::string signalToServos     = Folders::HardwareSignalsSubFolder + "/signalToServos";
static const std::string pressureSensorData = Folders::HardwareSignalsSubFolder + "/pressureSensorData";

static const std::string DVLVelocityData      = Folders::HardwareSignalsSubFolder + "/DVLVelocityData";
static const std::string DVLDeadReckoningData = Folders::HardwareSignalsSubFolder + "/DVLDeadReckoningData";
} // namespace HardwareSignals

namespace DevPC
{
static const std::string arbitrarlySetThrusters        = Folders::DevPC + "/arbitrarlySetThrusters";
static const std::string arbitrarlySetServos           = Folders::DevPC + "/arbitrarlySetServos";
static const std::string arbitrarlySetGlobalPosition   = Folders::DevPC + "/arbitrarlySetGlobalPosition";
static const std::string arbitrarlySetRelativePosition = Folders::DevPC + "/arbitrarlySetRelativePosition";
} // namespace DevPC

} // namespace Topics

namespace MessageTypes
{
using ThrustersSignal = std_msgs::Float32MultiArray;
using ServosSignal    = std_msgs::Float32MultiArray;
using Position        = geometry_msgs::Twist;
} // namespace MessageTypes

namespace QueueSize
{

static constexpr auto StandardQueueSize = 1000;

namespace Images
{

}
} // namespace QueueSize
} // namespace AUVROS