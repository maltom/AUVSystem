#pragma once

#include <array>
#include <string>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

namespace AUVROS
{
namespace Folders
{
	static const std::string MainFolder               = "/AUVInternalSystem";
	static const std::string MainFolderShort          = "/AUVInt";
	static const std::string PositionSubFolder        = MainFolder + "/Position";
	static const std::string CameraSubFolder          = MainFolder + "/Camera";
	static const std::string HardwareSignalsSubFolder = MainFolder + "/HardwareSignals";
	static const std::string DevPCSubFolder           = MainFolder + "/DevPC";
	static const std::string TorpedoSubFolder         = MainFolder + "/Torpedo";
	static const std::string HealthSubFolder          = MainFolder + "/Health";
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
		static const std::string DVLDRTimeStamp       = Folders::HardwareSignalsSubFolder + "/DVLDRTimeStamp";
	} // namespace HardwareSignals

	namespace DevPC
	{
		static const std::string arbitrarlySetThrusters      = Folders::DevPCSubFolder + "/arbitrarlySetThrusters";
		static const std::string arbitrarlySetServos         = Folders::DevPCSubFolder + "/arbitrarlySetServos";
		static const std::string arbitrarlySetGlobalPosition = Folders::DevPCSubFolder + "/arbitrarlySetGlobalPosition";
		static const std::string arbitrarlySetThrust         = Folders::DevPCSubFolder + "/arbitrarlySetThrust";

		static const std::string arbitrarlySetRelativePosition
		    = Folders::DevPCSubFolder + "/arbitrarlySetRelativePosition";
	} // namespace DevPC

	namespace Health
	{
		static const std::string healthReportSingleNode = Folders::HealthSubFolder + "/healthReportSingleNode";
		static const std::string healthHardwareReport   = Folders::HealthSubFolder + "/healthHardwareReport";

		static const std::string healthReportSummary = Folders::HealthSubFolder + "/healthReportSummary";
		static const std::string healthHardwareReportSummary
		    = Folders::HealthSubFolder + "/healthHardwareReportSummary";
	} // namespace Health

} // namespace Topics

namespace MessageTypes
{
	using ArbitrarlySetThrust = geometry_msgs::Twist;
	using ThrustersSignal     = std_msgs::Float32MultiArray;
	using ServosSignal        = std_msgs::Float32MultiArray;
	using Position            = geometry_msgs::Twist;
	using Waypoint            = geometry_msgs::Twist;

	using DVLDeadReckoning = std_msgs::Float32MultiArray;
	using DVLVelocity      = geometry_msgs::Twist;
	using DVLDRTimeStamp   = std_msgs::Float64;
	// using OccupancyGrid
	// this type depends on number of nodes. max number of nodes for Int32 is 32, can be increased to Int64
	using HealthReport = std_msgs::Int32;
} // namespace MessageTypes

namespace QueueSize
{
	static constexpr auto StandardQueueSize     = 1000;
	static constexpr auto HealthQueueSize       = 10;
	static constexpr auto DVLQueueSize          = 50;
	static constexpr auto GlobalHealthQueueSize = 100;

	namespace Images
	{

	}
} // namespace QueueSize

// make sure that names in NodeIDs and NodeNames match
enum NodeIDs
{
	MainLogic = 0,
	HealthCheck,
	UDPNode,
	TCPNode,
	Debug,
	ThrusterRegulator,
	// TrajectoryPlanner,
	// AdditionalCameraProcessor,
	PositioningSLAM,

	Last = PositioningSLAM,
	Count
};

static const std::array< std::string, NodeIDs::Count > NodeNames{
	"MainLogic",       "HealthCheck", "UDPNode",
	"TCPNode",         "Debug",       "ThrusterRegulator", //"TrajectoryPlanner", //"AdditionalCameraProcessor",
	"PositioningSLAM",
};

} // namespace AUVROS