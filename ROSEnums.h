#pragma once

#include <string>

namespace TopicsAndServicesNames
{
namespace Folders
{
static const std::string MainFolder        = "/AUVInternalSystem";
static const std::string PositionSubFolder = MainFolder + "/Position";
static const std::string CameraSubFolder   = MainFolder + "/Camera";
static const std::string TorpedoSubFolder  = MainFolder + "/Torpedo";
} // namespace Folders
namespace Topics
{
static const std::string globalEstimatedPosition = Folders::PositionSubFolder + "/globalEstimatedPosition";
}

} // namespace TopicsAndServicesNames

namespace QueueSize
{
namespace SimpleMessages
{
static constexpr auto StandardQueueSize = 1000;
}
} // namespace QueueSize