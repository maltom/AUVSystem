#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "CommonEnums.h"
#include "jsonCommonFunctions.h"
#include "NodeBase.h"
namespace display
{
constexpr unsigned visibleDisplayWidth = 80;
// include new line at the end
constexpr unsigned totalDisplayWidth            = 81;
constexpr unsigned totalDisplayHeight           = 24;
constexpr unsigned columnSeparatorWidth         = 1;
constexpr unsigned rowSeparatorWidth            = totalDisplayWidth - 2 * columnSeparatorWidth;
constexpr std::string_view majorColumnSeparator = "┃";
constexpr std::string_view minorColumnSeparator = "┃";
constexpr std::string_view rowSeparator         = "━";
constexpr std::string_view topLeftCorner        = "┏";
constexpr std::string_view topRightCorner       = "┓";
constexpr std::string_view bottomLeftCorner     = "┗";
constexpr std::string_view bottomRightCorner    = "┛";
constexpr std::string_view leftBorderSeparator  = "┣";
constexpr std::string_view rightBorderSeparator = "┫";
enum class rowType
{
	topBorder,
	separator,
	data,
	bottomBorder
};

} // namespace display

class Debug final : public NodeBase
{
public:
	Debug( std::shared_ptr< ros::NodeHandle >& node, configFiles::fileID configID, AUVROS::NodeIDs nID )
	    : NodeBase( node, configID, nID )
	{
		auto debugRate = jsonFunctions::ROS::readDebugRate( this->configFileID );
		debugTickSpan  = static_cast< unsigned >( rosRate / debugRate );
		subscribeTopics();
		advertiseTopics();
	}
	~Debug() = default;

protected:
private:
	void processInMainLoop() override;
	void subscribeTopics() override;
	void advertiseTopics() override;
	void connectServices() override;

	bool countAndCompareNumberOfTopics() const;

	void displayDebugInfo( bool isNumberOfSubscribedTopicsGood ) const;

	std::string createRow( display::rowType type ) const;

	ros::master::V_TopicInfo listOfTopics;
	unsigned debugTickSpan{ 0u };
};