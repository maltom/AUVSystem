#pragma once

#include <memory>
#include <queue>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "CommonEnums.h"
#include "NodeBase.h"
#include "ROSEnums.h"

// Dummy declarations for unknown types - these will be declared in ROSEnums, not here
using OccupancyGrid  = int; // OccupancyGrid is to become a custom message type
using GenerationMode = int; // three variants - 1. long straights (pinger, "banana" marker), 2. path between
                            // obstacles, 3. precise steering to reach the object of interest

class TrajectoryPlanner final : public NodeBase
{
	enum class WaypointType
	{
		first,
		intermediate,
		target
	};

public:
	TrajectoryPlanner( std::shared_ptr< ros::NodeHandle >& node, configFiles::fileID configID, AUVROS::NodeIDs nID )
	    : NodeBase( node, configID, nID )
	{
		subscribeTopics();
		advertiseTopics();
	}
	~TrajectoryPlanner() {}

protected:
private:
	void processInMainLoop() override;
	void subscribeTopics() override;
	void advertiseTopics() override;
	void connectServices() override;

	void generateObstacleModeTrajectory( const OccupancyGrid& grid ); // for ros, receive argument from caller
	void generatePreciseTrajectory( const AUVROS::MessageTypes::Position& target ); // same
	void generateStraightTrajectory( const double azimuth );                        // same

	void generatePath( const GenerationMode mode );

	void checkCurrentPathForCollisions();
	void switchPublisherToNextWaypoint(
	    bool shouldSwitch ); // method to call when thruster sends "signal" on topic to change -
	                         // with argument because you can't send an empty message

	void notifyTargetWaypointReached();   // signalizing to other nodes
	void publishCurrentWaypointToReach(); // publishing Waypoint in meters

	OccupancyGrid occupancyGrid;	// local occupancy grid is needed to mark waypoints inside
	std::queue< AUVROS::MessageTypes::Waypoint > plannedPath;
	AUVROS::MessageTypes::Position currentPosition;
};