#include "TrajectoryPlanner.h"

void TrajectoryPlanner::processInMainLoop() {}

void TrajectoryPlanner::subscribeTopics()
{
	// this->globalEstimatedPositionPublisher =  this->rosNode->advertise< std_msgs::Float32 >(
	// TopicsAndServicesNames::Topics::globalEstimatedPosition, 1000 );
}
void TrajectoryPlanner::advertiseTopics() {}
void TrajectoryPlanner::connectServices() {}

void TrajectoryPlanner::generateObstacleModeTrajectory( const OccupancyGrid& grid )
{
	this->occupancyGrid = grid;
}
