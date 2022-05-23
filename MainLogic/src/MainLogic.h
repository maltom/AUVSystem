#pragma once

#include <memory>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "CommonEnums.h"
#include "NodeBase.h"
#include "ROSEnums.h"
#include "StateMachine/LogicCommonData.h"
#include "StateMachine/StateStack.h"

class MainLogic final : public NodeBase
{
public:
	MainLogic( std::shared_ptr< ros::NodeHandle >& node, configFiles::fileID configID, AUVROS::NodeIDs nID )
	    : NodeBase( node, configID, nID )
	{
		logicData    = std::make_shared< LogicCommonData >();
		stateMachine = std::make_unique< StateStack< StateBase > >( this->logicData );

		subscribeTopics();
		advertiseTopics();
	}
	~MainLogic() = default;

protected:
	void processInMainLoop() override;
	void subscribeTopics() override;
	void advertiseTopics() override;
	void connectServices() override;

private:
	enum PublishersCodes
	{
		stateName
	};

	void popState( const AUVROS::MessageTypes::States& state );
	void popToFundamental( const AUVROS::MessageTypes::States& state );
	void nextTask( const AUVROS::MessageTypes::States& state );
	void pushEmergency( const AUVROS::MessageTypes::States& state );
	void pushMission( const AUVROS::MessageTypes::States& state );
	void pushSpecificState( const AUVROS::MessageTypes::States& state );

	std::shared_ptr< LogicCommonData > logicData;
	std::unique_ptr< StateStack< StateBase > > stateMachine;

	void globalEstimatedPositionObtained( const geometry_msgs::Twist& position );
};