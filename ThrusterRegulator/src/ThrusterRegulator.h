#pragma once
#include <memory>

#include <ct/optcon/optcon.h>
#include <Eigen/Dense>

#include "CommonEnums.h"
#include "NodeBase.h"
#include "LQRRegulator.h"
#include "ROSEnums.h"
#include "VehiclePhysicalModel.h"

using namespace Eigen;
using namespace regulator;

void allocateThrust2Azimuthal( VectorXd& thrustSignal_u,
                               const VectorXd& desiredForces_tau,
                               VehiclePhysicalModel& model,
                               const AllocationPenalizers& penalizers );

class ThrusterRegulator final : public NodeBase
{
public:
	ThrusterRegulator( std::shared_ptr< ros::NodeHandle >& node, configFiles::fileID configID, AUVROS::NodeIDs nID )
	    : NodeBase( node, configID, nID ), model( configID )
	{
		loadRegulatorParameters( this->configFileID );
		model.adjustParametersForWorkingFrequency( this->regulatorWorkingFrequency );
		subscribeTopics();
		advertiseTopics();
	}
	~ThrusterRegulator() {}

protected:
private:
	enum advertisers
	{
		signalToThrusters,
		signalToServos,

#ifdef SIMULATION
		estimatedPosition,
#endif
#ifdef NOLQR
		forcesGenerated,
#endif
	};

	VehiclePhysicalModel model;
	LQRRegulator lqrRegulator;
	AllocationPenalizers penalizers;

	// VectorXd::Zero( thrusterAmount, 1 ). u is vector of -1 to 1 values of how each thruster is working
	VectorXd thrustValues_u            = VectorXd::Zero( 5 );
	VectorXd currentState              = VectorXd::Zero( stateDim ); // state = position and speed
	VectorXd currentPosition           = VectorXd::Zero( sixDim );
	VectorXd currentSpeed              = VectorXd::Zero( sixDim );
	VectorXd positionToReach           = VectorXd::Ones( sixDim );
	VectorXd regulatorFeedbackPosition = VectorXd::Zero( sixDim );
	VectorXd simulationResultState     = VectorXd::Zero( stateDim );

	// VectorXd dummyForces       = VectorXd::Zero( 6 );
	// VectorXd dummyThrustSignal = VectorXd::Zero( 5 );
	double timeStamp{ 0.0 };
	float regulatorWorkingFrequency{ 10.0f };
	unsigned regulatorTickSpan;
	void processInMainLoop() override;
	void subscribeTopics() override;
	void advertiseTopics() override;
	void connectServices() override;
	void publishSignalsToHardware();
	void loadRegulatorParameters( configFiles::fileID configID );

	// TODO: change to SLAM
	void updateCurrentPositionAndAngularSpeed( const AUVROS::MessageTypes::DVLDeadReckoning& newPosition );
	void updateVelocity( const AUVROS::MessageTypes::DVLVelocity& newVelocity );

#ifdef SIMULATION
	void calculateSimulationState();
#ifndef NOLQR
	void publishEstimatedPosition();

#else
	void updateDesiredForcesError( const AUVROS::MessageTypes::arbitrarlySetThrustForce& newForces );
#endif
#endif
};