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
MatrixXd calculateNbar( const Matrix< double, stateDim, stateDim >& A,
                        const Matrix< double, stateDim, controlDim >& B,
                        const Matrix< double, controlDim, stateDim >& K );
Matrix< double, stateDim, stateDim > calculateAStateMatrix( const VectorXd& currentState,
                                                            const VehiclePhysicalModel& model );
Matrix< double, stateDim, controlDim > calculateBStateMatrix( const VehiclePhysicalModel& model );

class ThrusterRegulator final : public NodeBase
{
public:
	ThrusterRegulator( std::shared_ptr< ros::NodeHandle >& node, configFiles::fileID configID, AUVROS::NodeIDs nID )
	    : NodeBase( node, configID, nID ), model( configID )
	{
		loadRegulatorParameters( this->configFileID );
		subscribeTopics();
		advertiseTopics();

		// dummyForces << 20.0, 0.0, 20.0, 0.0, 0.0, 0.0;
		// dummyThrustSignal << 0.0, 0.0, 0.0, 0.0, 0.0;
	}
	~ThrusterRegulator() {}

protected:
private:
	enum advertisers
	{
		signalToThrusters,
		signalToServos,
		thrustersArbitrarly,
		servosArbitrarly
	};

	VehiclePhysicalModel model;
	LQRRegulator lqrRegulator;
	AllocationPenalizers penalizers;

	// VectorXd::Zero( thrusterAmount, 1 ). u is vector of -1 to 1 values of how each thruster is working
	VectorXd thrustValues_u;

	// VectorXd dummyForces       = VectorXd::Zero( 6 );
	// VectorXd dummyThrustSignal = VectorXd::Zero( 5 );

	ct::optcon::LQR< controlDim, controlDim > lqrSolver;

	float regulatorWorkingFrequency{ 10.0f };
	unsigned regulatorTickSpan;
	void processInMainLoop() override;
	void subscribeTopics() override;
	void advertiseTopics() override;
	void connectServices() override;
	void loadRegulatorParameters( configFiles::fileID configID );
};