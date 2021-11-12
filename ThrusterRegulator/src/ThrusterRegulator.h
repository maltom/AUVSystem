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

void allocateThrust( const VectorXd& desiredForces_tau,
                     const VehiclePhysicalModel& model,
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
	}
	~ThrusterRegulator() {}

protected:
private:
	VehiclePhysicalModel model;
	LQRRegulator lqrRegulator;
	AllocationPenalizers penalizers;

	ct::optcon::LQR< controlDim, controlDim > lqrSolver;

	float regulatorWorkingFrequency{ 10.0f };

	void processInMainLoop() override;
	void subscribeTopics() override;
	void advertiseTopics() override;
	void connectServices() override;
	void loadRegulatorParameters( configFiles::fileID configID );

	void sendArbitrarlySetThrusters( const AUVROS::MessageTypes::ThrustersSignal& signal );
	void sendArbitrarlySetServos( const AUVROS::MessageTypes::ServosSignal& signal );
	// void allocateThrust();
};