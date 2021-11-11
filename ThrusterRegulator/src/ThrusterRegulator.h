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

MatrixXd calculateNbar( const Matrix< double, 12, 12 >& A,
                        const Matrix< double, 12, 6 >& B,
                        const Matrix< double, 6, 12 >& K );
Matrix< double, 12, 12 > calculateAStateMatrix( const VectorXd& currentState, const VehiclePhysicalModel& model );
Matrix< double, 12, 6 > calculateBStateMatrix( const VehiclePhysicalModel& model );

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

	ct::optcon::LQR< 6, 6 > lqrSolver;

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