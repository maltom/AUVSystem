#pragma once

#include <memory>

#include <ct/optcon/optcon.h>
#include <Eigen/Dense>

#include "CommonEnums.h"
#include "NodeBase.h"
#include "LQRRegulator.h"
#include "VehiclePhysicalModel.h"

using namespace Eigen;

class ThrusterRegulator final : public NodeBase
{
public:
	ThrusterRegulator( std::shared_ptr< ros::NodeHandle >& node, configFiles::fileID configID )
	    : NodeBase( node, configID ), model( configID )
	{
		loadRegulatorParameters( this->configFileID );
		subscribeTopics();
	}
	~ThrusterRegulator() {}

	void startMainLoop() override;

protected:
private:
	VehiclePhysicalModel model;
	LQRRegulator lqrRegulator;

	ct::optcon::LQR< 6, 6 > lqrSolver;

	float regulatorWorkingFrequency{ 10.0f };

	void subscribeTopics() override;
	void advertiseTopics() override;
	void connectServices() override;
	void loadRegulatorParameters( configFiles::fileID configID );
	// void allocateThrust();
};