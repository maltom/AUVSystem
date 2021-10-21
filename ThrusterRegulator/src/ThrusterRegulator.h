#pragma once

#include <fstream>
#include <memory>

#include <ct/optcon/optcon.h>

#include "CommonEnums.h"
#include "NodeBase.h"
#include "VehiclePhysicalModel.h"

class ThrusterRegulator final : public NodeBase
{
public:
	ThrusterRegulator( std::shared_ptr< ros::NodeHandle >& node, configFiles::fileID configID )
	    : NodeBase( node, configID ), model( configID )
	{
		subscribeTopics();
	}
	~ThrusterRegulator() {}

	void startMainLoop() override;
	ct::optcon::LQR< 6, 6 > lqrSolver;

protected:
private:
	VehiclePhysicalModel model;
	void subscribeTopics() override;
	void advertiseTopics() const override;
	void connectServices() const override;
};