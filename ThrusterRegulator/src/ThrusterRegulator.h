#pragma once

#include <fstream>
#include <memory>

#include <ct/optcon/optcon.h>

#include "NodeBase.h"

class ThrusterRegulator final : public NodeBase
{
public:
	ThrusterRegulator( std::shared_ptr< ros::NodeHandle >& node, std::fstream& config ) : NodeBase( node, config )
	{
		subscribeTopics();
	}
	~ThrusterRegulator() {}

	void startMainLoop()  override;
	ct::optcon::LQR< 6, 6 > lqrSolver;

protected:
private:
	void subscribeTopics() override;
	void advertiseTopics() const override;
	void connectServices() const override;

	

};