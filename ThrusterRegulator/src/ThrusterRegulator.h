#pragma once

#include <fstream>
#include <memory>

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

protected:
private:
	void subscribeTopics() override;
	void advertiseTopics() const override;
	void connectServices() const override;

	

};