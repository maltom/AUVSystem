#include <memory>

#include "StateMachine/StateMachine.h"

class MainLogic final
{
public:
	MainLogic( std::shared_ptr< ros::NodeHandle >& node, std::fstream& config ) : rosNode( node ), configFile( config )
	{
		file.parse( configFile );
	}
	~MainLogic() {}

protected:
private:
	std::unique_ptr< StateMachine > stateMachine;
	std::shared_ptr< ros::NodeHandle >& rosNode;
	std::fstream& configFile;
	jsonxx::Object file;

	void subscribeTopics() const;
	void advertiseTopics() const;
	void connectServices() const;
};