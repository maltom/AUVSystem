#include <stack>
#include <fstream>
#include <memory>

#include "jsonxx/jsonxx.h"
#include <ros/ros.h>

#include "States/StateBase.h"

template< typename T >
class StateStack final
{
public:
	StateStack() {}
	~StateStack() {}

private:
	std::stack< std::unique_ptr< StateBase > > stateStack;
};