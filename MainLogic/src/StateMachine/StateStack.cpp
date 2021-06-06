#include "StateStack.h"

std::shared_ptr< StateBase > createState( StateEnums stateNum )
{
	switch( stateNum )
	{
	case StateEnums::StateIdle:
		return std::make_shared< StateIdle >();
		break;
	default:
		return std::make_shared< StateIdle >();
		break;
	}
}