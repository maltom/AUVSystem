#include "StateBase.h"

#include <exception>

#include "StateEnums.h"

StateReturnType StateBase::process()
{
	switch( this->stateLevel )
	{
	case StateLevel::fundamental:
		if( this->currentChildIndex < this->childStates.size() )
		{
			return { StateProcessed::pushChild, this->childStates.at( currentChildIndex++ ) };
		}
		else
		{
			return { StateProcessed::finished, this->stateType };
		}
		break;
	case StateLevel::intermediate:
		if( this->currentChildIndex < this->childStates.size() )
		{
			return { StateProcessed::pushChild, this->childStates.at( currentChildIndex++ ) };
		}
		else
		{
			return { StateProcessed::finished, this->stateType };
		}
		break;
	case StateLevel::final:
		throw std::runtime_error( "You need to overload process function for this state." );
		break;
	default:
		break;
	}
}
