#include "StateBase.h"

#include <exception>

#include "StateEnums.h"

StateReturnType StateBase::process()
{
	switch( this->stateLevel )
	{
	case StateLevel::fundamental:
		if( this->currentChild == this->childStates.end() )
		{
			return { StateProcessed::finished, this->stateType };
		}
		else
		{
			++currentChild;
			return { StateProcessed::pushChild, *this->currentChild };
		}
		break;
	case StateLevel::intermediate:
		if( this->currentChild == this->childStates.end() )
		{
			return { StateProcessed::finished, this->stateType };
		}
		else
		{
			++currentChild;
			return { StateProcessed::pushChild, *this->currentChild };
		}
		break;
	case StateLevel::final:
		throw std::runtime_error( "You need to overload process function for this state." );
		break;
	default:
		break;
	}
}
