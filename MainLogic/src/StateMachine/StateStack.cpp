#include "StateStack.h"

std::shared_ptr< StateBase > createState( const StateType type,
                                          LogicCommonData* const data,
                                          const AdditionalDataParameter param )
{
	switch( type )
	{
	case StateType::idle:
		return std::make_shared< StateIdle >();
		break;
	default:
		return std::make_shared< StateIdle >();
		break;
	}
}