#pragma once
#include <memory>

#include "../LogicCommonData.h"
#include "StateEnums.h"

class StateBase
{
private:
	const AdditionalDataParameter parameter;
	const std::shared_ptr< LogicCommonData > commonData;
	const std::shared_ptr< StateBase > parentState;
	const StateLevel stateLevel;
	const StateType stateType;

protected:
	std::vector< StateType > childStates;

public:
	explicit StateBase( StateBase* const parent,
	                    LogicCommonData* const data,
	                    const StateLevel level,
	                    const StateType type,
	                    const AdditionalDataParameter param )
	    : parameter( param ), commonData( data ), parentState( parent ), stateLevel( level ), stateType( type )
	{
		commonData->currentStateName = getStateName< std::string >( stateType );
	}

	virtual StateReturnType process() = 0;
	virtual void checkExitCondition() = 0;

	StateLevel getHierarchy() const
	{
		return this->stateLevel;
	}
	StateType getType() const
	{
		return this->stateType;
	}
};