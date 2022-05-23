#pragma once

#include "CommonEnums.h"
#include "StateBase.h"
#include "StateEnums.h"

#include "jsonCommonFunctions.h"

class StateTask1 : public StateBase
{
private:
protected:
public:
	explicit StateTask1( const std::shared_ptr< LogicCommonData >& data,
	                     AdditionalDataParameter param,
	                     const std::shared_ptr< StateBase >& parent )
	    : StateBase( parent, data, StateLevel::intermediate, StateType::task1, param )
	{
		this->childStates.insert( this->childStates.begin(),
		                          { StateType::diveToDepth, StateType::findGate, StateType::goLittleTowardsGate } );
	}
	void checkExitCondition() override;
};