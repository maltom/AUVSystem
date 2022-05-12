#pragma once

#include "CommonEnums.h"
#include "StateBase.h"
#include "StateEnums.h"

#include "jsonCommonFunctions.h"

class StateGoLittleTowardsGate : public StateBase
{
private:
protected:
public:
	explicit StateGoLittleTowardsGate( LogicCommonData* const data,
	                                   AdditionalDataParameter param,
	                                   StateBase* const parentState )
	    : StateBase( parentState, data, StateLevel::final, StateType::goLittleTowardsGate, param )
	{
	}
	StateReturnType process() override;
	void checkExitCondition() override;
};