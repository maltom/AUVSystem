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
	explicit StateGoLittleTowardsGate( const std::shared_ptr< LogicCommonData >& data,
	                                   AdditionalDataParameter param,
	                                   const std::shared_ptr< StateBase >& parent )
	    : StateBase( parent, data, StateLevel::final, StateType::goLittleTowardsGate, param )
	{
	}
	StateReturnType process() override;
	void checkExitCondition() override;
};