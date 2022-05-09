#pragma once

#include "CommonEnums.h"
#include "StateBase.h"
#include "StateEnums.h"

#include "jsonCommonFunctions.h"

class StateMission : public StateBase
{
private:
protected:
public:
	explicit StateMission( LogicCommonData* const data, AdditionalDataParameter param )
	    : StateBase( nullptr, data, StateLevel::fundamental, StateType::mission, param )
	{
		this->childStates = jsonFunctions::states::readMission( configFiles::fileID::mission_planner );
	}
	StateReturnType process() override;
	void checkExitCondition() override;
};