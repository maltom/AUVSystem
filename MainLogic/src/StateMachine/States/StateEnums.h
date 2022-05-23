#pragma once
#include <utility>
#include <variant>

#include "CommonEnums.h"
#include "ROSEnums.h"

using AdditionalDataParameter = std::variant< double >;

enum class StateType
{
	// fundamental types
	idle,
	mission,
	startup,
	emergency,
	surfaceTheBoat,

	// task[N] intermediate types,
	// final types

	// test and qualification
	test,
	qualificationTask,

	task1, // with moxy - coin flip - get out of dock
	diveToDepth,
	findGate,
	goLittleTowardsGate,

	task2, // get through gate
	detectGate,
	getToThePositionBeforeGate,
	chooseGateSide,
	getThroughGate,

	task3, // follow path 1
	detectMarker,
	getOverMarker,
	reachCorrectAzimuth,

	task4,                              // buoys
	travelAzimuthUntilObstacleDetected, // parameter - buoys
	chooseBuoy,
	bumpBuoy,
	moveAsideFromBuoys,

	task5, // follow path 2 - reuse states as in task3
	       // detectMarker,
	       // getOverMarker,
	       // reachCorrectAzimuth,

	task6, // collecting - bins
	       // travelAzimuthUntilObstacleDetected, // reuse, parameter - bins
	       // getOverBinOne,			// propositions
	       // recognizeTheBin,			// propositions
	       // getOverBinTwo,			// propositions
	       // recognizeTheBin,			// propositions
	       // dropMarkersToTargetBin,	// propositions
	       // getCoverOfTargetBin,		// propositions

	task7,              // pinger 1
	getAzimuthToPinger, // parameter - pinger frequency
	                    // reachCorrectAzimuth, // reuse

	task8, // survive the shootout
	       // travelAzimuthUntilObstacleDetected, // reuse, parameter - targets
	getToThePositionBeforeTargets,
	positionBeforeCorrectTarget,
	aimAndFireTorpedoes,
	moveAsideFromTargets,

	task9, // pinger 2 - reuse states as in task7
	       // getAzimuthToPinger, // reuse, parameter - pinger frequency
	       // reachCorrectAzimuth // reuse

	task10, // cash or smash
	        // travelAzimuthUntilObstacleDetected, // reuse, parameter - platforms/bottles/octagon
	        // surfaceWithFirstBottle,				// proposition
	        // putFirstBottleOnCorrectPlatform,		// proposition
	        // surfaceWithSecondBottle,				// proposition
	        // putSecondBottleOnCorrectPlatform,	// proposition
	        // surfaceWithFThirdBottle,				// proposition
	        // putThirdBottleOnCorrectPlatform,		// proposition
	        // surfaceWithFourthBottle,				// proposition
	        // putFourthBottleOnCorrectPlatform,	// proposition
	surfaceInsideOctagon,

	StartingState = mission
};

enum class StateLevel
{
	fundamental,

	intermediate,
	final

};

enum class StateProcessed
{
	keepCurrentState,
	finished,
	pushChild,
};

template< typename T >
constexpr T getStateName( const StateType type )
{
	switch( type )
	{
	case StateType::idle:
		return "Idle";
	case StateType::mission:
		return "Mission";
	case StateType::startup:
		return "Start-up";
	case StateType::emergency:
		return "Emergency";
	case StateType::surfaceTheBoat:
		return "Surface The Boat";
	case StateType::test:
		return "Test";
	case StateType::qualificationTask:
		return "QualificationTask";
	case StateType::task1:
		return "Task1";
	case StateType::diveToDepth:
		return "Dive To Depth";
	case StateType::findGate:
		return "Find Gate";
	case StateType::goLittleTowardsGate:
		return "GoLittleTowardsGate";
	case StateType::task2:
		return "Task2";
	case StateType::detectGate:
		return "Detect Gate";
	case StateType::getToThePositionBeforeGate:
		return "GetToThePositionBeforeGate";
	case StateType::chooseGateSide:
		return "Choose Gate Side";
	case StateType::getThroughGate:
		return "GetThroughGate";
	case StateType::task3:
		return "";
	case StateType::detectMarker:
		return "";
	case StateType::getOverMarker:
		return "";
	case StateType::reachCorrectAzimuth:
		return "";
	case StateType::task4:
		return "";
	case StateType::travelAzimuthUntilObstacleDetected:
		return "";
	case StateType::chooseBuoy:
		return "";
	case StateType::bumpBuoy:
		return "";
	case StateType::moveAsideFromBuoys:
		return "";
	case StateType::task5:
		return "";
	case StateType::task6:
		return "";
	case StateType::task7:
		return "";
	case StateType::getAzimuthToPinger:
		return "";
	case StateType::task8:
		return "";
	case StateType::getToThePositionBeforeTargets:
		return "";
	case StateType::positionBeforeCorrectTarget:
		return "";
	case StateType::aimAndFireTorpedoes:
		return "";
	case StateType::moveAsideFromTargets:
		return "";
	case StateType::task9:
		return "";
	case StateType::task10:
		return "";
	case StateType::surfaceInsideOctagon:
		return "";
	default:
		return "incorrect name";
	}
}

using StateReturnType = std::pair< StateProcessed, StateType >;