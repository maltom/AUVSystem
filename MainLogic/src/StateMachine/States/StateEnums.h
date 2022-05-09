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

	StartingState = idle
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
		break;
	case StateType::mission:
		return "Mission";
	default:
		break;
	}
}

using StateReturnType = std::pair< StateProcessed, StateType >;