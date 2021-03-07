#include "Vehicle.h"

Vehicle::Vehicle()
{
    statemachine = std::make_unique<StateMachine>();
}