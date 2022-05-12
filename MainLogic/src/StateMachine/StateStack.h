#pragma once

#include <exception>
#include <fstream>
#include <memory>
#include <stack>

#include "LogicCommonData.h"
#include "States/StateBase.h"
#include "States/StateIdle.h"
#include "States/StateMission.h"
#include "States/StateEnums.h"

#include "States/StateDiveToDepth.h"
#include "States/StateFindGate.h"
#include "States/StateGoLittleTowardsGate.h"
#include "States/StateTask1.h"
#include "States/StateTask2.h"

// class StateStack
template< typename T >
class StateStack final
{
public:
	StateStack( LogicCommonData* const data ) : commonData( data )
	{
		pushStateOnTop( StateType::StartingState );
	}

	std::shared_ptr< T > getCurrentState() const;
	void setStateInsteadOfCurrent( const StateType type );
	void pushStateOnTop( const StateType type );
	void popState();
	void popMultipleStates( const unsigned number );

	void process();

private:
	std::stack< std::shared_ptr< T > > stateStack;
	std::shared_ptr< LogicCommonData > commonData;
};

// definitions
template< typename T >
std::shared_ptr< T > StateStack< T >::getCurrentState() const
{
	return this->stateStack.top();
}

template< typename T >
void StateStack< T >::setStateInsteadOfCurrent( const StateType type )
{
	if( this->stateStack.size() > 0 )
	{
		this->stateStack.pop();
		pushStateOnTop( this->pushStateOnTop( type ) );
	}
	else
	{
		throw std::runtime_error( "StateStack is empty!" );
	}
}

template< typename T >
void StateStack< T >::popState()
{
	if( this->stateStack.size() > 1 )
	{
		this->stateStack.pop();
	}
	else
	{
		throw std::runtime_error( "StateStack can't be empty!" );
	}
}

template< typename T >
void StateStack< T >::popMultipleStates( const unsigned number )
{
	if( this->stateStack.size() > number )
	{
		for( auto i = 0u; i < number; ++i )
		{
			this->stateStack.pop();
		}
	}
	else
	{
		throw std::runtime_error( "Too few states on stack to pop!" );
	}
}

template< typename T >
void StateStack< T >::pushStateOnTop( const StateType type )
{
	switch( type )
	{
	case StateType::idle:
		this->stateStack.emplace( std::make_shared< StateIdle >( this->commonData.get(), 0.0 ) );
		break;
	case StateType::startup:
		break;
	case StateType::mission:
		this->stateStack.emplace( std::make_shared< StateMission >( this->commonData.get(), 0.0 ) );
		break;
	case StateType::task1:
		this->stateStack.emplace(
		    std::make_shared< StateTask1 >( this->commonData.get(), 0.0, &( *stateStack.top() ) ) );
		break;
	case StateType::task2:
		this->stateStack.emplace(
		    std::make_shared< StateTask2 >( this->commonData.get(), 0.0, &( *stateStack.top() ) ) );
		break;
	default:
		break;
	}
}

template< typename T >
void StateStack< T >::process()
{
	auto result = this->stateStack.top()->process();

	switch( result.first )
	{
	case StateProcessed::finished:
		this->popState();
		break;
	case StateProcessed::keepCurrentState:
		break;
	case StateProcessed::pushChild:
		this->pushStateOnTop( result.second );
		break;
	default:
		break;
	}
}