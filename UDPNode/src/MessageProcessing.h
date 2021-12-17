#pragma once

#include <cstdint>

#include "CommonEnums.h"
#include "ROSEnums.h"

template< typename T >
constexpr uint16_t adjustThrusterValues( T value )
{
	return static_cast< uint16_t >(
	    ( value
	      + ( static_cast< T >( hardware::motorTorqueMinMax.first )
	          - static_cast< T >( regulator::regulatorOutputSignalMinMax.first ) ) )
	    * ( static_cast< T >( hardware::motorTorqueMinMax.second - hardware::motorTorqueMinMax.first )
	        / static_cast< T >( regulator::regulatorOutputSignalMinMax.second
	                              - regulator::regulatorOutputSignalMinMax.first ) ) );
}