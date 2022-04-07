#include "Displayer.h"

#include <algorithm>
#include <iostream>
#include <exception>

void Displayer::addInfoToDisplay( const std::string& header,
                                  const std::vector< std::string >& labels,
                                  const DataType& init )
{
	if( header.size() > display::headerDisplayWidth )
	{
		throw std::runtime_error( "Header name is too long." );
	}

	this->dataColumns.emplace_back( header, labels, init );
}

void Displayer::setMajorColumnValues( const unsigned dataColumnIndex, const std::vector< DataType >& dataForDisplayer )
{
	if( this->dataColumns.at( dataColumnIndex ).dataLabelsFields.size() != dataForDisplayer.size() )
	{
		throw std::runtime_error( "Wrong data or data size for display." );
	}

	for( auto i = 0u; i < dataForDisplayer.size(); ++i )
	{
		this->dataColumns.at( dataColumnIndex ).dataLabelsFields.at( i ).second = dataForDisplayer.at( i );
	}
}

std::string Displayer::createUtilityRow( const display::rowType type ) const
{
	std::string row;
	row.reserve( display::totalDisplayWidth );
	switch( type )
	{
	case display::rowType::topBorder:
		row = display::topLeftCorner;
		for( auto i = 0u; i < display::rowSeparatorWidth; ++i )
		{
			row += display::rowMajorSeparator;
		}
		row += display::topRightCorner;
		break;
	case display::rowType::bottomBorder:
		row = display::bottomLeftCorner;
		for( auto i = 0u; i < display::rowSeparatorWidth; ++i )
		{
			row += display::rowMajorSeparator;
		}
		row += display::bottomRightCorner;
		break;
	case display::rowType::majorSeparator:
		row = display::leftBorderSeparator;
		for( auto i = 0u; i < display::rowSeparatorWidth; ++i )
		{
			row += display::rowMajorSeparator;
		}
		row += display::rightBorderSeparator;
		break;
	case display::rowType::minorSeparator:
		row = display::leftBorderSeparator;
		for( auto i = 0u; i < display::rowSeparatorWidth; ++i )
		{
			row += display::rowMinorSeparator;
		}
		row += display::rightBorderSeparator;
		break;
	default:
		break;
	}
	row += '\n';
	return row;
}
std::string Displayer::createMajorRow( const unsigned dataRowNumber ) const
{
	std::string dataMajorRow;
	std::string headerRow;
	headerRow.reserve( display::totalDisplayWidth );

	headerRow = display::sideBorderSeparator;
	int bias{ -1 }; // adjusting padding for signs longer than one 1B
	for( auto i = dataRowNumber * display::totalNumberOfMajorColumnsPerRow;
	     ( i < dataColumns.size() && i < ( dataRowNumber + 1 ) * display::totalNumberOfMajorColumnsPerRow );
	     ++i )
	{
		if( i < dataColumns.size() )
		{
			headerRow += " ";
			headerRow += dataColumns.at( i ).header;

			for( auto j = 0u;
			     j < display::dataDisplayWidth + display::labelDisplayWidth - dataColumns.at( i ).header.size();
			     ++j )
			{
				headerRow += " ";
			}
			headerRow += display::majorColumnSeparator;
			bias -= 2;
		}
		else
		{
			for( auto j = headerRow.size(); j < display::totalDisplayWidth - bias; ++j )
			{
				headerRow += " ";
			}
			break;
		}
	}
	headerRow += display::sideBorderSeparator;
	headerRow += "\n";

	dataMajorRow = headerRow;

	dataMajorRow.append( createUtilityRow( display::rowType::minorSeparator ) );

	size_t maxColumnHeight{ 0 };

	for( auto i = dataRowNumber * display::totalNumberOfMajorColumnsPerRow;
	     ( i < dataColumns.size() && i < ( dataRowNumber + 1 ) * display::totalNumberOfMajorColumnsPerRow );
	     ++i )
	{
		maxColumnHeight = std::max( maxColumnHeight, dataColumns.at( i ).dataLabelsFields.size() );
	}

	for( auto subRowIndex = 0u; subRowIndex < maxColumnHeight; ++subRowIndex )
	{
		std::string dataSubRow;
		dataSubRow.reserve( display::totalDisplayWidth );
		dataSubRow = display::sideBorderSeparator;

		bias = -1; // adjusting padding for signs longer than one 1B

		for( auto i = dataRowNumber * display::totalNumberOfMajorColumnsPerRow;
		     ( i < dataColumns.size() && i < ( dataRowNumber + 1 ) * display::totalNumberOfMajorColumnsPerRow );
		     ++i )
		{
			if( i < dataColumns.size() )
			{
				if( subRowIndex < dataColumns.at( i ).dataLabelsFields.size() )
				{
					// label
					dataSubRow += " ";
					auto labelToAdd = dataColumns.at( i ).dataLabelsFields.at( subRowIndex ).first;
					labelToAdd.resize( display::labelDisplayWidth - 2u, ' ' );
					dataSubRow += labelToAdd;

					for( auto j = 0u; j < display::labelDisplayWidth - labelToAdd.size() - 1; ++j )
					{
						dataSubRow += " ";
					}

					dataSubRow += display::minorColumnSeparator;
					bias -= 2;
					// value
					dataSubRow += " ";
					auto valueToAdd = dataColumns.at( i ).dataLabelsFields.at( subRowIndex ).second;
					valueToAdd.resize( display::dataDisplayWidth - 2u, ' ' );
					dataSubRow += valueToAdd;

					for( auto j = 0u; j < display::dataDisplayWidth - valueToAdd.size() - 1; ++j )
					{
						dataSubRow += " ";
					}
					dataSubRow += display::majorColumnSeparator;
					bias -= 2;
				}
				else
				{
					for( auto j = 0u;
					     j < display::labelDisplayWidth + display::dataDisplayWidth + display::columnSeparatorWidth;
					     ++j )
					{
						dataSubRow += " ";
					}
					dataSubRow += display::majorColumnSeparator;
					bias -= 2;
				}
			}
			else
			{
				for( auto j = dataSubRow.size(); j < display::totalDisplayWidth - bias; ++j )
				{
					dataSubRow += " ";
				}
				break;
			}
		}

		dataSubRow += display::sideBorderSeparator;
		dataSubRow += "\n";
		dataMajorRow += dataSubRow;
	}
	return dataMajorRow;
}

void Displayer::displayDebugInfo( const bool isNumberOfSubscribedTopicsGood ) const
{
	std::string finalDisplay;
	finalDisplay.reserve( display::totalDisplayWidth * display::totalDisplayHeight );
	if( !isNumberOfSubscribedTopicsGood )
	{
		finalDisplay = "[WARNING] Number of subscribed topics is different than number of topics in system.\n";
	}

	// top border
	finalDisplay.append( createUtilityRow( display::rowType::topBorder ) );

	// add some stuff
	finalDisplay.append( createMajorRow( 0u ) );

	finalDisplay.append( createUtilityRow( display::rowType::majorSeparator ) );

	finalDisplay.append( createMajorRow( 1u ) );

	// bottom
	finalDisplay.append( createUtilityRow( display::rowType::bottomBorder ) );
	system( "clear" );
	finalDisplay.shrink_to_fit();
	std::cout << finalDisplay;
}

void Displayer::setupDisplayPositions() {}
void Displayer::setupDisplayLabels() {}
