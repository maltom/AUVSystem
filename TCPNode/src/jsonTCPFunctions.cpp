#include "jsonTCPFunctions.h"

#include <exception>
#include <string>

bool Frame::processMe()
{
	if( currentType != Type::unprocessed )
	{
		return false;
	}

	this->currentType = Type::preprocessed;

	jsonxx::Object message;
	message.parse( std::get< UnprocessedFrame >( this->content ) );

	if( message.has< jsonxx::String >( dvlKeys::typeOfReport ) )
	{
		if( message.get< jsonxx::String >( dvlKeys::typeOfReport ) == dvlReportTypes::velocity )
		{

			auto timeFromLastReport
			    = static_cast< double >( message.get< jsonxx::Number >( dvlKeys::velocity::timeFromLastReport ) );
			auto vx = static_cast< double >( message.get< jsonxx::Number >( dvlKeys::velocity::vx ) );
			auto vy = static_cast< double >( message.get< jsonxx::Number >( dvlKeys::velocity::vy ) );
			auto vz = static_cast< double >( message.get< jsonxx::Number >( dvlKeys::velocity::vz ) );
			auto figureOfMerit
			    = static_cast< double >( message.get< jsonxx::Number >( dvlKeys::velocity::figureOfMerit ) );
			auto altitude = static_cast< double >( message.get< jsonxx::Number >( dvlKeys::velocity::altitude ) );
			auto velocityValid
			    = static_cast< bool >( message.get< jsonxx::Boolean >( dvlKeys::velocity::velocityValid ) );

			DVLStatus status
			    = static_cast< DVLStatus >( static_cast< int >( message.get< jsonxx::Number >( dvlKeys::status ) ) );

			this->content = ProcessedVelocityFrame(
			    timeFromLastReport, vx, vy, vz, figureOfMerit, altitude, velocityValid, status );

			this->currentType = Type::velocity;
			return true;
		}

		if( message.get< jsonxx::String >( dvlKeys::typeOfReport ) == dvlReportTypes::deadReckoning )
		{
			auto timeStamp
			    = static_cast< double >( message.get< jsonxx::Number >( dvlKeys::deadReckoning::timeStamp ) );
			auto x     = static_cast< double >( message.get< jsonxx::Number >( dvlKeys::deadReckoning::x ) );
			auto y     = static_cast< double >( message.get< jsonxx::Number >( dvlKeys::deadReckoning::y ) );
			auto z     = static_cast< double >( message.get< jsonxx::Number >( dvlKeys::deadReckoning::z ) );
			auto roll  = static_cast< double >( message.get< jsonxx::Number >( dvlKeys::deadReckoning::roll ) );
			auto pitch = static_cast< double >( message.get< jsonxx::Number >( dvlKeys::deadReckoning::pitch ) );
			auto yaw   = static_cast< double >( message.get< jsonxx::Number >( dvlKeys::deadReckoning::yaw ) );

			DVLStatus status
			    = static_cast< DVLStatus >( static_cast< int >( message.get< jsonxx::Number >( dvlKeys::status ) ) );

			this->content = ProcessedDeadReckoningFrame( timeStamp, x, y, z, roll, pitch, yaw, status );

			this->currentType = Type::deadReckoning;
			return true;
		}
	}
	else if( message.has< jsonxx::String >( dvlKeys::responseTo ) )
	{
		if( !message.get< jsonxx::Boolean >( dvlKeys::success ) )
		{
			throw std::runtime_error( "Wrong command sent to DVL!" );
		}
	}
	return false;
}
