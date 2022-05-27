#include <array>
#include <optional>
#include <string>
#include <variant>

#include "external/jsonxx/jsonxx.h"

#include "CommonEnums.h"

using UnprocessedFrame = network::TCPunstickedMessage;

namespace dvlKeys
{
namespace velocity
{
	const std::string timeFromLastReport{ "time" };
	const std::string vx{ "vx" };
	const std::string vy{ "vy" };
	const std::string vz{ "vz" };
	const std::string figureOfMerit{ "fom" };
	const std::string altitude{ "altitude" };
	const std::string velocityValid{ "velocity_valid" };

} // namespace velocity

namespace deadReckoning
{
	const std::string timeStamp{ "ts" };
	const std::string x{ "x" };
	const std::string y{ "y" };
	const std::string z{ "z" };
	const std::string standardDeviation{ "std" };
	const std::string roll{ "roll" };
	const std::string pitch{ "pitch" };
	const std::string yaw{ "yaw" };
} // namespace deadReckoning

const std::string status{ "status" };
const std::string typeOfReport{ "type" };
const std::string responseTo{ "response_to" };
const std::string success{ "success" };

} // namespace dvlKeys

namespace dvlReportTypes
{
const std::string deadReckoning{ "position_local" };
const std::string velocity{ "velocity" };
} // namespace dvlReportTypes

enum class DVLStatus
{
	healthy = 0,
	failure
};

struct ProcessedVelocityFrame
{
	ProcessedVelocityFrame( double _timeFromLastReport,
	                        double _vx,
	                        double _vy,
	                        double _vz,
	                        double _figureOfMerit,
	                        double _altitude,
	                        bool _velocityValid,
	                        DVLStatus _status )
	    : timeFromLastReport( _timeFromLastReport ),
	      vx( _vx ),
	      vy( vy ),
	      vz( _vz ),
	      figureOfMerit( _figureOfMerit ),
	      altitude( _altitude ),
	      velocityValid( _velocityValid ),
	      status( _status )
	{
	}

	double timeFromLastReport{ 0.0 };
	double vx{ 0.0 };
	double vy{ 0.0 };
	double vz{ 0.0 };
	double figureOfMerit{ 0.0 };
	double altitude{ 0.0 };
	bool velocityValid{ false };
	DVLStatus status{ DVLStatus::healthy };
};

struct ProcessedDeadReckoningFrame
{
	ProcessedDeadReckoningFrame( double _timeStamp,
	                             double _x,
	                             double _y,
	                             double _z,
	                             double _roll,
	                             double _pitch,
	                             double _yaw,
	                             DVLStatus _status )
	    : timeStamp( _timeStamp ),
	      x( _x ),
	      y( _y ),
	      z( _z ),
	      roll( _roll ),
	      pitch( _pitch ),
	      yaw( _yaw ),
	      status( _status )
	{
	}
	double timeStamp{ 0.0 };
	double x{ 0.0 };
	double y{ 0.0 };
	double z{ 0.0 };
	double roll{ 0.0 };
	double pitch{ 0.0 };
	double yaw{ 0.0 };
	DVLStatus status{ DVLStatus::healthy };
};

struct Frame
{
	enum class Type
	{
		none,
		unprocessed,
		preprocessed,
		deadReckoning,
		velocity
	};

	explicit Frame( UnprocessedFrame incomingFrame )
	{
		this->content     = std::move( incomingFrame );
		this->currentType = Type::unprocessed;
	}

	std::variant< UnprocessedFrame, ProcessedVelocityFrame, ProcessedDeadReckoningFrame > content;

	Type currentType{ Type::none };

	bool processMe();
};