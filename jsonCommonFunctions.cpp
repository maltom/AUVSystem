#include "jsonCommonFunctions.h"

#include <cstdint>
#include <exception>
#include <mutex>

#include <Eigen/Dense>

#include "CommonEnums.h"

namespace jsonFunctions
{
std::mutex busy;
namespace ROS
{
	double readDebugRate( configFiles::fileID configID )
	{
		busy.lock();
		ConfigFile* desiredConfigFile = new ConfigFile( configID );

		auto rosConfig = desiredConfigFile->parsedFile.get< jsonxx::Object >( "ROS" );

		delete desiredConfigFile;
		busy.unlock();
		return rosConfig.get< jsonxx::Number >( "debugRate" );
	}

	double readRosRate( configFiles::fileID configID )
	{
		busy.lock();
		ConfigFile* desiredConfigFile = new ConfigFile( configID );

		auto rosConfig = desiredConfigFile->parsedFile.get< jsonxx::Object >( "ROS" );

		delete desiredConfigFile;
		busy.unlock();
		return rosConfig.get< jsonxx::Number >( "rate" );
	}

	double readHealthCheckRate( configFiles::fileID configID )
	{
		busy.lock();
		ConfigFile* desiredConfigFile = new ConfigFile( configID );

		auto rosConfig = desiredConfigFile->parsedFile.get< jsonxx::Object >( "ROS" );

		delete desiredConfigFile;
		busy.unlock();
		return rosConfig.get< jsonxx::Number >( "healthCheckRate" );
	}

	double readHealthReportRate( configFiles::fileID configID )
	{
		busy.lock();
		ConfigFile* desiredConfigFile = new ConfigFile( configID );

		auto rosConfig = desiredConfigFile->parsedFile.get< jsonxx::Object >( "ROS" );

		delete desiredConfigFile;
		busy.unlock();
		return rosConfig.get< jsonxx::Number >( "healthReportRate" );
	}
} // namespace ROS
namespace network
{
	using namespace ::network;

	uint16_t readDevicePortNumber( configFiles::fileID configID, Device device )
	{

		busy.lock();
		ConfigFile* desiredConfigFile = new ConfigFile( configID );

		jsonxx::Object networkConfig;
		if( desiredConfigFile->parsedFile.has< jsonxx::Object >( "network" ) )
		{
			networkConfig = desiredConfigFile->parsedFile.get< jsonxx::Object >( "network" );
		}
		else
		{
			throw std::runtime_error( "Config file doesn't have any network data!" );
		}
		auto devicesConfig = networkConfig.get< jsonxx::Object >( "devices" );
		jsonxx::Object deviceObj;
		switch( device )
		{
		case Device::devPC:
			if( devicesConfig.has< jsonxx::Object >( "devPC" ) )
			{
				deviceObj = devicesConfig.get< jsonxx::Object >( "devPC" );
			}
			else
			{
				throw std::runtime_error( "Config file doesn't have devPC network data!" );
			}
			break;
		case Device::DVL:
			if( devicesConfig.has< jsonxx::Object >( "DVL" ) )
			{
				deviceObj = devicesConfig.get< jsonxx::Object >( "DVL" );
			}
			else
			{
				throw std::runtime_error( "Config file doesn't have DVL network data!" );
			}
			break;
		case Device::jetson:
			if( devicesConfig.has< jsonxx::Object >( "jetson" ) )
			{
				deviceObj = devicesConfig.get< jsonxx::Object >( "jetson" );
			}
			else
			{
				throw std::runtime_error( "Config file doesn't have jetson network data!" );
			}
			break;
		case Device::microcontroller:
			if( devicesConfig.has< jsonxx::Object >( "microcontroller" ) )
			{
				deviceObj = devicesConfig.get< jsonxx::Object >( "microcontroller" );
			}
			else
			{
				throw std::runtime_error( "Config file doesn't have microcontroller network data!" );
			}
			break;
		default:
			break;
		}
		auto result = deviceObj.get< jsonxx::Number >( "port" );

		delete desiredConfigFile;
		busy.unlock();
		return static_cast< uint16_t >( result );
	}

	std::string readDeviceIPNumber( configFiles::fileID configID, Device device )
	{

		busy.lock();
		ConfigFile* desiredConfigFile = new ConfigFile( configID );

		jsonxx::Object networkConfig;
		if( desiredConfigFile->parsedFile.has< jsonxx::Object >( "network" ) )
		{
			networkConfig = desiredConfigFile->parsedFile.get< jsonxx::Object >( "network" );
		}
		else
		{
			throw std::runtime_error( "Config file doesn't have any network data!" );
		}
		auto devicesConfig = networkConfig.get< jsonxx::Object >( "devices" );
		jsonxx::Object deviceObj;
		switch( device )
		{
		case Device::devPC:
			if( devicesConfig.has< jsonxx::Object >( "devPC" ) )
			{
				deviceObj = devicesConfig.get< jsonxx::Object >( "devPC" );
			}
			else
			{
				throw std::runtime_error( "Config file doesn't have devPC network data!" );
			}
			break;
		case Device::DVL:
			if( devicesConfig.has< jsonxx::Object >( "DVL" ) )
			{
				deviceObj = devicesConfig.get< jsonxx::Object >( "DVL" );
			}
			else
			{
				throw std::runtime_error( "Config file doesn't have DVL network data!" );
			}
			break;
		case Device::jetson:
			if( devicesConfig.has< jsonxx::Object >( "jetson" ) )
			{
				deviceObj = devicesConfig.get< jsonxx::Object >( "jetson" );
			}
			else
			{
				throw std::runtime_error( "Config file doesn't have jetson network data!" );
			}
			break;
		case Device::microcontroller:
			if( devicesConfig.has< jsonxx::Object >( "microcontroller" ) )
			{
				deviceObj = devicesConfig.get< jsonxx::Object >( "microcontroller" );
			}
			else
			{
				throw std::runtime_error( "Config file doesn't have microcontroller network data!" );
			}
			break;
		default:
			break;
		}
		auto ipArray = deviceObj.get< jsonxx::Array >( "IPAdress" );

		std::string result;
		if( ipArray.size() != 4 )
		{
			throw std::runtime_error( "IP is invalid." );
		}
		else
		{
			result = std::to_string( static_cast< uint8_t >( ipArray.get< jsonxx::Number >( 0 ) ) ) + "."
			    + std::to_string( static_cast< uint8_t >( ipArray.get< jsonxx::Number >( 1 ) ) ) + "."
			    + std::to_string( static_cast< uint8_t >( ipArray.get< jsonxx::Number >( 2 ) ) ) + "."
			    + std::to_string( static_cast< uint8_t >( ipArray.get< jsonxx::Number >( 3 ) ) );
		}

		delete desiredConfigFile;
		busy.unlock();
		return result;
	}
} // namespace network

namespace vehicle
{
	VehiclePhysicalModel::Inertial readInertialData( configFiles::fileID configID )
	{
		busy.lock();
		ConfigFile* desiredConfigFile = new ConfigFile( configID );

		VehiclePhysicalModel::Inertial data;

		auto inertialData
		    = desiredConfigFile->parsedFile.get< jsonxx::Object >( "vehicle" ).get< jsonxx::Object >( "inertial" );

		data.buoyancy = inertialData.get< jsonxx::Number >( "buoyancy" );
		data.mass     = inertialData.get< jsonxx::Number >( "mass" );
		data.weight   = data.mass * 9.8066;

		auto moments = inertialData.get< jsonxx::Object >( "moments" );

		data.Ix  = moments.get< jsonxx::Number >( "Ix" );
		data.Iy  = moments.get< jsonxx::Number >( "Iy" );
		data.Iz  = moments.get< jsonxx::Number >( "Iz" );
		data.Ixy = moments.get< jsonxx::Number >( "Ixy" );
		data.Ixz = moments.get< jsonxx::Number >( "Ixz" );
		data.Iyx = moments.get< jsonxx::Number >( "Iyx" );
		data.Iyz = moments.get< jsonxx::Number >( "Iyz" );
		data.Izx = moments.get< jsonxx::Number >( "Izx" );
		data.Izy = moments.get< jsonxx::Number >( "Izy" );

		auto centerOfBuoyancy = inertialData.get< jsonxx::Array >( "centerOfBuoyancy" );
		data.centerOfBuoyancy << centerOfBuoyancy.get< jsonxx::Number >( 0 ),
		    centerOfBuoyancy.get< jsonxx::Number >( 1 ), centerOfBuoyancy.get< jsonxx::Number >( 2 );

		auto centerOfGravity = inertialData.get< jsonxx::Array >( "centerOfGravity" );
		data.centerOfGravity << centerOfGravity.get< jsonxx::Number >( 0 ), centerOfGravity.get< jsonxx::Number >( 1 ),
		    centerOfGravity.get< jsonxx::Number >( 2 );

		delete desiredConfigFile;
		busy.unlock();
		return data;
	}

	VehiclePhysicalModel::Thrusters readThrustersData( configFiles::fileID configID )
	{
		busy.lock();
		ConfigFile* desiredConfigFile = new ConfigFile( configID );

		VehiclePhysicalModel::Thrusters data;

		auto thrustersData
		    = desiredConfigFile->parsedFile.get< jsonxx::Object >( "vehicle" ).get< jsonxx::Object >( "thrusters" );

		auto thrustersPosRot = thrustersData.get< jsonxx::Array >( "positionsAndRotations" );

		data.maxThrust       = thrustersData.get< jsonxx::Number >( "maxThrust" );
		data.deltaU          = thrustersData.get< jsonxx::Number >( "deltaUPerSecond" );
		int thrustersAmount  = thrustersPosRot.size();
		data.thrustersAmount = thrustersAmount;

		for( auto i = 0u; i < thrustersAmount; ++i )
		{
			auto oneThrusterPosRot = thrustersPosRot.get< jsonxx::Array >( i );
			VectorXd thrusterVec   = VectorXd::Zero( 6 );
			thrusterVec << static_cast< double >( oneThrusterPosRot.get< jsonxx::Number >( 0 ) ),
			    static_cast< double >( oneThrusterPosRot.get< jsonxx::Number >( 1 ) ),
			    static_cast< double >( oneThrusterPosRot.get< jsonxx::Number >( 2 ) ),
			    static_cast< double >( oneThrusterPosRot.get< jsonxx::Number >( 3 ) ),
			    static_cast< double >( oneThrusterPosRot.get< jsonxx::Number >( 4 ) ),
			    static_cast< double >( oneThrusterPosRot.get< jsonxx::Number >( 5 ) );
			data.positionsAndRotations.push_back( thrusterVec );
		}

		auto isAzimuthalThrusters = thrustersData.get< jsonxx::Array >( "azimuthal" );

		auto azimuthalThrustersCount{ 0u };

		for( auto i = 0u; i < isAzimuthalThrusters.size(); ++i )
		{
			if( isAzimuthalThrusters.get< jsonxx::Boolean >( i ) )
			{
				++azimuthalThrustersCount;
			}
		}

		data.numberOfAzimuthalThrusters = azimuthalThrustersCount;

		auto mathFunctions = thrustersData.get< jsonxx::Object >( "azimuthalThrustersDimensionsMath" );

		auto baseMathFunctions       = mathFunctions.get< jsonxx::Array >( "baseFunction" );
		auto derivativeMathFunctions = mathFunctions.get< jsonxx::Array >( "derivative" );

		auto sign = []( const std::string& str ) -> double
		{
			if( str == "-" )
				return -1.0;
			else if( str == "+" )
				return 1.0;
			else
				return 0.0;
		};

		auto function = []( const std::string& str ) -> std::function< double( double ) >
		{
			std::function< double( double ) > result;
			if( str == "cos" )
				result = []( double arg ) -> double { return std::cos( arg ); };
			else if( str == "sin" )
				result = []( double arg ) -> double { return std::sin( arg ); };
			else
				result = []( double arg ) -> double { return 0.0; };
			return result;
		};

		auto dimension = []( const std::string& str ) -> dimensionsIndex
		{
			if( str == "x" )
				return dimensionsIndex::x;
			else if( str == "y" )
				return dimensionsIndex::y;
			else if( str == "z" )
				return dimensionsIndex::z;
			else if( str == "0" )
				return static_cast< dimensionsIndex >( 0 );
			else
				throw std::runtime_error( "Wrong dimension index in Azimuthal Math" );
		};

		for( auto i = 0u; i < azimuthalThrustersCount; ++i )
		{
			VehiclePhysicalModel::Thrusters::AzimuthalThrusterFunctions baseFunction;

			auto functionsForThruster = baseMathFunctions.get< jsonxx::Object >( i );
			auto linearFunctions      = functionsForThruster.get< jsonxx::Array >( "linear" );
			auto torqueFunctions      = functionsForThruster.get< jsonxx::Array >( "moments" );

			for( auto j = 0u; j < 3; ++j )
			{
				auto funForOneDim                 = linearFunctions.get< jsonxx::Array >( j );
				baseFunction.linearForces.at( j ) = { sign( funForOneDim.get< jsonxx::String >( 0 ) ),
					                                  function( funForOneDim.get< jsonxx::String >( 1 ) ) };
			}

			for( auto j = 0u; j < 3; ++j )
			{
				auto firstFunForOneDim = torqueFunctions.get< jsonxx::Array >( j ).get< jsonxx::Array >( 0 );
				baseFunction.torques.at( j ).first = { sign( firstFunForOneDim.get< jsonxx::String >( 0 ) ),
					                                   function( firstFunForOneDim.get< jsonxx::String >( 1 ) ),
					                                   dimension( firstFunForOneDim.get< jsonxx::String >( 2 ) ) };

				auto secondFunForOneDim = torqueFunctions.get< jsonxx::Array >( j ).get< jsonxx::Array >( 1 );
				baseFunction.torques.at( j ).second = { sign( secondFunForOneDim.get< jsonxx::String >( 0 ) ),
					                                    function( secondFunForOneDim.get< jsonxx::String >( 1 ) ),
					                                    dimension( secondFunForOneDim.get< jsonxx::String >( 2 ) ) };
			}
			data.azimuthalBaseFunctions.push_back( baseFunction );

			VehiclePhysicalModel::Thrusters::AzimuthalThrusterFunctions derivativeFunction;

			auto derivativesForThruster = derivativeMathFunctions.get< jsonxx::Object >( i );
			auto linearDeirvatives      = derivativesForThruster.get< jsonxx::Array >( "linear" );
			auto torqueDerivatives      = derivativesForThruster.get< jsonxx::Array >( "moments" );

			for( auto j = 0u; j < 3; ++j )
			{
				auto funForOneDim                       = linearDeirvatives.get< jsonxx::Array >( j );
				derivativeFunction.linearForces.at( j ) = { sign( funForOneDim.get< jsonxx::String >( 0 ) ),
					                                        function( funForOneDim.get< jsonxx::String >( 1 ) ) };
			}

			for( auto j = 0u; j < 3; ++j )
			{
				auto firstFunForOneDim = torqueDerivatives.get< jsonxx::Array >( j ).get< jsonxx::Array >( 0 );
				derivativeFunction.torques.at( j ).first
				    = { sign( firstFunForOneDim.get< jsonxx::String >( 0 ) ),
					    function( firstFunForOneDim.get< jsonxx::String >( 1 ) ),
					    dimension( firstFunForOneDim.get< jsonxx::String >( 2 ) ) };

				auto secondFunForOneDim = torqueDerivatives.get< jsonxx::Array >( j ).get< jsonxx::Array >( 1 );
				derivativeFunction.torques.at( j ).second
				    = { sign( secondFunForOneDim.get< jsonxx::String >( 0 ) ),
					    function( secondFunForOneDim.get< jsonxx::String >( 1 ) ),
					    dimension( secondFunForOneDim.get< jsonxx::String >( 2 ) ) };
			}
			data.azimuthalDerivativeFunctions.push_back( derivativeFunction );
		}

		delete desiredConfigFile;
		busy.unlock();
		return data;
	}

	VehiclePhysicalModel::Servos readServosData( configFiles::fileID configID )
	{
		busy.lock();
		ConfigFile* desiredConfigFile = new ConfigFile( configID );

		VehiclePhysicalModel::Servos data;

		auto thrustersData
		    = desiredConfigFile->parsedFile.get< jsonxx::Object >( "vehicle" ).get< jsonxx::Object >( "thrusters" );

		auto isAzimuthalThrusters = thrustersData.get< jsonxx::Array >( "azimuthal" );

		unsigned azimuthalThrustersCount{ 0u };

		for( auto i = 0u; i < isAzimuthalThrusters.size(); ++i )
		{
			if( isAzimuthalThrusters.get< jsonxx::Boolean >( i ) )
			{
				++azimuthalThrustersCount;
			}
		}

		auto dimensionsOfInfluence = thrustersData.get< jsonxx::Array >( "azimuthalThrustersDimensionsInfluence" );

		if( dimensionsOfInfluence.size() > azimuthalThrustersCount )
		{
			throw std::runtime_error( "Too much data describing dimensions of influence." );
		}
		else if( dimensionsOfInfluence.size() < azimuthalThrustersCount )
		{
			throw std::runtime_error( "Not all azimuthal thrusters are described!" );
		}

		for( auto i = 0u; i < azimuthalThrustersCount; ++i )
		{
			auto dimensionsOfOne = dimensionsOfInfluence.get< jsonxx::Array >( i );
			std::vector< dimensionsIndex > dimensionsVec;

			for( auto i = 0u; i < dimensionsOfOne.size(); ++i )
			{
				auto dimension = dimensionsOfOne.get< std::string >( i );
				if( dimension == "x" )
				{
					dimensionsVec.push_back( dimensionsIndex::x );
				}
				else if( dimension == "y" )
				{
					dimensionsVec.push_back( dimensionsIndex::y );
				}
				else if( dimension == "z" )
				{
					dimensionsVec.push_back( dimensionsIndex::z );
				}
				else if( dimension == "roll" )
				{
					dimensionsVec.push_back( dimensionsIndex::roll );
				}
				else if( dimension == "pitch" )
				{
					dimensionsVec.push_back( dimensionsIndex::pitch );
				}
				else if( dimension == "yaw" )
				{
					dimensionsVec.push_back( dimensionsIndex::yaw );
				}
			}
			data.azimuthalThrusterDimensionsOfInfluence.emplace_back( i, dimensionsVec );
		}

		auto servoData
		    = desiredConfigFile->parsedFile.get< jsonxx::Object >( "vehicle" ).get< jsonxx::Object >( "servos" );

		data.servoSpeed = servoData.get< jsonxx::Number >( "angleSpeedPerSecond" );

		// awfully bad practice - but very specific to project's hardware
		for( auto i = 0u; i < azimuthalThrustersCount; ++i )
		{
			data.servosAngles.emplace_back( std::make_pair( math::piNumber / 2.0, dimensionsIndex::pitch ) );
		}

		delete desiredConfigFile;
		busy.unlock();
		return data;
	}

	VehiclePhysicalModel::Drag readDragData( configFiles::fileID configID )
	{
		busy.lock();
		ConfigFile* desiredConfigFile = new ConfigFile( configID );

		VehiclePhysicalModel::Drag data;

		auto dragData = desiredConfigFile->parsedFile.get< jsonxx::Object >( "vehicle" )
		                    .get< jsonxx::Object >( "dragCoefficients" );

		auto linearData    = dragData.get< jsonxx::Object >( "linear" );
		auto quadraticData = dragData.get< jsonxx::Object >( "quadratic" );
		auto addedMassData = dragData.get< jsonxx::Object >( "addedMass" );

		data.linear
		    = VehiclePhysicalModel::Drag::Linear{ static_cast< double >( linearData.get< jsonxx::Number >( "Xu" ) ),
			                                      static_cast< double >( linearData.get< jsonxx::Number >( "Yv" ) ),
			                                      static_cast< double >( linearData.get< jsonxx::Number >( "Zw" ) ),
			                                      static_cast< double >( linearData.get< jsonxx::Number >( "Kp" ) ),
			                                      static_cast< double >( linearData.get< jsonxx::Number >( "Mq" ) ),
			                                      static_cast< double >( linearData.get< jsonxx::Number >( "Nr" ) ) };
		data.quadratic = VehiclePhysicalModel::Drag::Quadratic{
			static_cast< double >( quadraticData.get< jsonxx::Number >( "Xuu" ) ),
			static_cast< double >( quadraticData.get< jsonxx::Number >( "Yvv" ) ),
			static_cast< double >( quadraticData.get< jsonxx::Number >( "Zww" ) ),
			static_cast< double >( quadraticData.get< jsonxx::Number >( "Kpp" ) ),
			static_cast< double >( quadraticData.get< jsonxx::Number >( "Mqq" ) ),
			static_cast< double >( quadraticData.get< jsonxx::Number >( "Nrr" ) )
		};
		data.addedMass = VehiclePhysicalModel::Drag::AddedMass{
			static_cast< double >( addedMassData.get< jsonxx::Number >( "Xua" ) ),
			static_cast< double >( addedMassData.get< jsonxx::Number >( "Yva" ) ),
			static_cast< double >( addedMassData.get< jsonxx::Number >( "Zwa" ) ),
			static_cast< double >( addedMassData.get< jsonxx::Number >( "Kpa" ) ),
			static_cast< double >( addedMassData.get< jsonxx::Number >( "Mqa" ) ),
			static_cast< double >( addedMassData.get< jsonxx::Number >( "Nra" ) )
		};

		delete desiredConfigFile;
		busy.unlock();
		return data;
	}

} // namespace vehicle

namespace regulator
{
	AllocationPenalizers readPenalizerData( configFiles::fileID configID )
	{
		busy.lock();
		ConfigFile* desiredConfigFile = new ConfigFile( configID );

		AllocationPenalizers data;

		auto penData
		    = desiredConfigFile->parsedFile.get< jsonxx::Object >( "regulator" ).get< jsonxx::Object >( "penalizers" );

		auto Omega = penData.get< jsonxx::Array >( "Omega" );
		auto Q     = penData.get< jsonxx::Array >( "Q" );
		auto W     = penData.get< jsonxx::Array >( "W" );

		data.Omega = VectorXd::Zero( Omega.size() );
		data.Q     = VectorXd::Zero( Q.size() );
		data.W     = VectorXd::Zero( W.size() );

		for( auto i = 0u; i < Omega.size(); ++i )
		{
			data.Omega( i ) = static_cast< double >( Omega.get< jsonxx::Number >( i ) );
		}
		for( auto i = 0u; i < Q.size(); ++i )
		{
			data.Q( i ) = static_cast< double >( Q.get< jsonxx::Number >( i ) );
		}
		for( auto i = 0u; i < W.size(); ++i )
		{
			data.W( i ) = static_cast< double >( W.get< jsonxx::Number >( i ) );
		}

		delete desiredConfigFile;
		busy.unlock();
		return data;
	}

	LQRRegulator readLQRData( configFiles::fileID configID )
	{
		busy.lock();
		ConfigFile* desiredConfigFile = new ConfigFile( configID );

		LQRRegulator data;

		auto regulatorData = desiredConfigFile->parsedFile.get< jsonxx::Object >( "regulator" );

		auto QVectorData = regulatorData.get< jsonxx::Array >( "QMatrix" );
		auto RVectorData = regulatorData.get< jsonxx::Array >( "RMatrix" );

		if( QVectorData.size() != ::regulator::stateDim )
		{
			throw std::runtime_error( "Wrong number of parameters in QMatrix!" );
		}
		if( RVectorData.size() != ::regulator::controlDim )
		{
			throw std::runtime_error( "Wrong number of parameters in RMatrix!" );
		}

		VectorXd Qvec = VectorXd::Zero( ::regulator::stateDim );
		VectorXd Rvec = VectorXd::Zero( ::regulator::controlDim );

		for( auto i = 0u; i < ::regulator::stateDim; ++i )
		{
			Qvec( i ) = static_cast< double >( QVectorData.get< jsonxx::Number >( i ) );
		}

		for( auto i = 0u; i < ::regulator::controlDim; ++i )
		{
			Rvec( i ) = static_cast< double >( RVectorData.get< jsonxx::Number >( i ) );
		}

		data.Q.diagonal() << Qvec;
		data.R.diagonal() << Rvec;

		delete desiredConfigFile;
		busy.unlock();
		return data;
	}
	double readWorkingFrequency( configFiles::fileID configID )
	{
		busy.lock();
		ConfigFile* desiredConfigFile = new ConfigFile( configID );

		double data{ 0.0 };

		auto regulatorData = desiredConfigFile->parsedFile.get< jsonxx::Object >( "regulator" );

		data = static_cast< double >( regulatorData.get< jsonxx::Number >( "regulatorWorkingFrequency" ) );

		delete desiredConfigFile;
		busy.unlock();
		return data;
	}

} // namespace regulator

namespace cameras
{
	bool checkForStereoscopicCameraPresent( configFiles::fileID configID )
	{
		busy.lock();
		ConfigFile* desiredConfigFile = new ConfigFile( configID );

		auto cameraData
		    = desiredConfigFile->parsedFile.get< jsonxx::Object >( "cameras" ).get< jsonxx::Array >( "stereoscopic" );

		auto countStereo = 0u;
		for( const auto& in : cameraData.values() )
		{
			countStereo += static_cast< unsigned >( in->get< jsonxx::Boolean >() );
		}

		delete desiredConfigFile;
		busy.unlock();

		auto result = false;
		switch( countStereo )
		{
		case 0u:
			result = false;
			break;
		case 1u:
			result = true;
			break;
		default:
			throw std::runtime_error( "More than one stereoscopic camera found in config!" );
			break;
		}
		return result;
	}

	uint8_t countAdditionalCameras( configFiles::fileID configID )
	{
		busy.lock();
		ConfigFile* desiredConfigFile = new ConfigFile( configID );

		auto cameraData = desiredConfigFile->parsedFile.get< jsonxx::Object >( "cameras" );

		auto stereoscopic = cameraData.get< jsonxx::Array >( "stereoscopic" );
		if( stereoscopic.size() != cameraData.get< jsonxx::Array >( "positionsAndRotations" ).size() )
		{
			throw std::runtime_error( "Data describing camera is corrupted." );
		}
		uint8_t data{ 0u };

		for( auto i = 0u; i < stereoscopic.size(); ++i )
		{
			if( !stereoscopic.get< jsonxx::Boolean >( i ) )
			{
				++data;
			}
		}

		delete desiredConfigFile;
		busy.unlock();
		return data;
	}

	// std::vector< float > getFOV( configFiles::fileID configID )
	// {
	// 	busy.lock();
	// 	ConfigFile* desiredConfigFile = new ConfigFile( configID );

	// 	double data;

	// 	auto regulatorData = desiredConfigFile->parsedFile.get< jsonxx::Object >( "regulator" );

	// 	data = static_cast< double >( regulatorData.get< jsonxx::Number >( "regulatorWorkingFrequency" ) );

	// 	delete desiredConfigFile;
	// 	busy.unlock();
	// 	return data;
	// }
} // namespace cameras
namespace states
{
	std::vector< StateType > readMission( configFiles::fileID configID )
	{
		busy.lock();
		ConfigFile* desiredConfigFile = new ConfigFile( configID );

		std::vector< StateType > data;

		auto mission = desiredConfigFile->parsedFile.get< jsonxx::Array >( "mission" );

		for( auto i = 0u; i < mission.size(); ++i )
		{
			auto task = mission.get< jsonxx::String >( i );

			if( task == "test" )
			{
				data.emplace_back( StateType::test );
			}
			else if( task == "qualificationTask" )
			{
				data.emplace_back( StateType::qualificationTask );
			}
			else if( task == "task1" )
			{
				data.emplace_back( StateType::task1 );
			}
			else if( task == "task2" )
			{
				data.emplace_back( StateType::task2 );
			}
			else if( task == "task3" )
			{
				data.emplace_back( StateType::task3 );
			}
			else if( task == "task4" )
			{
				data.emplace_back( StateType::task4 );
			}
			else if( task == "task5" )
			{
				data.emplace_back( StateType::task5 );
			}
			else if( task == "task6" )
			{
				data.emplace_back( StateType::task6 );
			}
			else if( task == "task7" )
			{
				data.emplace_back( StateType::task7 );
			}
			else if( task == "task8" )
			{
				data.emplace_back( StateType::task8 );
			}
			else if( task == "task9" )
			{
				data.emplace_back( StateType::task9 );
			}
			else if( task == "task10" )
			{
				data.emplace_back( StateType::task10 );
			}
		}

		delete desiredConfigFile;
		busy.unlock();
		return data;
	}
} // namespace states

} // namespace jsonFunctions