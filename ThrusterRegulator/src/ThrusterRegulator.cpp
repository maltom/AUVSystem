#include "ThrusterRegulator.h"

#include <cassert>
#include <chrono>
#include <iostream>
#include <limits>
#include <vector>

#include "external/EigenQP/EigenQP.h"
#include "ROSEnums.h"

void ThrusterRegulator::processInMainLoop()
{

	if( ticks % regulatorTickSpan == 0 )
	{
		// std::cout << "\n\n\n\n\n\nQ:\n"
		//           << static_cast< MatrixXd >( lqrRegulator.Q ) << "\nR:\n"
		//           << static_cast< MatrixXd >( lqrRegulator.R ) << "\nA:\n"
		//           << lqrRegulator.A << "\nB:\n"
		//           << lqrRegulator.B << "\ncurrent pos:\n"
		//           << this->currentPosition << "\ndesired pos:\n"
		//           << this->positionToReach << "\nthruster sig:\n"
		//           << this->thrustValues_u <<"\n---------"<< std::endl;
#ifndef NOLQR
		this->lqrRegulator.calculate( this->currentState, this->model );
		// std::cout << "Q:\n"
		//           << static_cast< MatrixXd >( lqrRegulator.Q ) << "\nR:\n"
		//           << static_cast< MatrixXd >( lqrRegulator.R ) << "\nA:\n"
		//           << lqrRegulator.A << "\nB:\n"
		//           << lqrRegulator.B << "\ncurrent pos:\n"
		//           << this->currentPosition << "\ndesired pos:\n"
		//           << this->positionToReach << "\nthruster sig:\n"
		//           << this->thrustValues_u <<"\n---------"<< std::endl;

		this->lqrRegulator.calculateError( this->currentState, this->positionToReach );
		// std::cout << "Q:\n"
		//           << static_cast< MatrixXd >( lqrRegulator.Q ) << "\nR:\n"
		//           << static_cast< MatrixXd >( lqrRegulator.R ) << "\nA:\n"
		//           << lqrRegulator.A << "\nB:\n"
		//           << lqrRegulator.B << "\ncurrent pos:\n"
		//           << this->currentPosition << "\ndesired pos:\n"
		//           << this->positionToReach << "\nthruster sig:\n"
		//           << this->thrustValues_u <<"\n---------"<< std::endl;
#endif
		allocateThrust2Azimuthal( this->thrustValues_u, lqrRegulator.error, this->model, this->penalizers );
		// std::cout << "Q:\n"
		//           << static_cast< MatrixXd >( lqrRegulator.Q ) << "\nR:\n"
		//           << static_cast< MatrixXd >( lqrRegulator.R ) << "\nA:\n"
		//           << lqrRegulator.A << "\nB:\n"
		//           << lqrRegulator.B << "\ncurrent pos:\n"
		//           << this->currentPosition << "\ndesired pos:\n"
		//           << this->positionToReach << "\nthruster sig:\n"
		//           << this->thrustValues_u << std::endl;

#ifdef SIMULATION
		auto simulationFrequency = 1000.0f;
		for( auto i = 0u; i < simulationFrequency / this->regulatorWorkingFrequency; ++i )
		{
			this->calculateSimulationState( simulationFrequency );
			// simulation
			this->currentState = simulationResultState;
			currentPosition    = currentState.head< 6 >();
			currentSpeed       = currentState.tail< 6 >();
			// std::cout << "STAN: " << currentState << "\nPOZYCJA: " << currentPosition << "\nPREDKOSC: " <<
			// currentSpeed
			//   << std::endl;
		}
#ifndef NOLQR
		this->publishEstimatedPosition();
#endif
#endif

#ifndef MANUAL
		publishSignalsToHardware();
#endif
	}
}
void ThrusterRegulator::subscribeTopics()
{

	// position - for now from DVL, eventually from PositioningSLAM
	// TODO: change to SLAM
	this->rosSubscribers.emplace_back(
	    this->rosNode->subscribe( AUVROS::Topics::HardwareSignals::DVLDeadReckoningData,
	                              AUVROS::QueueSize::StandardQueueSize,
	                              &ThrusterRegulator::updateCurrentPositionAndAngularSpeed,
	                              this ) );

#ifdef NOLQR
	this->rosSubscribers.emplace_back( this->rosNode->subscribe( AUVROS::Topics::DevPC::arbitrarlySetThrustForce,
	                                                             AUVROS::QueueSize::StandardQueueSize,
	                                                             &ThrusterRegulator::updateDesiredForcesError,
	                                                             this ) );

#endif

	this->rosSubscribers.emplace_back( this->rosNode->subscribe( AUVROS::Topics::DevPC::arbitrarlySetGlobalPosition,
	                                                             AUVROS::QueueSize::StandardQueueSize,
	                                                             &ThrusterRegulator::updateTargetPosition,
	                                                             this ) );
}

void ThrusterRegulator::advertiseTopics()
{
	this->rosPublishers.emplace_back(
	    std::make_unique< ros::Publisher >( this->rosNode->advertise< AUVROS::MessageTypes::ThrustersSignal >(
	        AUVROS::Topics::HardwareSignals::signalToThrusters, AUVROS::QueueSize::StandardQueueSize ) ) );

	this->rosPublishers.emplace_back(
	    std::make_unique< ros::Publisher >( this->rosNode->advertise< AUVROS::MessageTypes::ServosSignal >(
	        AUVROS::Topics::HardwareSignals::signalToServos, AUVROS::QueueSize::StandardQueueSize ) ) );

#ifdef SIMULATION
#ifndef NOLQR
	this->rosPublishers.emplace_back(
	    std::make_unique< ros::Publisher >( this->rosNode->advertise< AUVROS::MessageTypes::Position >(
	        AUVROS::Topics::Positions::globalEstimatedPosition, AUVROS::QueueSize::StandardQueueSize ) ) );
#endif
#endif
}
void ThrusterRegulator::connectServices() {}

void ThrusterRegulator::loadRegulatorParameters( configFiles::fileID config )
{
	this->lqrRegulator              = jsonFunctions::regulator::readLQRData( config );
	this->penalizers                = jsonFunctions::regulator::readPenalizerData( config );
	this->regulatorWorkingFrequency = jsonFunctions::regulator::readWorkingFrequency( config );
	this->regulatorTickSpan         = static_cast< unsigned >( this->rosRate / this->regulatorWorkingFrequency );

	this->thrustValues_u = VectorXd::Zero( model.getModelThrusters().thrustersAmount, 1 );
}

void ThrusterRegulator::updateCurrentPositionAndAngularSpeed(
    const AUVROS::MessageTypes::DVLDeadReckoning& newPosition )
{
	auto newTimeStamp = std::chrono::steady_clock::now();
	auto deltaT       = std::chrono::duration< double >( newTimeStamp - this->timeStamp ).count();
	this->timeStamp   = newTimeStamp;
	currentPosition( 3 ) *= ( math::piNumber / 180.0 );
	currentPosition( 4 ) *= ( math::piNumber / 180.0 );
	currentPosition( 5 ) *= ( math::piNumber / 180.0 );
	// angular speed
	this->currentSpeed( 3 ) = ( newPosition.data.at( 3 ) - this->currentPosition( 3 ) ) / deltaT;
	this->currentSpeed( 4 ) = ( newPosition.data.at( 4 ) - this->currentPosition( 4 ) ) / deltaT;
	this->currentSpeed( 5 ) = ( newPosition.data.at( 5 ) - this->currentPosition( 5 ) ) / deltaT;

	// std::cout << "Pozycja:\n"
	//           << currentPosition( 0 ) << "\n"
	//           << currentPosition( 1 ) << "\n"
	//           << currentPosition( 2 ) << "\n"
	//           << currentPosition( 3 ) << "\n"
	//           << currentPosition( 4 ) << "\n"
	//           << currentPosition( 5 ) << "\n"
	//           << std::endl;

	// std::cout << "Pryndkosc:\n"
	//           << currentSpeed( 0 ) << "\n"
	//           << currentSpeed( 1 ) << "\n"
	//           << currentSpeed( 2 ) << "\n"
	//           << currentSpeed( 3 ) << "\n"
	//           << currentSpeed( 4 ) << "\n"
	//           << currentSpeed( 5 ) << "\n"
	//           << std::endl;

	// position
	this->currentPosition( 0 ) = newPosition.data.at( 0 );
	this->currentPosition( 1 ) = newPosition.data.at( 1 );
	this->currentPosition( 2 ) = newPosition.data.at( 2 );
	this->currentPosition( 3 ) = newPosition.data.at( 3 );
	this->currentPosition( 4 ) = newPosition.data.at( 4 );
	this->currentPosition( 5 ) = newPosition.data.at( 5 );

	this->currentState.block( 0, 0, 6, 1 ) = this->currentPosition;
	this->currentState.block( 6, 0, 6, 1 ) = this->currentSpeed;
}

void ThrusterRegulator::updateVelocity( const AUVROS::MessageTypes::DVLVelocity& newVelocity )
{
	this->currentSpeed( 0 )                = newVelocity.linear.x;
	this->currentSpeed( 1 )                = newVelocity.linear.y;
	this->currentSpeed( 2 )                = newVelocity.linear.z;
	this->currentState.block( 6, 0, 6, 1 ) = this->currentSpeed;
}

void allocateThrust2Azimuthal( VectorXd& thrustSignal_u,
                               const VectorXd& desiredForces_tau,
                               VehiclePhysicalModel& model,
                               const AllocationPenalizers& penalizers )
{
	VectorXd uPrev                         = thrustSignal_u;
	const auto& deltaA                     = model.getModelServos().servoSpeed;
	const auto& numberOfAzimuthalThrusters = model.getModelThrusters().numberOfAzimuthalThrusters;
	const auto& maxThrust                  = model.getModelThrusters().maxThrust;
	const auto& allThrustersConfig_T       = model.getModelThrusters().AllThrustersConfigurationsMatrix;
	const auto& servoMaxAngles             = model.getModelServos().servoAngleLimits;
	const auto& servoCurrentAngles         = model.getModelServos().servosAngles;
	const auto& azimuthalThrustersDifferentialConfig_Tdiff
	    = model.getModelThrusters().AzimuthalThrustersDifferentialConfig;
	// delta u which means how fast the force can grow in 1 timestep
	const auto& deltaU = model.getModelThrusters().deltaU;
	// change from [-1:1] to newtons
	uPrev *= maxThrust;

	// for now - only one pair of azimuthal thrusters is considered here working in the same plane and axis
	auto numberOfDims = model.getModelServos().azimuthalThrusterDimensionsOfInfluence.at( 0 ).second.size();

	// Diagonal matrix H which is main matrix in quadprog problem. x^T * H * X + f*X
	VectorXd diag_H = VectorXd::Zero( 13 );
	diag_H << 2.0 * penalizers.W, 2.0 * penalizers.Q, 2.0 * penalizers.Omega;
	MatrixXd H = MatrixXd::Zero( 13, 13 );
	H          = diag_H.asDiagonal();

	// Vector of linearity in quadprog as seen before
	VectorXd f = VectorXd::Zero( 13 );

	// Equality constraints for QP
	MatrixXd Aeq         = MatrixXd::Zero( 6, 13 );
	VectorXd tempOnesVec = VectorXd::Ones( sixDim );
	MatrixXd tempOnesMat = tempOnesVec.asDiagonal();

	Aeq.block< sixDim, 5 >( 0, 0 )      = allThrustersConfig_T;
	Aeq.block< sixDim, sixDim >( 0, 5 ) = tempOnesMat;
	Aeq.block< sixDim, 2 >( 0, 11 )     = azimuthalThrustersDifferentialConfig_Tdiff;
	Aeq.transposeInPlace();
	VectorXd Beq = VectorXd::Zero( sixDim );

	Beq = -( desiredForces_tau - allThrustersConfig_T * uPrev );

	// Inequality constraints
	// I need to specify lower and upper bounds for the variables
	// The difference between Matlab and this library is that in matlab the function looks like
	// lb < x < ub
	// Here it looks like
	// Ci^T * X + ci0 >= 0
	// So I needed to create a matrix Ci which gives vector of both +-u, +-s, +-alpha
	// And ci0 vector which corresponds to proper values of bounds
	MatrixXd lowerBoundary   = MatrixXd::Zero( 13, 1 );
	MatrixXd upperBoundary   = MatrixXd::Zero( 13, 1 );
	VectorXd temporaryVector = VectorXd::Ones( 13 );
	// The same as in Aeq - I pass tranposed version of matrix so I need to create temp_Ci matrix
	MatrixXd Ci      = MatrixXd::Zero( 13, 52 );
	MatrixXd temp_Ci = MatrixXd::Zero( 52, 13 );
	VectorXd ci0     = VectorXd::Zero( 52, 1 );
	lowerBoundary    = temporaryVector.asDiagonal(); // Lower bound
	upperBoundary    = -lowerBoundary;               // Upper bound

	MatrixXd lowerServoAnglesBoundary = MatrixXd::Zero( 13, 1 );
	MatrixXd upperServoAnglesBoundary = MatrixXd::Zero( 13, 1 );

	temporaryVector = VectorXd::Zero( 13, 1 );

	temporaryVector.tail( 2 ) = VectorXd::Ones( 2 );

	lowerServoAnglesBoundary = temporaryVector.asDiagonal();
	upperServoAnglesBoundary = -lowerServoAnglesBoundary;

	temp_Ci << lowerBoundary, upperBoundary, lowerServoAnglesBoundary, upperServoAnglesBoundary;
	Ci = temp_Ci.transpose();

	const auto infinity = std::numeric_limits< double >::max();

	ci0 << deltaU, deltaU, deltaU, deltaU, deltaU, infinity, infinity, infinity, infinity, infinity, infinity, deltaA,
	    deltaA, deltaU, deltaU, deltaU, deltaU, deltaU, infinity, infinity, infinity, infinity, infinity, infinity,
	    deltaA, deltaA, VectorXd::Zero( 11 ), -( servoMaxAngles.first - servoCurrentAngles.at( 0 ).first ),
	    -( servoMaxAngles.first - servoCurrentAngles.at( 1 ).first ), VectorXd::Zero( 11 ),
	    ( servoMaxAngles.second - servoCurrentAngles.at( 0 ).first ),
	    ( servoMaxAngles.second - servoCurrentAngles.at( 1 ).first ); // Vector of boundary values
	// std::cout << "=========BEFORE QUADPROG================\n";
	VectorXd quadProgSolution_x = VectorXd::Zero( 13 ); // Initializing solution vector
	// std::cout << "========PARAMETERS BEGIN'====================\n"
	//<< temp_Ci << "\n"
	//<< ci0 << "\n"
	//<< Aeq << "\n"
	//<< Beq << "\n" << std::endl;

	QP::solve_quadprog( H, f, Aeq, Beq, Ci, ci0, quadProgSolution_x );
	// std::cout<<"===========AFTER QUADPROG============\n";
	// std::cout << "========QUADPROG BEGIN=======================\n"
	//           << quadProgSolution_x << "\n========QUADPROG END===========\n"
	//           << std::endl;

	// std::cout << "====SERWA====\n"
	// 		  << servoCurrentAngles.at( 0 ).first << "\n"
	// 		  << servoCurrentAngles.at( 1 ).first << "\n"
	// 		  << std::endl;

	thrustSignal_u += quadProgSolution_x.head( 5 )
	    / maxThrust; // Adding values of calculated change in force. 5 is number of thrusters

	model.updateAzimuthalThrusterConfig( { servoCurrentAngles.at( 0 ).first + quadProgSolution_x( 11 ),
	                                       servoCurrentAngles.at( 1 ).first + quadProgSolution_x( 12 ) },
	                                     thrustSignal_u );

	// Making sure that we cannot demand 110% of power
	for( auto i{ 0u }; i < model.getModelThrusters().thrustersAmount; ++i )
	{
		if( thrustSignal_u( i ) > 1.0 )
		{
			thrustSignal_u( i ) = 1.0;
		}
		else if( thrustSignal_u( i ) < -1.0 )
		{
			thrustSignal_u( i ) = -1.0;
		}
	}
	// std::cout << "DONE\n" << quadProgSolution_x << std::endl;
}

void ThrusterRegulator::publishSignalsToHardware()
{
	AUVROS::MessageTypes::ThrustersSignal thrusterSignal;
	thrusterSignal.layout.data_offset = 0;
	std_msgs::MultiArrayDimension dimThrust;
	dimThrust.label  = "thrust";
	dimThrust.size   = 5;
	dimThrust.stride = 0;
	thrusterSignal.layout.dim.emplace_back( dimThrust );
	for( auto i = 0u; i < thrustValues_u.size(); ++i )
	{
		thrusterSignal.data.emplace_back( thrustValues_u( i ) );
	}
	this->rosPublishers.at( advertisers::signalToThrusters )->publish( thrusterSignal );

	AUVROS::MessageTypes::ThrustersSignal servoSignal;
	servoSignal.layout.data_offset = 0;
	std_msgs::MultiArrayDimension dimServos;
	dimServos.label  = "servos";
	dimServos.size   = 2;
	dimServos.stride = 0;
	servoSignal.layout.dim.emplace_back( dimServos );
	const auto& servos = this->model.getModelServos().servosAngles;
	for( auto i = 0u; i < servos.size(); ++i )
	{
		servoSignal.data.emplace_back( servos.at( i ).first );
	}
	this->rosPublishers.at( advertisers::signalToServos )->publish( servoSignal );
}

#ifdef SIMULATION
void ThrusterRegulator::calculateSimulationState( const float simulationFrequency )
{
	VectorXd tau = VectorXd::Zero( 6 );

	tau = this->model.getModelThrusters().AllThrustersConfigurationsMatrix * model.getModelThrusters().KMax
	    * this->thrustValues_u;
	this->simulationResultState = this->currentState
	    + ( this->lqrRegulator.A * currentState
	        + this->lqrRegulator.B * ( tau - model.getRestoringForces( this->currentState ) ) )
	        * ( 1.0 / simulationFrequency );
}
#ifndef NOLQR
void ThrusterRegulator::publishEstimatedPosition()
{
	AUVROS::MessageTypes::Position simResult;
	simResult.linear.x  = this->simulationResultState( 0 );
	simResult.linear.y  = this->simulationResultState( 1 );
	simResult.linear.z  = this->simulationResultState( 2 );
	simResult.angular.x = this->simulationResultState( 3 );
	simResult.angular.y = this->simulationResultState( 4 );
	simResult.angular.z = this->simulationResultState( 5 );

	this->rosPublishers.at( advertisers::estimatedPosition )->publish( simResult );

	// std::cout << this->simulationResultState << std::endl;
}
#else
void ThrusterRegulator::updateDesiredForcesError( const AUVROS::MessageTypes::arbitrarlySetThrustForce& newForces )
{
	this->lqrRegulator.error( 0 ) = newForces.linear.x;
	this->lqrRegulator.error( 1 ) = newForces.linear.y;
	this->lqrRegulator.error( 2 ) = newForces.linear.z;
	this->lqrRegulator.error( 3 ) = newForces.angular.x;
	this->lqrRegulator.error( 4 ) = newForces.angular.y;
	this->lqrRegulator.error( 5 ) = newForces.angular.z;
}
#endif
#endif

void ThrusterRegulator::updateTargetPosition( const AUVROS::MessageTypes::Position& newPos )
{
	this->positionToReach( 0 ) = newPos.linear.x;
	this->positionToReach( 1 ) = newPos.linear.y;
	this->positionToReach( 2 ) = newPos.linear.z;
	this->positionToReach( 3 ) = newPos.angular.x;
	this->positionToReach( 4 ) = newPos.angular.y;
	this->positionToReach( 5 ) = newPos.angular.z;
}