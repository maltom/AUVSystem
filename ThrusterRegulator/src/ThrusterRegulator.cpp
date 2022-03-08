#include "ThrusterRegulator.h"

#include <cassert>
#include <iostream>
#include <limits>
#include <vector>

#include "external/EigenQP/EigenQP.h"
#include "ROSEnums.h"

void ThrusterRegulator::processInMainLoop()
{

	if( ticks % regulatorTickSpan == 0 )
	{
		this->lqrRegulator.calculate( this->currentState, this->model );
		allocateThrust2Azimuthal( this->thrustValues_u, lqrRegulator.error, this->model, this->penalizers );
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
}

void ThrusterRegulator::advertiseTopics()
{
	this->rosPublishers.emplace_back(
	    std::make_unique< ros::Publisher >( this->rosNode->advertise< AUVROS::MessageTypes::ThrustersSignal >(
	        AUVROS::Topics::HardwareSignals::signalToThrusters, AUVROS::QueueSize::StandardQueueSize ) ) );

	this->rosPublishers.emplace_back(
	    std::make_unique< ros::Publisher >( this->rosNode->advertise< AUVROS::MessageTypes::ServosSignal >(
	        AUVROS::Topics::HardwareSignals::signalToServos, AUVROS::QueueSize::StandardQueueSize ) ) );
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
	auto newTimeStamp = newPosition.data.at( 6 );
	auto deltaT       = newTimeStamp - this->timeStamp;

	// angular speed
	this->currentSpeed( 3 ) = ( newPosition.data.at( 3 ) - this->currentSpeed( 3 ) ) / deltaT;
	this->currentSpeed( 4 ) = ( newPosition.data.at( 4 ) - this->currentSpeed( 4 ) ) / deltaT;
	this->currentSpeed( 5 ) = ( newPosition.data.at( 5 ) - this->currentSpeed( 5 ) ) / deltaT;

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
	// normalized - [0:1]

	// change from [-1:1] to newtons
	uPrev *= maxThrust;

	// for now - only one pair of azimuthal thrusters is considered here working in the same plane and axis
	auto numberOfDims = model.getModelServos().azimuthalThrusterDimensionsOfInfluence.at( 0 ).second.size();

	// Vector of desired forces and moments only in azimuthal dimensions
	VectorXd azimuthalDesiredForces_tau = VectorXd::Zero( numberOfDims, 1 );
	const auto& influences              = model.getModelServos().azimuthalThrusterDimensionsOfInfluence.at( 0 ).second;

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

	// TODO: ustalić czy spada z rowerka. jak spada, to znaczy, że trzeba transponować u
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

	ci0 << -deltaU, -deltaU, -deltaU, -deltaU, -deltaU, -infinity, -infinity, -infinity, -infinity, -infinity,
	    -infinity, -deltaA, -deltaA, deltaU, deltaU, deltaU, deltaU, deltaU, infinity, infinity, infinity, infinity,
	    infinity, infinity, deltaA, deltaA, VectorXd::Zero( 11 ),
	    -( servoMaxAngles.first - servoCurrentAngles.at( 0 ).first ),
	    -( servoMaxAngles.first - servoCurrentAngles.at( 1 ).first ), VectorXd::Zero( 11 ),
	    ( servoMaxAngles.second - servoCurrentAngles.at( 0 ).first ),
	    ( servoMaxAngles.second - servoCurrentAngles.at( 1 ).first ); // Vector of boundary values

	VectorXd quadProgSolution_x = VectorXd::Zero( 13 ); // Initializing solution vector

	QP::solve_quadprog( H, f, Aeq, Beq, Ci, ci0, quadProgSolution_x );

	thrustSignal_u += quadProgSolution_x.head( 5 )
	    / maxThrust; // Adding values of calculated change in force. 5 is number of thrusters

	model.updateAzimuthalThrusterConfig( { servoCurrentAngles.at( 0 ).first + quadProgSolution_x( 11 ),
	                                       servoCurrentAngles.at( 1 ).first + quadProgSolution_x( 12 ) } );

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
