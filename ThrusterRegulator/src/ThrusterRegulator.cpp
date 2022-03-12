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
		

		
		// allocateThrust2Azimuthal( dummyThrustSignal, dummyForces, this->model, this->penalizers );
	}
}
void ThrusterRegulator::subscribeTopics() {}

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

	this->thrustValues_u    = VectorXd::Zero( model.getModelThrusters().thrustersAmount, 1 );
}

MatrixXd calculateNbar( const Matrix< double, stateDim, stateDim >& A,
                        const Matrix< double, stateDim, controlDim >& B,
                        const Matrix< double, controlDim, stateDim >& K )
{
	MatrixXd C     = MatrixXd::Identity( stateDim, stateDim );
	MatrixXd scale = MatrixXd::Identity( stateDim, controlDim );

	return -( C * ( A - B * K ).inverse() * B ).bdcSvd( ComputeThinU | ComputeThinV ).solve( scale );
}

Matrix< double, stateDim, stateDim > calculateAStateMatrix( const VectorXd& currentState,
                                                            const VehiclePhysicalModel& model )
{
	Matrix< double, stateDim, stateDim > A         = MatrixXd::Zero( stateDim, stateDim );
	Matrix< double, sixDim, 1 > speed              = MatrixXd::Zero( sixDim, 1 );
	Matrix< double, sixDim, sixDim > dampingCoeffs = MatrixXd::Zero( sixDim, sixDim );
	MatrixXd speedDiag                             = MatrixXd::Zero( sixDim, sixDim );

	// Obtaining velocity vector and putting it as diagonal into a speed_diag matrix
	speed     = currentState.block( sixDim, 0, sixDim, 1 );
	speedDiag = speed.asDiagonal();

	// This definition can also be found in documentation
	// First I create damping_coeffs matrix which is the sum
	// Of all elements which create opposing forces
	// Then I divide it by -M matrix which comes from State Space equation
	dampingCoeffs = model.getModelDrag().Dnl * speedDiag.cwiseAbs() + model.calculateCoriolisMatrix( currentState )
	    + model.getModelDrag().Dl;
	dampingCoeffs = ( model.getModelInertial().Mrb + model.getModelDrag().addedMass.Ma ).inverse() * dampingCoeffs;

	// State Space matrix
	A << MatrixXd::Zero( sixDim, sixDim ), MatrixXd::Identity( sixDim, sixDim ), MatrixXd::Zero( sixDim, sixDim ),
	    -dampingCoeffs;

	return A;
}

Matrix< double, stateDim, controlDim > calculateBStateMatrix( const VehiclePhysicalModel& model )
{
	Matrix< double, stateDim, controlDim > B = MatrixXd::Zero( stateDim, controlDim );
	B.block( controlDim, 0, controlDim, controlDim )
	    = ( model.getModelInertial().Mrb + model.getModelDrag().addedMass.Ma ).inverse()
	    * MatrixXd::Identity( controlDim, controlDim );
	return B;
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
