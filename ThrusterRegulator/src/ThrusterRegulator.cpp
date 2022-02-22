#include "ThrusterRegulator.h"

#include <cassert>
#include <vector>

#include "external/EigenQP/EigenQP.h"
#include "ROSEnums.h"

void ThrusterRegulator::processInMainLoop()
{

	if( ticks % regulatorTickSpan == 0 )
	{
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
	this->servoAngles_alpha = VectorXd::Zero( model.getModelThrusters().numberOfAzimuthalThrusters, 1 );
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
                               const VehiclePhysicalModel& model,
                               const AllocationPenalizers& penalizers )
{
	VectorXd uPrev                         = thrustSignal_u;
	const auto& deltaA                     = model.getModelServos().servoSpeed;
	const auto& numberOfAzimuthalThrusters = model.getModelThrusters().numberOfAzimuthalThrusters;
	const auto& maxThrust                  = model.getModelThrusters().maxThrust;
	const auto& allThrustersConfig_T       = model.getModelThrusters().AllThrustersConfigurationsMatrix;
	const auto& azimuthalThrustersDifferentialConfig_Tdiff
	    = model.getModelThrusters().AzimuthalThrustersDifferentialConfig;
	// delta u which means how fast the force can grow in 1 timestep
	const auto& deltaU = model.getModelThrusters().deltaU;
	// normalized - [0:1]
	auto deltaUNormalized = deltaU / model.getModelThrusters().maxThrust;

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

	VectorXd Beq = VectorXd::Zero( sixDim );

	// TODO: ustalić czy spada z rowerka. jak spada, to znaczy, że trzeba transponować u
	Beq = -( desiredForces_tau - allThrustersConfig_T * thrustSignal_u );

	// Inequality constraints
	// I need to specify lower and upper bounds for the variables
	// The difference between Matlab and this library is that in matlab the function looks like
	// lb < x < ub
	// Here it looks like
	// Ci^T * X + ci0 >= 0
	// So I needed to create a matrix Ci which gives vector of both +-u, +-s, +-alpha
	// And ci0 vector which corresponds to proper values of bounds
	VectorXd lowerBoundary = MatrixXd::Zero( 13, 1 );
	MatrixXd upperBoundary = MatrixXd::Zero( 13, 1 );
	VectorXd vec_ones      = VectorXd::Ones( 13 );
	// The same as in Aeq - I pass tranposed version of matrix so I need to create temp_Ci matrix
	MatrixXd Ci      = MatrixXd::Zero( 13, 26 );
	MatrixXd temp_Ci = MatrixXd::Zero( 26, 13 );
	VectorXd ci0     = VectorXd::Zero( 26, 1 );
	lowerBoundary = vec_ones.asDiagonal();                    // Lower bound
	upperBoundary = -lowerBoundary;                                      // Upper bound
	temp_Ci << lowerBoundary, upperBoundary;
	Ci = temp_Ci.transpose();
	// TE MAGIC NUMBERS 128 POWINNY BYĆ JAKIMIŚ W KURWE DUŻYMI LICZBAMI. W MATLABIE MAM +- INF A TU NIE WIEM CO DAĆ
	ci0 << -deltaUNormalized, -deltaUNormalized,-deltaUNormalized, -deltaUNormalized, -deltaUNormalized, -128, -128, -128, -128, -128, -128, -deltaA, -deltaA,
		deltaUNormalized, deltaUNormalized,deltaUNormalized, deltaUNormalized, deltaUNormalized, 128, 128, 128, 128, 128, 128, deltaA, deltaA; // Vector of bound valuses

	VectorXd x = VectorXd::Zero( 13 ); // Initializing solution vector

	QP::solve_quadprog( H, f, Aeq, Beq, Ci, ci0, x );

	thrustSignal_u += x.head(5); // Adding values of calculated change in force. 5 is number of thrusters

	thrustSignal_u /= maxThrust;

	servoAngle_alpha( 0 ) += x( 11 ); // And calculated change in servo angle
	servoAngle_alpha( 1 ) += x( 12 );

	//DO WYPIERDOLENIA W SUMIE
	// Classical THRUST ALLOCATION
	// Here I solve thrust allocation problem in classical way for forces in z,roll,pitch, for other 3 thrusters
	MatrixXd Thrust_conf = MatrixXd::Zero( 6, 3 ); // Matrix for only 3 thrusters
	MatrixXd Thrust_conf_inv;                      // Its pseudoinverse
	Thrust_conf << model.getModelThrusters().thrusterConfigurations.at( 2 ),
	    model.getModelThrusters().thrusterConfigurations.at( 3 ),
	    model.getModelThrusters().thrusterConfigurations.at( 4 );
	Thrust_conf_inv = Thrust_conf.completeOrthogonalDecomposition().pseudoInverse();

	// Matrix of maximum values of thrust force
	Vector3d diag_K( maxThrust, maxThrust, maxThrust );
	Matrix3d K;
	K = diag_K.asDiagonal();

	// Desired tau for this thrust allocation
	VectorXd tau_c = VectorXd::Zero( sixDim );
	tau_c << 0.0, 0.0, desiredForces_tau( 2 ), desiredForces_tau( 3 ), desiredForces_tau( 4 ), 0.0;

	// Final calculated vector of control signal
	Vector3d u2 = Vector3d::Zero( 3 );
	u2          = K.inverse() * Thrust_conf_inv * tau_c;

	// Final vector u which is vector of all control signals for all thrusters
	thrustSignal_u( 2 ) = u2( 0 );
	thrustSignal_u( 3 ) = u2( 1 );
	thrustSignal_u( 4 ) = u2( 2 );

	// Adding some inertia to the thrusters
	for( int i = 0; i < model.getModelThrusters().thrustersAmount; i++ )
	{
		if( ( thrustSignal_u( i ) - uPrev( i ) ) > deltaU_max )
		{
			thrustSignal_u( i ) = uPrev( i ) + deltaU_max;
		}
		else if( ( thrustSignal_u( i ) - uPrev( i ) ) < -deltaU_max )
		{
			thrustSignal_u( i ) = uPrev( i ) - deltaU_max;
		}
	}

	// Making sure that we cannot demand 110% of power
	for( int i = 0; i < model.getModelThrusters().thrustersAmount; i++ )
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

	// std::cout << "Alpha 01: " << alpha01 << " alpha 02: " << alpha02 << std::endl;
	// std::cout << "u = " << u << std::endl;
}
