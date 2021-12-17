#include "ThrusterRegulator.h"

#include <cassert>
#include <vector>

#include "external/EigenQP/EigenQP.h"
#include "ROSEnums.h"

void ThrusterRegulator::processInMainLoop()
{

	if( ticks % regulatorTickSpan == 0 )
	{
		ramp += deltaRamp;
		if( ramp >= 1.0f )
		{
			ramp = 0.0f;
		}
		AUVROS::MessageTypes::ThrustersSignal dummy;
		dummy.layout.dim.emplace_back( std_msgs::MultiArrayDimension() );

		dummy.data                     = { ramp, ramp, ramp, ramp, ramp };
		dummy.layout.dim.begin()->size = 5;
		rosPublishers.at( advertisers::thrustersArbitrarly )->publish( dummy );
	}
}
void ThrusterRegulator::subscribeTopics()
{
	this->rosSubscribers.emplace_back( this->rosNode->subscribe( AUVROS::Topics::DevPC::arbitrarlySetThrusters,
	                                                             AUVROS::QueueSize::StandardQueueSize,
	                                                             &ThrusterRegulator::sendArbitrarlySetThrusters,
	                                                             this ) );
	this->rosSubscribers.emplace_back( this->rosNode->subscribe( AUVROS::Topics::DevPC::arbitrarlySetServos,
	                                                             AUVROS::QueueSize::StandardQueueSize,
	                                                             &ThrusterRegulator::sendArbitrarlySetServos,
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

	this->rosPublishers.emplace_back(
	    std::make_unique< ros::Publisher >( this->rosNode->advertise< AUVROS::MessageTypes::ThrustersSignal >(
	        AUVROS::Topics::DevPC::arbitrarlySetThrusters, AUVROS::QueueSize::StandardQueueSize ) ) );

	this->rosPublishers.emplace_back(
	    std::make_unique< ros::Publisher >( this->rosNode->advertise< AUVROS::MessageTypes::ServosSignal >(
	        AUVROS::Topics::DevPC::arbitrarlySetServos, AUVROS::QueueSize::StandardQueueSize ) ) );
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
                               VectorXd& servoAngle_alpha,
                               const VectorXd& desiredForces_tau,
                               const VehiclePhysicalModel& model,
                               const AllocationPenalizers& penalizers )
{
	VectorXd uPrev = thrustSignal_u;
	for( auto i = 0u; i < model.getModelThrusters().numberOfAzimuthalThrusters; ++i )
	{
		thrustSignal_u( model.getModelServos().azimuthalThrusterDimensionsOfInfluence.at( i ).first )
		    *= model.getModelThrusters().maxThrust;
	}

	// for now - only one pair of azimuthal thrusters is considered here working in the same plane and axis
	auto numberOfDims = model.getModelServos().azimuthalThrusterDimensionsOfInfluence.at( 0 ).second.size();

	// Vector of desired forces and moments only in azimuthal dimensions
	VectorXd azimuthalDesiredForces_tau = VectorXd::Zero( numberOfDims, 1 );
	const auto& influences              = model.getModelServos().azimuthalThrusterDimensionsOfInfluence.at( 0 ).second;
	switch( influences.size() )
	{
	case 1u:
		azimuthalDesiredForces_tau << desiredForces_tau( influences.at( 0 ) );
		break;
	case 2u:
		azimuthalDesiredForces_tau << desiredForces_tau( influences.at( 0 ) ), desiredForces_tau( influences.at( 1 ) );
		break;
	case 3u:
		azimuthalDesiredForces_tau << desiredForces_tau( influences.at( 0 ) ), desiredForces_tau( influences.at( 1 ) ),
		    desiredForces_tau( influences.at( 2 ) );
		break;
	case 4u:
		azimuthalDesiredForces_tau << desiredForces_tau( influences.at( 0 ) ), desiredForces_tau( influences.at( 1 ) ),
		    desiredForces_tau( influences.at( 2 ) ), desiredForces_tau( influences.at( 3 ) );
		break;
	case 5u:
		azimuthalDesiredForces_tau << desiredForces_tau( influences.at( 0 ) ), desiredForces_tau( influences.at( 1 ) ),
		    desiredForces_tau( influences.at( 2 ) ), desiredForces_tau( influences.at( 3 ) ),
		    desiredForces_tau( influences.at( 4 ) );
		break;
	case 6u:
		azimuthalDesiredForces_tau << desiredForces_tau( influences.at( 0 ) ), desiredForces_tau( influences.at( 1 ) ),
		    desiredForces_tau( influences.at( 2 ) ), desiredForces_tau( influences.at( 3 ) ),
		    desiredForces_tau( influences.at( 4 ) ), desiredForces_tau( influences.at( 5 ) );
		break;
	default:
		break;
	}

	// delta u which means how fast the force can grow in 1 timestep
	double deltaU_max = model.getModelThrusters().deltaU / model.getModelThrusters().maxThrust;

	// Diagonal matrix H which is main matrix in quadprog problem. x^T * H * X + f*X
	VectorXd diag_H = VectorXd::Zero( 7 );
	diag_H << 2.0 * penalizers.W, 2.0 * penalizers.Q, 2.0 * penalizers.Omega;
	MatrixXd H = MatrixXd::Zero( 7, 7 );
	H          = diag_H.asDiagonal();

	// Vector of linearity in quadprog as seen before
	VectorXd f = VectorXd::Zero( 7 );

	// Calculating derivatives for linearization
	MatrixXd da1    = MatrixXd::Zero( azimuthalDesiredForces_tau.size(), 1 );
	MatrixXd da2    = MatrixXd::Zero( azimuthalDesiredForces_tau.size(), 1 );
	MatrixXd diff_T = MatrixXd::Zero( azimuthalDesiredForces_tau.size(), 2 );

	// First and second azimuthal thruster. Below are calculated derivatives of thrust. conf. matrices

	// Jka masz wektor sygnalow pednikow (u) to musisz go rozdzielic na 2 podwektory - taki co idzie do QP i taki co nie
	// idzie. tutaj brane bylo X,Y YAW z pednikow 1 i 2 do osobnych wektorow jako pochodne, dlatego przemnozone
	// jest przez te sinusy da1 << -sin( servoAngle_alpha( 0 ) ) * thrustSignal_u( 0 ), cos( servoAngle_alpha( 0 ) ) *
	// thrustSignal_u( 0 ),
	//     ( ( -0.165 * sin( servoAngle_alpha( 0 ) ) ) + ( 0.038 * cos( servoAngle_alpha( 0 ) ) ) ) * thrustSignal_u( 0
	//     );
	// da2 << -sin( servoAngle_alpha( 1 ) ) * thrustSignal_u( 1 ), cos( servoAngle_alpha( 1 ) ) * thrustSignal_u( 1 ),
	//     ( ( 0.165 * sin( servoAngle_alpha( 1 ) ) ) + ( 0.038 * cos( servoAngle_alpha( 1 ) ) ) ) * thrustSignal_u( 1
	//     );

	// TODO: zmien nazwe  da1 i da2, daj sprytne mnozenie przez pochodne funkcji kata
	da1 << -sin( servoAngle_alpha( 0 ) ) * thrustSignal_u( 0 ), cos( servoAngle_alpha( 0 ) ) * thrustSignal_u( 0 ),
	    ( ( -0.165 * sin( servoAngle_alpha( 0 ) ) ) + ( 0.038 * cos( servoAngle_alpha( 0 ) ) ) ) * thrustSignal_u( 0 );
	da2 << -sin( servoAngle_alpha( 1 ) ) * thrustSignal_u( 1 ), cos( servoAngle_alpha( 1 ) ) * thrustSignal_u( 1 ),
	    ( ( 0.165 * sin( servoAngle_alpha( 1 ) ) ) + ( 0.038 * cos( servoAngle_alpha( 1 ) ) ) ) * thrustSignal_u( 1 );

	diff_T << da1, da2;

	// Equality constraints for QP
	// In Matlab there is only Aeq and beq. Here I need to pass Aeq^T to the function so I calculate it's transpose
	MatrixXd Aeq      = MatrixXd::Zero( 7, 3 );
	MatrixXd temp_Aeq = MatrixXd::Zero( 3, 7 ); // Matrix which looks identical to that one from Matlab

	temp_Aeq.block( 0, 0, 3, 2 ) = model.getModelThrusters().azimuthalThrustersConfigMatrix;
	Vector3d v_diag( 1, 1, 1 );
	temp_Aeq.block( 0, 2, 3, 3 ) = v_diag.asDiagonal();
	temp_Aeq.block( 0, 5, 3, 2 ) = diff_T;

	Aeq = temp_Aeq.transpose();

	// Also the same as Matlab
	MatrixXd Beq;
	Beq = -( azimuthalDesiredForces_tau
	         - ( model.getModelThrusters().azimuthalThrustersConfigMatrix * thrustSignal_u.block( 0, 0, 2, 1 ) ) );

	// Inequality constraints
	// I need to specify lower and upper bounds for the variables
	// The difference between Matlab and this library is that in matlab the function looks like
	// lb < x < ub
	// Here it looks like
	// Ci^T * X + ci0 >= 0
	// So I needed to create a matrix Ci which gives vector of both +-u, +-s, +-alpha
	// And ci0 vector which corresponds to proper values of bounds
	MatrixXd Lb       = MatrixXd::Zero( 7, 7 );
	MatrixXd Ub       = MatrixXd::Zero( 7, 7 );
	VectorXd vec_ones = VectorXd::Zero( 7 );
	// The same as in Aeq - I pass tranposed version of matrix so I need to create temp_Ci matrix
	MatrixXd Ci      = MatrixXd::Zero( 7, 14 );
	MatrixXd temp_Ci = MatrixXd::Zero( 14, 7 );
	VectorXd ci0     = VectorXd::Zero( 14, 1 );
	vec_ones << 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0; // u,u,s,s,s,a,a
	Lb = vec_ones.asDiagonal();                    // Lower bound
	Ub = -Lb;                                      // Upper bound
	temp_Ci << Lb, Ub;
	Ci = temp_Ci.transpose();

	ci0 << -deltaU_max, -deltaU_max, 0.0, 0.0, 0.0, model.getModelServos().servoSpeed,
	    model.getModelServos().servoSpeed, deltaU_max, deltaU_max, 0.0, 0.0, 0.0, model.getModelServos().servoSpeed,
	    model.getModelServos().servoSpeed; // Vector of bound valuses

	VectorXd x = VectorXd::Zero( 7 ); // Initializing solution vector

	QP::solve_quadprog( H, f, Aeq, Beq, Ci, ci0, x );

	thrustSignal_u( 0 ) += x( 0 ); // Adding values of calculated change in force
	thrustSignal_u( 1 ) += x( 1 );

	thrustSignal_u( 0 ) /= model.getModelThrusters().maxThrust;
	thrustSignal_u( 1 ) /= model.getModelThrusters().maxThrust;

	servoAngle_alpha( 0 ) += x( 5 ); // And calculated change in servo angle
	servoAngle_alpha( 1 ) += x( 6 );

	// Classical THRUST ALLOCATION
	// Here I solve thrust allocation problem in classical way for forces in z,roll,pitch, for other 3 thrusters
	MatrixXd Thrust_conf = MatrixXd::Zero( 6, 3 ); // Matrix for only 3 thrusters
	MatrixXd Thrust_conf_inv;                      // Its pseudoinverse
	Thrust_conf << model.getModelThrusters().thrusterConfigurations.at( 2 ),
	    model.getModelThrusters().thrusterConfigurations.at( 3 ),
	    model.getModelThrusters().thrusterConfigurations.at( 4 );
	Thrust_conf_inv = Thrust_conf.completeOrthogonalDecomposition().pseudoInverse();

	// Matrix of maximum values of thrust force
	Vector3d diag_K(
	    model.getModelThrusters().maxThrust, model.getModelThrusters().maxThrust, model.getModelThrusters().maxThrust );
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

void ThrusterRegulator::sendArbitrarlySetThrusters( const AUVROS::MessageTypes::ThrustersSignal& signal ) {}
void ThrusterRegulator::sendArbitrarlySetServos( const AUVROS::MessageTypes::ServosSignal& signal ) {}