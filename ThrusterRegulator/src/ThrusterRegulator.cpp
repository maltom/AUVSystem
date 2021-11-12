#include "ThrusterRegulator.h"
#include "ROSEnums.h"

void ThrusterRegulator::processInMainLoop() {}
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
}
void ThrusterRegulator::connectServices() {}

void ThrusterRegulator::loadRegulatorParameters( configFiles::fileID config )
{
	this->lqrRegulator              = jsonFunctions::regulator::readLQRData( config );
	this->penalizers                = jsonFunctions::regulator::readPenalizerData( config );
	this->regulatorWorkingFrequency = jsonFunctions::regulator::readWorkingFrequency( config );
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

void allocateThrust( const VectorXd& desiredForces_tau,
                     const VehiclePhysicalModel& model,
                     const AllocationPenalizers& penalizers )
{
	// Initializing thrust conf. matrix for azimuthal thrusters

	MatrixXd T1 = MatrixXd::Zero( 3, 1 );
	MatrixXd T2 = MatrixXd::Zero( 3, 1 );
	MatrixXd T_azimuth
	    = MatrixXd::Zero( 3, 2 ); // Thrust conf. matrix for 2 azimuthal thrusters, including only x,y,yaw forces
	VectorXd tau_desired = VectorXd::Zero( 3, 1 ); // Vector of desired forces  and moments: x,y,yaw
	T1 << t1( 0 ), t1( 1 ), t1( 5 );               // t1 and t2 are global thrust conf. matrices including sin and cos
	T2 << t2( 0 ), t2( 1 ), t2( 5 );
	T_azimuth << T1, T2;

	VectorXd uPrev;
	uPrev  = u;
	u( 0 ) = u( 0 ) * 40.0;
	u( 1 ) = u( 1 ) * 40.0;

	tau_desired << tau( 0 ), tau( 1 ), tau( 5 );

	// Constraints
	double delta_a = 0.03; // Speed of servo - the angle which it turns by in 1 timestep 0.015 for 0.005deltaT

	// to jest to samo co deltaU, do wywalenia
	double u_min = -0.4; // delta u which means how fast the force can grow in 1 timestep
	double u_max = 0.4;

	// Diagonal matrix H which is main matrix in quadprog problem. x^T * H * X + f*X
	VectorXd diag_H = VectorXd::Zero( 7 );
	diag_H << 2.0 * penalizers.W, 2.0 * penalizers.Q, 2.0 * penalizers.Omega;
	MatrixXd H = MatrixXd::Zero( 7, 7 );
	H          = diag_H.asDiagonal();

	// Vector of linearity in quadprog as seen before
	VectorXd f = VectorXd::Zero( 7 );

	// Calculating derivatives for linearization
	MatrixXd da1    = MatrixXd::Zero( 3, 1 );
	MatrixXd da2    = MatrixXd::Zero( 3, 1 );
	MatrixXd diff_T = MatrixXd::Zero( 3, 2 );

	// First and second azimuthal thruster. Below are calculated derivatives of thrust. conf. matrices
	da1 << -sin( alpha01 ) * u( 0 ), cos( alpha01 ) * u( 0 ),
	    ( ( -0.165 * sin( alpha01 ) ) + ( 0.038 * cos( alpha01 ) ) ) * u( 0 );
	da2 << -sin( alpha02 ) * u( 1 ), cos( alpha02 ) * u( 1 ),
	    ( ( 0.165 * sin( alpha02 ) ) + ( 0.038 * cos( alpha02 ) ) ) * u( 1 );

	diff_T << da1, da2;

	// Equality constraints for QP
	// In Matlab there is only Aeq and beq. Here I need to pass Aeq^T to the function so I calculate it's transpose
	MatrixXd Aeq      = MatrixXd::Zero( 7, 3 );
	MatrixXd temp_Aeq = MatrixXd::Zero( 3, 7 ); // Matrix which looks identical to that one from Matlab

	temp_Aeq.block( 0, 0, 3, 2 ) = T_azimuth;
	Vector3d v_diag( 1, 1, 1 );
	temp_Aeq.block( 0, 2, 3, 3 ) = v_diag.asDiagonal();
	temp_Aeq.block( 0, 5, 3, 2 ) = diff_T;

	Aeq = temp_Aeq.transpose();

	// Also the same as Matlab
	MatrixXd Beq;
	Beq = -( tau_desired - ( T_azimuth * u.block( 0, 0, 2, 1 ) ) );

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

	ci0 << -u_min, -u_min, 0.0, 0.0, 0.0, delta_a, delta_a, u_max, u_max, 0.0, 0.0, 0.0, delta_a,
	    delta_a; // Vector of bound valuses

	VectorXd x = VectorXd::Zero( 7 ); // Initializing solution vector

	QP::solve_quadprog( H, f, Aeq, Beq, Ci, ci0, x );

	// Solving and printing quadprog
	//    std::cout << "Solve quadprog:" << QP::solve_quadprog(H,f,Aeq,Beq,Ci,ci0,x) << std::endl;
	//    std::cout << "x= " << std::endl << x << std::endl;

	u( 0 ) += x( 0 ); // Adding values of calculated change in force
	u( 1 ) += x( 1 );

	u( 0 ) = u( 0 ) / 40.0;
	u( 1 ) = u( 1 ) / 40.0;

	alpha01 += x( 5 ); // And calculated change in servo angle
	alpha02 += x( 6 );

	// Classical THRUST ALLOCATION
	// Here I solve thrust allocation problem in classical way for forces in z,roll,pitch, for other 3 thrusters
	MatrixXd Thrust_conf = MatrixXd::Zero( 6, 3 ); // Matrix for only 3 thrusters
	MatrixXd Thrust_conf_inv;                      // Its pseudoinverse
	Thrust_conf << t3, t4, t5;
	Thrust_conf_inv = Thrust_conf.completeOrthogonalDecomposition().pseudoInverse();

	// Matrix of maximum values of thrust force
	Vector3d diag_K( 40.0, 40.0, 40.0 );
	Matrix3d K;
	K = diag_K.asDiagonal();

	// Desired tau for this thrust allocation
	VectorXd tau_c = VectorXd::Zero( sixDim );
	tau_c << 0.0, 0.0, tau( 2 ), tau( 3 ), tau( 4 ), 0.0;

	// Final calculated vector of control signal
	Vector3d u2 = Vector3d::Zero( 3 );
	u2          = K.inverse() * Thrust_conf_inv * tau_c;

	// Final vector u which is vector of all control signals for all thrusters
	u( 2 ) = u2( 0 );
	u( 3 ) = u2( 1 );
	u( 4 ) = u2( 2 );

	// Adding some inertia to the thrusters
	for( int i = 0; i <= 4; i++ )
	{
		if( ( u( i ) - uPrev( i ) ) > deltaU )
		{
			u( i ) = uPrev( i ) + deltaU;
		}
		else if( ( u( i ) - uPrev( i ) ) < -deltaU )
		{
			u( i ) = uPrev( i ) - deltaU;
		}
	}

	// Making sure that we cannot demand 110% of power
	for( int i = 0; i <= 4; i++ )
	{
		if( u( i ) > 1.0 )
		{
			u( i ) = 1.0;
		}
		else if( u( i ) < -1.0 )
		{
			u( i ) = -1.0;
		}
	}

	// std::cout << "Alpha 01: " << alpha01 << " alpha 02: " << alpha02 << std::endl;
	// std::cout << "u = " << u << std::endl;
}

void ThrusterRegulator::sendArbitrarlySetThrusters( const AUVROS::MessageTypes::ThrustersSignal& signal ) {}
void ThrusterRegulator::sendArbitrarlySetServos( const AUVROS::MessageTypes::ServosSignal& signal ) {}