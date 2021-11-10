#include "VehiclePhysicalModel.h"

#include <iostream>

#include "jsonCommonFunctions.h"

void VehiclePhysicalModel::loadPhysicalParameters( configFiles::fileID configID )
{
	this->inertialParams = jsonFunctions::vehicle::readInertialData( configID );
	this->thrusterParams = jsonFunctions::vehicle::readThrustersData( configID );
	this->dragParams     = jsonFunctions::vehicle::readDragData( configID );
	this->servos         = jsonFunctions::vehicle::readServosData( configID );

	this->thrusterParams.u = VectorXd::Zero( this->thrusterParams.thrustersAmount, 1 );
}

void VehiclePhysicalModel::adjustParametersForWorkingFrequency( float freq ) {}

void VehiclePhysicalModel::initMatrices()
{
	// Ib matrix
	this->inertialParams.Ib << this->inertialParams.Ix, -this->inertialParams.Ixy, -this->inertialParams.Ixz,
	    -this->inertialParams.Iyx, this->inertialParams.Iy, -this->inertialParams.Iyz, -this->inertialParams.Izx,
	    -this->inertialParams.Izy, this->inertialParams.Iz;

	// Mrb matrix
	this->inertialParams.Mrb.block( 0, 0, 3, 3 ) = this->inertialParams.mass * Matrix3d::Identity( 3, 3 );
	this->inertialParams.Mrb.block( 0, 3, 3, 3 )
	    = -this->inertialParams.mass * Smtrx( this->inertialParams.centerOfGravity );
	this->inertialParams.Mrb.block( 3, 0, 3, 3 )
	    = this->inertialParams.mass * Smtrx( this->inertialParams.centerOfGravity );
	this->inertialParams.Mrb.block( 3, 3, 3, 3 ) = this->inertialParams.Ib;

	// Ma matrix
	VectorXd MaDiag = VectorXd::Zero( 6 );
	MaDiag << this->dragParams.addedMass.Xua, this->dragParams.addedMass.Yva, this->dragParams.addedMass.Zwa,
	    this->dragParams.addedMass.Kpa, this->dragParams.addedMass.Mqa, this->dragParams.addedMass.Nra;

	this->dragParams.addedMass.Ma = ( -MaDiag ).asDiagonal();

	// Drag matrices

	VectorXd vl  = VectorXd::Zero( 6 );
	VectorXd vnl = VectorXd::Zero( 6 );

	vl << this->dragParams.linear.Xu, this->dragParams.linear.Yv, this->dragParams.linear.Zw,
	    this->dragParams.linear.Kp, this->dragParams.linear.Mq, this->dragParams.linear.Nr;
	vnl << this->dragParams.quadratic.Xuu, this->dragParams.quadratic.Yvv, this->dragParams.quadratic.Zww,
	    this->dragParams.quadratic.Kpp, this->dragParams.quadratic.Mqq, this->dragParams.quadratic.Nrr;

	this->dragParams.Dl  = ( -vl ).asDiagonal();
	this->dragParams.Dnl = ( -vnl ).asDiagonal();

	// Thrust Matrix
}

VectorXd VehiclePhysicalModel::getRestoringForces( const VectorXd& currentState ) const
{
	double th = currentState( 4 );
	double ph = currentState( 3 );

	VectorXd restoringForces = VectorXd::Zero( 6 );
	// tutaj jest mnozenie przez this->inertialParams.centerOfGravity( 2 ) - kiedys to bylo rg(2) - czy nie powinnismy
	// tez gdzies mnozyc tez przez wektor centerOfBuoyancy?
	// z reszta chyba za malo tych mnozen jest
	restoringForces << ( this->inertialParams.weight - this->inertialParams.buoyancy ) * sin( th ),
	    -( this->inertialParams.weight - this->inertialParams.buoyancy ) * cos( th ) * sin( ph ),
	    -( this->inertialParams.weight - this->inertialParams.buoyancy ) * cos( th ) * cos( ph ),
	    this->inertialParams.centerOfGravity( 2 ) * this->inertialParams.weight * cos( th ) * sin( ph ),
	    this->inertialParams.centerOfGravity( 2 ) * this->inertialParams.weight * sin( th ), 0.0;
	return restoringForces;
}

Matrix< double, 6, 6 > VehiclePhysicalModel::calculateCoriolisMatrix( const VectorXd& currentState ) const
{
	// Initializing parameters
	Matrix< double, 6, 6 > MrbTemp = Matrix< double, 6, 6 >::Zero();
	Matrix< double, 6, 6 > MaTemp  = Matrix< double, 6, 6 >::Zero();

	Matrix< double, 6, 6 > Crb   = Matrix< double, 6, 6 >::Zero( 6, 6 );
	Matrix< double, 6, 6 > Ca    = Matrix< double, 6, 6 >::Zero( 6, 6 );
	Matrix< double, 6, 1 > speed = Matrix< double, 6, 1 >::Zero( 6, 1 );

	Matrix3d M11 = Matrix3d::Zero( 3, 3 ), M12 = Matrix3d::Zero( 3, 3 ), M21 = Matrix3d::Zero( 3, 3 ),
	         M22   = Matrix3d::Zero( 3, 3 );
	Matrix3d M11_A = Matrix3d::Zero( 3, 3 ), M12_A = Matrix3d::Zero( 3, 3 ), M21_A = Matrix3d::Zero( 3, 3 ),
	         M22_A = Matrix3d::Zero( 3, 3 );
	Vector3d nu1 = Vector3d::Zero( 3 ), nu2 = Vector3d::Zero( 3 );

	// Splitting linear and angular velocities into 2 vectors
	speed = currentState.block( 6, 0, 6, 1 );
	nu1   = speed.block( 0, 0, 3, 1 );
	nu2   = speed.block( 3, 0, 3, 1 );

	// Creating sub-matrices derived from rigid-body mass matrix
	MrbTemp = 0.5 * ( this->inertialParams.Mrb + this->inertialParams.Mrb.transpose() ); // Making matrix square
	MaTemp  = 0.5 * ( this->dragParams.addedMass.Ma + this->dragParams.addedMass.Ma.transpose() );

	M11 = MrbTemp.topLeftCorner( 3, 3 );
	M12 = MrbTemp.topRightCorner( 3, 3 );
	M21 = M12.transpose();
	M22 = MrbTemp.bottomRightCorner( 3, 3 );

	M11_A = MaTemp.topLeftCorner( 3, 3 );
	M12_A = MaTemp.topRightCorner( 3, 3 );
	M21_A = M12_A.transpose();
	M22_A = MaTemp.bottomRightCorner( 3, 3 );

	// Creating Coriolis forces matrix
	Crb << Matrix3d::Zero( 3, 3 ), -Smtrx( M11 * nu1 + M12 * nu2 ), -Smtrx( M11 * nu1 + M12 * nu2 ),
	    -Smtrx( M21 * nu1 + M22 * nu2 );

	Ca << Matrix3d::Zero( 3, 3 ), -Smtrx( M11_A * nu1 + M12_A * nu2 ), -Smtrx( M11_A * nu1 + M12_A * nu2 ),
	    -Smtrx( M21_A * nu1 + M22_A * nu2 );

	return Crb + Ca;
}


// blagam o inna nazwe zamiast tego tau xD anyway, to bedzie musialo cale zostac przepisane raczej ze wzgledu na inne
// rozlozenie pednikow - zamiast tau - desiredForces albo desiredForces_tau
void VehiclePhysicalModel::allocateThrust( const VectorXd& tau )
{
	// Initializing thrust conf. matrix for azimuthal thrusters
	std::vector< MatrixXd > T;
	for( auto& in : T )
	{
		in = MatrixXd::Zero( 3, 1 );
	}

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

	// Cost matrices for quad prog
	VectorXd Q  = VectorXd::Zero( 3 ); // Penalizing the difference between desired tau and generated one
	VectorXd Om = VectorXd::Zero( 2 ); // Penalizing too fast turn rate - not really important
	VectorXd W  = VectorXd::Zero(
        2 ); // Penalizing the power consumption of motors. Not really important as it's taken care of in LQR
	Q << 1000.0, 1000.0, 1000.0;
	Om << 1.0, 1.0;
	W << 300.0, 300.0;

	// Diagonal matrix H which is main matrix in quadprog problem. x^T * H * X + f*X
	VectorXd diag_H = VectorXd::Zero( 7 );
	diag_H << 2.0 * W, 2.0 * Q, 2.0 * Om;
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
	VectorXd tau_c = VectorXd::Zero( 6 );
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

// return matrix of specific shape
Matrix3d Smtrx( const Eigen::Vector3d& r )
{
	Eigen::Matrix3d mtrx;
	mtrx << 0.0, -r( 2 ), r( 1 ), r( 2 ), 0.0, -r( 0 ), -r( 1 ), r( 0 ), 0.0;
	return mtrx;
}

MatrixXd calculateNbar( const Matrix< double, 12, 12 >& A,
                                        const Matrix< double, 12, 6 >& B,
                                        const Matrix< double, 6, 12 >& K )
{
	MatrixXd C     = MatrixXd::Identity( 12, 12 );
	MatrixXd scale = MatrixXd::Identity( 12, 6 );

	return -( C * ( A - B * K ).inverse() * B ).bdcSvd( ComputeThinU | ComputeThinV ).solve( scale );
}

Matrix< double, 12, 12 > calculateAStateMatrix( const VectorXd& currentState, const VehiclePhysicalModel& model )
{
	Matrix< double, 12, 12 > A           = MatrixXd::Zero( 12, 12 );
	Matrix< double, 6, 1 > speed         = MatrixXd::Zero( 6, 1 );
	Matrix< double, 6, 6 > dampingCoeffs = MatrixXd::Zero( 6, 6 );
	MatrixXd speedDiag                   = MatrixXd::Zero( 6, 6 );

	// Obtaining velocity vector and putting it as diagonal into a speed_diag matrix
	speed     = currentState.block( 6, 0, 6, 1 );
	speedDiag = speed.asDiagonal();

	// This definition can also be found in documentation
	// First I create damping_coeffs matrix which is the sum
	// Of all elements which create opposing forces
	// Then I divide it by -M matrix which comes from State Space equation
	dampingCoeffs = model.dragParams.Dnl * speedDiag.cwiseAbs() + model.calculateCoriolisMatrix( currentState )
	    + model.dragParams.Dl;
	dampingCoeffs = ( model.inertialParams.Mrb + model.dragParams.addedMass.Ma ).inverse() * dampingCoeffs;

	// State Space matrix
	A << MatrixXd::Zero( 6, 6 ), MatrixXd::Identity( 6, 6 ), MatrixXd::Zero( 6, 6 ), -dampingCoeffs;

	return A;
}

Matrix< double, 12, 6 > calculateBStateMatrix( const VehiclePhysicalModel& model )
{
	Matrix< double, 12, 6 > B = MatrixXd::Zero( 12, 6 );
	B.block( 6, 0, 6, 6 )
	    = ( model.inertialParams.Mrb + model.dragParams.addedMass.Ma ).inverse() * MatrixXd::Identity( 6, 6 );
	return B;
}