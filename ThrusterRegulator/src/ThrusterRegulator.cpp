#include "ThrusterRegulator.h"
#include "ROSEnums.h"

void ThrusterRegulator::startMainLoop()
{
	while( ros::ok() )
	{
		ros::spinOnce();
		rosLoopRate->sleep();
	}
}

void ThrusterRegulator::subscribeTopics()
{
	// this->globalEstimatedPositionPublisher =  this->rosNode->advertise< std_msgs::Float32 >(
	// TopicsAndServicesNames::Topics::globalEstimatedPosition, 1000 );
}
void ThrusterRegulator::advertiseTopics() const {}
void ThrusterRegulator::connectServices() const {}

void ThrusterRegulator::loadRegulatorParameters( configFiles::fileID config )
{
	this->lqrRegulator = jsonFunctions::regulator::readLQRData( config );
	this->regulatorWorkingFrequency = jsonFunctions::regulator::readWorkingFrequency( config );
}

// void ThrusterRegulator::allocateThrust( )
// {
//     // Initializing thrust conf. matrix for azimuthal thrusters
//     MatrixXd T1 = MatrixXd::Zero( 3, 1 );
//     MatrixXd T2 = MatrixXd::Zero( 3, 1 );
//     MatrixXd T_azimuth
//         = MatrixXd::Zero( 3, 2 ); // Thrust conf. matrix for 2 azimuthal thrusters, including only x,y,yaw forces
//     VectorXd tau_desired = VectorXd::Zero( 3, 1 ); // Vector of desired forces  and moments: x,y,yaw
//     T1 << t1( 0 ), t1( 1 ), t1( 5 );               // t1 and t2 are global thrust conf. matrices including sin and cos
//     T2 << t2( 0 ), t2( 1 ), t2( 5 );
//     T_azimuth << T1, T2;

//     VectorXd uPrev;
//     uPrev  = u;
//     u( 0 ) = u( 0 ) * 40.0;
//     u( 1 ) = u( 1 ) * 40.0;

//     tau_desired << tau( 0 ), tau( 1 ), tau( 5 );

//     // Constraints
//     double delta_a = 3; // Speed of servo - the angle which it turns by in 1 timestep 0.015 for 0.005deltaT

//     double u_min = -400.0; // delta u which means how fast the force can grow in 1 timestep
//     double u_max = 400.0;

//     // Cost matrices for quad prog
//     VectorXd Q  = VectorXd::Zero( 3 ); // Penalizing the difference between desired tau and generated one
//     VectorXd Om = VectorXd::Zero( 2 ); // Penalizing too fast turn rate - not really important
//     VectorXd W  = VectorXd::Zero(
//         2 ); // Penalizing the power consumption of motors. Not really important as it's taken care of in LQR
//     Q << 1000.0, 1000.0, 1000.0;
//     Om << 1.0, 1.0;
//     W << 300.0, 300.0;

//     // Diagonal matrix H which is main matrix in quadprog problem. x^T * H * X + f*X
//     VectorXd diag_H = VectorXd::Zero( 7 );
//     diag_H << 2.0 * W, 2.0 * Q, 2.0 * Om;
//     MatrixXd H = MatrixXd::Zero( 7, 7 );
//     H          = diag_H.asDiagonal();

//     // Vector of linearity in quadprog as seen before
//     VectorXd f = VectorXd::Zero( 7 );

//     // Calculating derivatives for linearization
//     MatrixXd da1    = MatrixXd::Zero( 3, 1 );
//     MatrixXd da2    = MatrixXd::Zero( 3, 1 );
//     MatrixXd diff_T = MatrixXd::Zero( 3, 2 );

//     // First and second azimuthal thruster. Below are calculated derivatives of thrust. conf. matrices
//     da1 << -sin( alpha01 ) * u( 0 ), cos( alpha01 ) * u( 0 ),
//         ( ( -0.165 * sin( alpha01 ) ) + ( 0.038 * cos( alpha01 ) ) ) * u( 0 );
//     da2 << -sin( alpha02 ) * u( 1 ), cos( alpha02 ) * u( 1 ),
//         ( ( 0.165 * sin( alpha02 ) ) + ( 0.038 * cos( alpha02 ) ) ) * u( 1 );

//     diff_T << da1, da2;

//     // Equality constraints for QP
//     // In Matlab there is only Aeq and beq. Here I need to pass Aeq^T to the function so I calculate it's transpose
//     MatrixXd Aeq      = MatrixXd::Zero( 7, 3 );
//     MatrixXd temp_Aeq = MatrixXd::Zero( 3, 7 ); // Matrix which looks identical to that one from Matlab

//     temp_Aeq.block( 0, 0, 3, 2 ) = T_azimuth;
//     Vector3d v_diag( 1, 1, 1 );
//     temp_Aeq.block( 0, 2, 3, 3 ) = v_diag.asDiagonal();
//     temp_Aeq.block( 0, 5, 3, 2 ) = diff_T;

//     Aeq = temp_Aeq.transpose();

//     // Also the same as Matlab
//     MatrixXd Beq;
//     Beq = -( tau_desired - ( T_azimuth * u.block( 0, 0, 2, 1 ) ) );

//     // Inequality constraints
//     // I need to specify lower and upper bounds for the variables
//     // The difference between Matlab and this library is that in matlab the function looks like
//     // lb < x < ub
//     // Here it looks like
//     // Ci^T * X + ci0 >= 0
//     // So I needed to create a matrix Ci which gives vector of both +-u, +-s, +-alpha
//     // And ci0 vector which corresponds to proper values of bounds
//     MatrixXd Lb       = MatrixXd::Zero( 7, 7 );
//     MatrixXd Ub       = MatrixXd::Zero( 7, 7 );
//     VectorXd vec_ones = VectorXd::Zero( 7 );
//     // The same as in Aeq - I pass tranposed version of matrix so I need to create temp_Ci matrix
//     MatrixXd Ci      = MatrixXd::Zero( 7, 14 );
//     MatrixXd temp_Ci = MatrixXd::Zero( 14, 7 );
//     VectorXd ci0     = VectorXd::Zero( 14, 1 );
//     vec_ones << 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0; // u,u,s,s,s,a,a
//     Lb = vec_ones.asDiagonal();                    // Lower bound
//     Ub = -Lb;                                      // Upper bound
//     temp_Ci << Lb, Ub;
//     Ci = temp_Ci.transpose();

//     ci0 << -u_min, -u_min, 0.0, 0.0, 0.0, delta_a, delta_a, u_max, u_max, 0.0, 0.0, 0.0, delta_a,
//         delta_a; // Vector of bound valuses

//     VectorXd x = VectorXd::Zero( 7 ); // Initializing solution vector

//     QP::solve_quadprog( H, f, Aeq, Beq, Ci, ci0, x );

//     // Solving and printing quadprog
//     //    std::cout << "Solve quadprog:" << QP::solve_quadprog(H,f,Aeq,Beq,Ci,ci0,x) << std::endl;
//     //    std::cout << "x= " << std::endl << x << std::endl;

//     u( 0 ) += x( 0 ); // Adding values of calculated change in force
//     u( 1 ) += x( 1 );

//     u( 0 ) = u( 0 ) / 40.0;
//     u( 1 ) = u( 1 ) / 40.0;

//     alpha01 += x( 5 ); // And calculated change in servo angle
//     alpha02 += x( 6 );

//     // Classical THRUST ALLOCATION
//     // Here I solve thrust allocation problem in classical way for forces in z,roll,pitch, for other 3 thrusters
//     MatrixXd Thrust_conf = MatrixXd::Zero( 6, 3 ); // Matrix for only 3 thrusters
//     MatrixXd Thrust_conf_inv;                      // Its pseudoinverse
//     Thrust_conf << t3, t4, t5;
//     Thrust_conf_inv = Thrust_conf.completeOrthogonalDecomposition().pseudoInverse();

//     // Matrix of maximum values of thrust force
//     Vector3d diag_K( 40.0, 40.0, 40.0 );
//     Matrix3d K;
//     K = diag_K.asDiagonal();

//     // Desired tau for this thrust allocation
//     VectorXd tau_c = VectorXd::Zero( 6 );
//     tau_c << 0.0, 0.0, tau( 2 ), tau( 3 ), tau( 4 ), 0.0;

//     // Final calculated vector of control signal
//     Vector3d u2 = Vector3d::Zero( 3 );
//     u2          = K.inverse() * Thrust_conf_inv * tau_c;

//     // Final vector u which is vector of all control signals for all thrusters
//     u( 2 ) = u2( 0 );
//     u( 3 ) = u2( 1 );
//     u( 4 ) = u2( 2 );

//     // Adding some inertia to the thrusters
//     for( int i = 0; i < 5; i++ )
//     {
//         if( ( u( i ) - uPrev( i ) ) > deltaU )
//         {
//             u( i ) = uPrev( i ) + deltaU;
//         }
//         else if( ( u( i ) - uPrev( i ) ) < -deltaU )
//         {
//             u( i ) = uPrev( i ) - deltaU;
//         }
//     }

//     for( int i = 0; i < 5; i++ )
//     {
//         if( u( i ) > 1.0 )
//         {
//             u( i ) = 1.0;
//         }
//         else if( u( i ) < -1.0 )
//         {
//             u( i ) = -1.0;
//         }
//     }

//     if( alpha01 > angleConstraint )
//         alpha01 = angleConstraint;
//     else if( alpha01 < -angleConstraint )
//         alpha01 = -angleConstraint;

//     if( alpha02 > angleConstraint )
//         alpha02 = angleConstraint;
//     else if( alpha02 < -angleConstraint )
//         alpha02 = -angleConstraint;
// }