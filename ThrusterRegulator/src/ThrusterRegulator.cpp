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
	this->regulatorWorkingFrequency = jsonFunctions::regulator::readWorkingFrequency( config );
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
	dampingCoeffs = model.getModelDrag().Dnl * speedDiag.cwiseAbs() + model.calculateCoriolisMatrix( currentState )
	    + model.getModelDrag().Dl;
	dampingCoeffs = ( model.getModelInertial().Mrb + model.getModelDrag().addedMass.Ma ).inverse() * dampingCoeffs;

	// State Space matrix
	A << MatrixXd::Zero( 6, 6 ), MatrixXd::Identity( 6, 6 ), MatrixXd::Zero( 6, 6 ), -dampingCoeffs;

	return A;
}

Matrix< double, 12, 6 > calculateBStateMatrix( const VehiclePhysicalModel& model )
{
	Matrix< double, 12, 6 > B = MatrixXd::Zero( 12, 6 );
	B.block( 6, 0, 6, 6 )
	    = ( model.getModelInertial().Mrb + model.getModelDrag().addedMass.Ma ).inverse() * MatrixXd::Identity( 6, 6 );
	return B;
}

void ThrusterRegulator::sendArbitrarlySetThrusters( const AUVROS::MessageTypes::ThrustersSignal& signal )
{}
void ThrusterRegulator::sendArbitrarlySetServos( const AUVROS::MessageTypes::ServosSignal& signal )
{}