#include "LQRRegulator.h"

#include <iostream>
void LQRRegulator::calculate( const VectorXd& currentState, const VehiclePhysicalModel& model )
{
	calculateAStateMatrix( currentState, model );
	calculateBStateMatrix( model );

	this->lqrSolver.compute( this->Q, this->R, this->A, this->B, this->K, true, true );
}

// for simulation purpose only
VectorXd LQRRegulator::calculateRegulatorFeedbackPose( const VectorXd& currentState )
{
	return -K * currentState;
}

void LQRRegulator::calculateError( const VectorXd& currentState, const VectorXd& positionToReach )
{
	this->error = calculateNbar() * positionToReach + calculateRegulatorFeedbackPose( currentState );
}

void LQRRegulator::calculateAStateMatrix( const VectorXd& currentState, const VehiclePhysicalModel& model )
{
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
	this->A << MatrixXd::Zero( sixDim, sixDim ), MatrixXd::Identity( sixDim, sixDim ), MatrixXd::Zero( sixDim, sixDim ),
	    -dampingCoeffs;
}

MatrixXd LQRRegulator::calculateNbar()
{
	MatrixXd C     = MatrixXd::Identity( stateDim, stateDim );
	MatrixXd scale = MatrixXd::Identity( stateDim, controlDim );

	MatrixXd dynamicA = this->A, dynamicB = this->B, dynamicK = this->K;

	return -( C * ( dynamicA - dynamicB * dynamicK ).inverse() * dynamicB )
	            .bdcSvd( ComputeThinU | ComputeThinV )
	            .solve( scale );
}

void LQRRegulator::calculateBStateMatrix( const VehiclePhysicalModel& model )
{
	this->B.block( controlDim, 0, controlDim, controlDim )
	    = ( model.getModelInertial().Mrb + model.getModelDrag().addedMass.Ma ).inverse()
	    * MatrixXd::Identity( controlDim, controlDim );
}