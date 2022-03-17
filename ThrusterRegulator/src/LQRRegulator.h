#pragma once

#include <memory>

#include <ct/optcon/optcon.h>
#include <Eigen/Dense>

#include "CommonEnums.h"
#include "VehiclePhysicalModel.h"

using namespace Eigen;
using namespace regulator;

struct LQRRegulator
{
	Matrix< double, stateDim, stateDim > A   = Matrix< double, stateDim, stateDim >::Zero( stateDim, stateDim );
	Matrix< double, stateDim, controlDim > B = Matrix< double, stateDim, controlDim >::Zero( stateDim, controlDim );

	DiagonalMatrix< double, stateDim > Q;
	DiagonalMatrix< double, controlDim > R;
	Matrix< double, controlDim, stateDim > K = Matrix< double, controlDim, stateDim >::Zero( controlDim, stateDim );

	// feedback error - desired forces
	VectorXd error = VectorXd::Zero( 6 );

	ct::optcon::LQR< stateDim, controlDim > lqrSolver;

	// for simulation purpose only
	VectorXd calculateRegulatorFeedbackPose( const VectorXd& currentState );

	void calculate( const VectorXd& currentState, const VehiclePhysicalModel& model );
	void calculateError( const VectorXd& currentState, const VectorXd& positionToReach );

private:
	MatrixXd calculateNbar();
	void calculateAStateMatrix( const VectorXd& currentState, const VehiclePhysicalModel& model );
	void calculateBStateMatrix( const VehiclePhysicalModel& model );
};

struct AllocationPenalizers
{
	VectorXd Q     = VectorXd::Zero( 6 ); // Penalizing the difference between desired tau and generated one
	VectorXd Omega = VectorXd::Zero( 2 ); // Penalizing too fast turn rate - not really important
	VectorXd W     = VectorXd::Zero(
        5 ); // Penalizing the power consumption of motors. Not really important as it's taken care of in LQR
};