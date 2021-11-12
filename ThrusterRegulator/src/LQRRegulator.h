#pragma once

#include <memory>

#include <ct/optcon/optcon.h>
#include <Eigen/Dense>

#include "CommonEnums.h"

using namespace Eigen;
using namespace regulator;

struct LQRRegulator
{
	Matrix< double, stateDim, stateDim > A   = Matrix< double, stateDim, stateDim >::Zero( stateDim, stateDim );
	Matrix< double, stateDim, controlDim > B = Matrix< double, stateDim, controlDim >::Zero( stateDim, controlDim );

	DiagonalMatrix< double, stateDim > Q;
	DiagonalMatrix< double, controlDim > R;
};

struct AllocationPenalizers
{
	VectorXd Q     = VectorXd::Zero( 3 ); // Penalizing the difference between desired tau and generated one
	VectorXd Omega = VectorXd::Zero( 2 ); // Penalizing too fast turn rate - not really important
	VectorXd W     = VectorXd::Zero(
        2 ); // Penalizing the power consumption of motors. Not really important as it's taken care of in LQR
};