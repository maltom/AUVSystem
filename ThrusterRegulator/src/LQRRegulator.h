#pragma once

#include <memory>

#include <ct/optcon/optcon.h>
#include <Eigen/Dense>

#include "CommonEnums.h"

using namespace Eigen;

struct LQRRegulator
	{
		static constexpr auto stateDim   = 12u;
		static constexpr auto controlDim = 6u;

		Matrix< double, stateDim, stateDim > A   = Matrix< double, stateDim, stateDim >::Zero( stateDim, stateDim );
		Matrix< double, stateDim, controlDim > B = Matrix< double, stateDim, controlDim >::Zero( stateDim, controlDim );

		DiagonalMatrix< double, stateDim > Q;
		DiagonalMatrix< double, controlDim > R;
	};