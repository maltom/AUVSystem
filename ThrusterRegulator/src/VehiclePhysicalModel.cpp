#include "VehiclePhysicalModel.h"

#include <algorithm>

#include "jsonCommonFunctions.h"

void VehiclePhysicalModel::loadPhysicalParameters( configFiles::fileID configID )
{
	this->inertialParams                                = jsonFunctions::vehicle::readInertialData( configID );
	this->thrusterParams                                = jsonFunctions::vehicle::readThrustersData( configID );
	this->dragParams                                    = jsonFunctions::vehicle::readDragData( configID );
	this->servos                                        = jsonFunctions::vehicle::readServosData( configID );
	this->thrusterParams.azimuthalThrustersConfigMatrix = this->getAzimuthalThrustersConfig();
}

void VehiclePhysicalModel::adjustParametersForWorkingFrequency( const float freq ) {}

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
	VectorXd MaDiag = VectorXd::Zero( sixDim );
	MaDiag << this->dragParams.addedMass.Xua, this->dragParams.addedMass.Yva, this->dragParams.addedMass.Zwa,
	    this->dragParams.addedMass.Kpa, this->dragParams.addedMass.Mqa, this->dragParams.addedMass.Nra;

	this->dragParams.addedMass.Ma = ( -MaDiag ).asDiagonal();

	// Drag matrices

	VectorXd vl  = VectorXd::Zero( sixDim );
	VectorXd vnl = VectorXd::Zero( sixDim );

	vl << this->dragParams.linear.Xu, this->dragParams.linear.Yv, this->dragParams.linear.Zw,
	    this->dragParams.linear.Kp, this->dragParams.linear.Mq, this->dragParams.linear.Nr;
	vnl << this->dragParams.quadratic.Xuu, this->dragParams.quadratic.Yvv, this->dragParams.quadratic.Zww,
	    this->dragParams.quadratic.Kpp, this->dragParams.quadratic.Mqq, this->dragParams.quadratic.Nrr;

	this->dragParams.Dl  = ( -vl ).asDiagonal();
	this->dragParams.Dnl = ( -vnl ).asDiagonal();

	// Thrust matrix
	// TODO: to musi byc odswiezane - trzeba dopisac funkcje osobna, ktora liczy i updateuje nie tylko macierz, ale tez
	// TODO: wektory w oparciu o funkcje, ktore musza byc zdefiniowane w pliku. dorob nowy atrybut klasy, ktory bedzie
	// TODO: przechowywal funkcje do przeliczenia wartosci (sin, cos, zero) jak i pochodne
	for( auto i = 0u; i < this->thrusterParams.thrustersAmount; ++i )
	{
		this->thrusterParams.AllThrustersConfigurationsMatrix.block< 6, 1 >( 0, i )
		    = this->thrusterParams.thrusterConfigurations.at( i );
	}
}

VectorXd VehiclePhysicalModel::getRestoringForces( const VectorXd& currentState ) const
{
	const auto& theta = currentState( 4 ); // pitch
	const auto& phi   = currentState( 3 ); // roll

	const auto& W = this->inertialParams.weight;
	const auto& B = this->inertialParams.buoyancy;

	const auto& xg = this->inertialParams.centerOfGravity( 0 );
	const auto& yg = this->inertialParams.centerOfGravity( 1 );
	const auto& zg = this->inertialParams.centerOfGravity( 2 );

	const auto& xb = this->inertialParams.centerOfBuoyancy( 0 );
	const auto& yb = this->inertialParams.centerOfBuoyancy( 1 );
	const auto& zb = this->inertialParams.centerOfBuoyancy( 2 );

	VectorXd restoringForces = VectorXd::Zero( sixDim );
	restoringForces << ( W - B ) * sin( theta ), -( W - B ) * cos( theta ) * sin( phi ),
	    -( W - B ) * cos( theta ) * cos( phi ),
	    -( yg * W - yb * B ) * cos( theta ) * cos( phi ) + ( zg * W - zb * B ) * cos( theta ) * sin( phi ),
	    ( zg * W - zb * B ) * sin( theta ) + ( xg * W - xb * B ) * cos( theta ) * cos( phi ),
	    -( xg * W - xb * B ) * cos( theta ) * sin( phi ) - ( yg * W - yb * B ) * sin( theta );
	return restoringForces;
}

MatrixXd VehiclePhysicalModel::getAzimuthalThrustersConfig()
{
	MatrixXd configMatrix;
	std::vector< MatrixXd > configVectors;
	for( auto i = 0u; i < this->servos.azimuthalThrusterDimensionsOfInfluence.size(); ++i )
	{
		const auto& thrusterNumber = this->servos.azimuthalThrusterDimensionsOfInfluence.at( i ).first;
		const auto& influences     = this->servos.azimuthalThrusterDimensionsOfInfluence.at( i ).second;
		const auto& thrusterConfig = this->thrusterParams.thrusterConfigurations.at( thrusterNumber );
		auto variant               = influences.size();
		configVectors.emplace_back( MatrixXd::Zero( variant, 1 ) );
		switch( variant )
		{
		case 1u:
			*configVectors.rbegin() << thrusterConfig( influences.at( 0 ) );
			break;
		case 2u:
			*configVectors.rbegin() << thrusterConfig( influences.at( 0 ) ), thrusterConfig( influences.at( 1 ) );
			break;
		case 3u:
			*configVectors.rbegin() << thrusterConfig( influences.at( 0 ) ), thrusterConfig( influences.at( 1 ) ),
			    thrusterConfig( influences.at( 2 ) );
			break;
		case 4u:
			*configVectors.rbegin() << thrusterConfig( influences.at( 0 ) ), thrusterConfig( influences.at( 1 ) ),
			    thrusterConfig( influences.at( 2 ) ), thrusterConfig( influences.at( 3 ) );
			break;
		case 5u:
			*configVectors.rbegin() << thrusterConfig( influences.at( 0 ) ), thrusterConfig( influences.at( 1 ) ),
			    thrusterConfig( influences.at( 2 ) ), thrusterConfig( influences.at( 3 ) ),
			    thrusterConfig( influences.at( 4 ) );
			break;
		case 6u:
			*configVectors.rbegin() << thrusterConfig( influences.at( 0 ) ), thrusterConfig( influences.at( 1 ) ),
			    thrusterConfig( influences.at( 2 ) ), thrusterConfig( influences.at( 3 ) ),
			    thrusterConfig( influences.at( 4 ) ), thrusterConfig( influences.at( 5 ) );
			break;
		default:
			break;
		}
	}
	auto maxVectorLength = 0u;
	for( const auto& in : configVectors )
	{
		maxVectorLength = std::max( maxVectorLength, static_cast< unsigned >( in.rows() ) );
	}
	configMatrix = MatrixXd::Zero( maxVectorLength, configVectors.size() );

	for( auto i = 0u; i < configMatrix.cols(); ++i )
	{
		configMatrix.block( 0, i, maxVectorLength, 1 ) = configVectors.at( i );
	}
	return configMatrix;
}

Matrix< double, sixDim, sixDim > VehiclePhysicalModel::calculateCoriolisMatrix( const VectorXd& currentState ) const
{
	// Initializing parameters
	Matrix< double, sixDim, sixDim > MrbTemp = Matrix< double, sixDim, sixDim >::Zero();
	Matrix< double, sixDim, sixDim > MaTemp  = Matrix< double, sixDim, sixDim >::Zero();

	Matrix< double, sixDim, sixDim > Crb = Matrix< double, sixDim, sixDim >::Zero();
	Matrix< double, sixDim, sixDim > Ca  = Matrix< double, sixDim, sixDim >::Zero();
	Matrix< double, sixDim, 1 > speed    = Matrix< double, sixDim, 1 >::Zero();

	Matrix3d M11 = Matrix3d::Zero( 3, 3 ), M12 = Matrix3d::Zero( 3, 3 ), M21 = Matrix3d::Zero( 3, 3 ),
	         M22   = Matrix3d::Zero( 3, 3 );
	Matrix3d M11_A = Matrix3d::Zero( 3, 3 ), M12_A = Matrix3d::Zero( 3, 3 ), M21_A = Matrix3d::Zero( 3, 3 ),
	         M22_A = Matrix3d::Zero( 3, 3 );
	Vector3d nu1 = Vector3d::Zero( 3 ), nu2 = Vector3d::Zero( 3 );

	// Splitting linear and angular velocities into 2 vectors
	speed = currentState.block( sixDim, 0, sixDim, 1 );
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

// return matrix of specific shape
Matrix3d Smtrx( const Eigen::Vector3d& r )
{
	Eigen::Matrix3d mtrx;
	mtrx << 0.0, -r( 2 ), r( 1 ), r( 2 ), 0.0, -r( 0 ), -r( 1 ), r( 0 ), 0.0;
	return mtrx;
}
