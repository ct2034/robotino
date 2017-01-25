#include "Transform.h"
#include "MyCom.h"
#include <rec/math/utils.h>

using namespace rec::math;

Transform::Transform()
	: _laserOffsetToBase( 0.1, 0 )
	, _laserOrientationOnBase( 0 )
	, _odomPosition( 0, 0 )
	, _odomOrientation( 0 )
	, _isInitialized( false )
{
}

rec::math::Vector2D Transform::fromLaserToOdom( const rec::math::Vector2D& laserPoint )
{
	Vector2D v = _odomPosition;
	
	Vector2D lv = _laserOffsetToBase;
	lv = rotate( lv, _laserOrientationOnBase + _odomOrientation );

	v += lv;

	Vector2D lp = rotate( laserPoint, _laserOrientationOnBase + _odomOrientation );

	v += lp;

	return v;
}

void Transform::odomPose( rec::math::Vector2D* pos, rec::math::Real* orientation )
{
	*pos = _odomPosition;
	*orientation = _odomOrientation;
}

void Transform::readingsEvent( double x, double y, double phi, float vx, float vy, float omega, unsigned int sequence )
{
	_odomPosition[0] = x;
	_odomPosition[1] = y;
	_odomOrientation = phi;

	_isInitialized = true;
}
