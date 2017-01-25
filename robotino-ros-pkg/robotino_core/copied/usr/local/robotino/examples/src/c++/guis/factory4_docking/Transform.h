#ifndef _TRANSFORM_H_
#define _TRANSFORM_H_

#include <rec/robotino/api2/Odometry.h>
#include <rec/math/Vector2D.h>

class MyCom;

class Transform : public rec::robotino::api2::Odometry
{
public:
	Transform();

	rec::math::Vector2D fromLaserToOdom( const rec::math::Vector2D& laserPoint );

	void odomPose( rec::math::Vector2D* pos, rec::math::Real* orientation );

	bool isInitialized() const { return _isInitialized; }

private:
	void readingsEvent( double x, double y, double phi, float vx, float vy, float omega, unsigned int sequence );

	rec::math::Vector2D _laserOffsetToBase;
	rec::math::Real _laserOrientationOnBase;

	rec::math::Vector2D _odomPosition;
	rec::math::Real _odomOrientation;

	bool _isInitialized;
};

#endif //_TRANSFORM_H_
