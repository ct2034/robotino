#include "Driver.h"
#include "MyCom.h"
#include "StationDetector.h"
#include "Transform.h"
#include "Log.h"
#include <rec/math/utils.h>
#include <rec/math/linearapproximator.h>

Driver::Driver( Transform* transform, StationDetector* stationDetector )
	: _transform( transform )
	, _stationDetector( stationDetector )
	, _timer( new QTimer( this ) )
	, _state( ROUGH )
{
	_timer->setSingleShot( false );
	_timer->setInterval( 30 );
	
	bool ok;
	ok = connect( _timer, SIGNAL( timeout() ), SLOT( on_timer_timeout() ) );
	Q_ASSERT( ok );

	_omegaControl << rec::math::Vector2D( 0, 0 ) << rec::math::Vector2D( 20, 5 );
	_vxControl << rec::math::Vector2D( 0.4, 0 ) << rec::math::Vector2D( 0.41, 0.05 ) << rec::math::Vector2D( 0.5, 0.1 );
	_vyControl << rec::math::Vector2D( 0, 0 ) << rec::math::Vector2D( 0.1, 0.05 );
}

void Driver::start( int belt )
{
	_belt = belt;
	_state = ROUGH;
	_stationDetector->reset();
	_timer->start();
}

void Driver::stop()
{
	_timer->stop();
	_omnidrive.setVelocity( 0, 0, 0 );
}

void Driver::on_timer_timeout()
{
	if( 0 == _stationDetector->stations().size() )
	{
		_omnidrive.setVelocity( 0, 0, 0 );
		return;
	}

	rec::math::Vector2D pos;
	rec::math::Real orientation;

	_transform->odomPose( &pos, &orientation );

	const Station& s =  _stationDetector->stations()[0];

	rec::math::Vector2D beltPos;
	if( s.width < 0.5 )
	{
		//station with one belt
		beltPos = s.position;
	}
	else
	{
		//station with 2 belts
		switch( _belt )
		{
		case 0:
			{
				rec::math::Vector2D d = rec::math::rotate( s.normal, -rec::math::PI_2 );
				d *= 0.2;
				beltPos = s.position + d;
			}
			break;

		default:
			{
				rec::math::Vector2D d = rec::math::rotate( s.normal, rec::math::PI_2 );
				d *= 0.2;
				beltPos = s.position + d;
			}
			break;
		}
	}

	rec::math::Vector2D sNormRotated = rec::math::rotate( s.normal, rec::math::PI );

	rec::math::Real sNormLength;
	rec::math::Real sNormPhi;

	rec::math::vector2DToPolar( sNormRotated, &sNormPhi, &sNormLength );

	rec::math::Real angleDelta = rec::math::mapToMinusPItoPI( sNormPhi - orientation );

	rec::math::Vector2D distVec = beltPos - pos;
	rec::math::Real dist = rec::math::norm2( distVec );

	rec::math::Vector2D beltPosInRobotFrame = rec::math::rotate( distVec, -orientation );

	recQtLog.log( QString("sangle %1  rangle %2  delta %3  dist %4  dy(Rframe) %5").arg( rec::math::rad2deg( sNormPhi ) ).arg( rec::math::rad2deg( orientation ) ).arg( rec::math::rad2deg( angleDelta ) ).arg( dist ).arg( beltPosInRobotFrame[1] ) );

	rec::math::Real vx = rec::math::linearapproximator( _vxControl, dist );

	rec::math::Real vy = rec::math::linearapproximator( _vyControl, fabs( beltPosInRobotFrame[1] ) );
	if( beltPosInRobotFrame[1] < 0 )
	{
		vy = -vy;
	}

	rec::math::Real omega = rec::math::linearapproximator( _omegaControl, fabs( rec::math::rad2deg( angleDelta ) ) );

	omega = rec::math::deg2rad( omega );
	if( angleDelta < 0 )
	{
		omega = -omega;
	}

	_omnidrive.setVelocity( vx, vy, omega );

	recQtLog.log( QString("vx=%1m/s vy=%2m/s omega=%3deg/s").arg(vx).arg(vy).arg(rec::math::rad2deg(omega)) );
}
