#ifndef _DRIVER_H_
#define _DRIVER_H_

#include <QtCore>
#include <rec/robotino/api2/OmniDrive.h>
#include <rec/math/Vector2D.h>

class MyCom;
class Transform;
class StationDetector;

class Driver : public QObject
{
	Q_OBJECT
public:
	Driver( Transform* transform, StationDetector* stationDetector );

public Q_SLOTS:
	void start( int belt );
	void stop();

private Q_SLOTS:
	void on_timer_timeout();

private:
	Transform* _transform;

	rec::robotino::api2::OmniDrive _omnidrive;
	StationDetector* _stationDetector;
	QTimer* _timer;

	typedef enum
	{
		ROUGH,
		FINE
	} State_t;

	State_t _state;
	int _belt;

	QVector<rec::math::Vector2D> _omegaControl;
	QVector<rec::math::Vector2D> _vxControl;
	QVector<rec::math::Vector2D> _vyControl;
};

#endif //_DRIVER_H_
