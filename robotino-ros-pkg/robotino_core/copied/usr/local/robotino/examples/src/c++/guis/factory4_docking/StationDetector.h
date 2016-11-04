#ifndef _STATIONDETECTOR_H_
#define _STATIONDETECTOR_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif

#include <rec/robotino/api2/LaserRangeFinder.h>
#include <rec/math/Vector2D.h>

class MyCom;
class Transform;

class Marker
{
public:
	Marker()
		: position( 0, 0 )
		, numScanPoints( 0 )
	{
	}

	rec::math::Vector2D position; //odom frame
	int numScanPoints;
};
Q_DECLARE_METATYPE(Marker)

class Station
{
public:
	Station()
		: position( 0, 0 )
		, normal( 0, 0 )
		, width( 0 )
	{
	}

	rec::math::Vector2D position; //odom frame
	rec::math::Vector2D normal; //odom frame
	rec::math::Real width;
};
Q_DECLARE_METATYPE(Station)

class StationDetector : public QObject, public rec::robotino::api2::LaserRangeFinder
{
	Q_OBJECT
public:
	StationDetector( Transform* transform );

	void reset();

	const QVector<Marker>& markers() const;
	const QVector<Station>& stations() const;

Q_SIGNALS:
	void markersAt( const QVector< Marker >& );
	void stationsAt( const QVector< Station >& );

private:
	void scanEvent( const rec::robotino::api2::LaserRangeFinderReadings& scan );

	Transform* _transform;

	QVector<Marker> _markers;
	QVector<Station> _stations;
};

#endif //_STATIONDETECTOR_H_

