#ifndef _REC_QT_LASERRANGEFINDER_SCENE_H_
#define _REC_QT_LASERRANGEFINDER_SCENE_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif

#include <rec/robotino/api2/LaserRangeFinder.h>
#include <rec/robotino/api2/Odometry.h>

#include "StationDetector.h"

class MyCom;
class Transform;
class RobotinoItem;

class Scene : public QGraphicsScene, public rec::robotino::api2::Odometry, public rec::robotino::api2::LaserRangeFinder
{
	Q_OBJECT
public:
	Scene( Transform* transform, QObject* parent );

	virtual ~Scene();

	float currentMinAngle() const;
	float currentMaxAngle() const;

public Q_SLOTS:
	void setMarkers( const QVector< Marker >& );
	void setStations( const QVector< Station >& );

protected:
	static const int _scale = 100;

private:
	void scanEvent( const rec::robotino::api2::LaserRangeFinderReadings& scan );
	void readingsEvent( double x, double y, double phi, float vx, float vy, float omega, unsigned int sequence );

	Transform* _transform;

	RobotinoItem* _robotino;
	QGraphicsLineItem* _robotinoNormal;

	QVector< QGraphicsEllipseItem* > _points;
	QVector< QGraphicsEllipseItem* > _markers;
	QVector< QGraphicsPolygonItem* > _stations;
	QVector< QGraphicsLineItem* > _stationNormals;
	QGraphicsLineItem* _xaxis;
	QGraphicsLineItem* _yaxis;
	QGraphicsLineItem* _startAngleLine;
	QGraphicsLineItem* _stopAngleLine;

	QVector< QGraphicsLineItem* > _gridLines;

	float _currentMaxRange;
	float _currentMinAngle;
	float _currentMaxAngle;

	void resetGrid();
	void resetAngleLines();
};

inline float Scene::currentMinAngle() const
{
	return _currentMinAngle;
}

inline float Scene::currentMaxAngle() const
{
	return _currentMaxAngle;
}

class RobotinoItem : public QGraphicsPixmapItem
{
public:
	RobotinoItem( QGraphicsItem* parent );
};


#endif
