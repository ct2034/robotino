#include "Scene.h"
#include "Transform.h"
#include "MyCom.h"
#include "Transform.h"
#include <rec/math/utils.h>

#include <cmath>

Scene::Scene( Transform* transform, QObject* parent )
: _transform( transform )
, _robotino( new RobotinoItem( NULL ) )
, _robotinoNormal( new QGraphicsLineItem )
, _xaxis( new QGraphicsLineItem )
, _yaxis( new QGraphicsLineItem )
, _startAngleLine( new QGraphicsLineItem )
, _stopAngleLine( new QGraphicsLineItem )
, _currentMaxRange( 30 )
{
	addItem( _robotino );
	_robotino->setScale( _scale * 0.001 );
	addItem( _robotinoNormal );
	_robotinoNormal->setPen( QPen( Qt::blue, 1 ) );

	_xaxis->setPen( QPen( Qt::black, 5 ) );
	_yaxis->setPen( QPen( Qt::black, 5 ) );

	_startAngleLine->setPen( QPen( Qt::blue, 5 ) );
	_stopAngleLine->setPen( QPen( Qt::blue, 5 ) );

	setBackgroundBrush( QColor( 0xd7d7d7 ) );

	resetGrid();
}

Scene::~Scene()
{
	clear(); 
}

void Scene::resetGrid()
{
	qDeleteAll( _gridLines );
	_gridLines.clear();

	QPen gridPen( Qt::gray );

	int range = static_cast< int >( ceilf( _currentMaxRange ) );

	for( int i = 1; i <= range; i++ )
	{
		_gridLines << addLine( -range * _scale,  i * _scale, range * _scale,  i * _scale, gridPen );
		_gridLines << addLine( -range * _scale, -i * _scale, range * _scale, -i * _scale, gridPen );
		_gridLines << addLine(  i * _scale, -range * _scale,  i * _scale, range * _scale, gridPen );
		_gridLines << addLine( -i * _scale, -range * _scale, -i * _scale, range * _scale, gridPen );
	}

	_xaxis->setLine( -_scale * ( range + 1 ), 0, _scale * ( range + 1 ), 0 );
	_yaxis->setLine( 0, -_scale * ( range + 1 ), 0, _scale * ( range + 1 ) );
}

void Scene::resetAngleLines()
{
	float range = ceilf( _currentMaxRange );
	const QLineF bounds[4] =
	{
		QLineF( -range * _scale, -range * _scale, -range * _scale,  range * _scale ),
		QLineF( -range * _scale,  range * _scale,  range * _scale,  range * _scale ),
		QLineF(  range * _scale,  range * _scale,  range * _scale, -range * _scale ),
		QLineF(  range * _scale, -range * _scale, -range * _scale, -range * _scale ),
	};

	QLineF l( 0, 0, 1, 1 );
	l.setAngle( _currentMinAngle * 180 / M_PI );
	l.setLength( _scale * range * 2 );
	QPointF intersectionPoint;
	for( int i = 0; i < 4; i++ )
	{
		if ( l.intersect( bounds[i], &intersectionPoint ) == QLineF::BoundedIntersection )
		{
			break;
		}
	}
	l.setP2( intersectionPoint );
	_startAngleLine->setLine( l );

	l.setAngle( _currentMaxAngle * 180 / M_PI );
	l.setLength( _scale * range * 2 );
	for( int i = 0; i < 4; i++ )
	{
		if ( l.intersect( bounds[i], &intersectionPoint ) == QLineF::BoundedIntersection )
		{
			break;
		}
	}
	l.setP2( intersectionPoint );
	_stopAngleLine->setLine( l );
}

void Scene::scanEvent( const rec::robotino::api2::LaserRangeFinderReadings& data )
{
	unsigned int rangesSize;
	const float* ranges;
	data.ranges( &ranges, &rangesSize );

	unsigned int intensitiesSize;
	const float* intensities;
	data.intensities( &intensities, &intensitiesSize );

	if( 0 == rangesSize )
	{
		//if ( _xaxis->scene() == this )
		//	removeItem( _xaxis );
		//if ( _yaxis->scene() == this )
		//	removeItem( _yaxis );
		//if ( _startAngleLine->scene() == this )
		//	removeItem( _startAngleLine );
		//if ( _stopAngleLine->scene() == this )
		//	removeItem( _stopAngleLine );

		//qDeleteAll( _gridLines );
		//_gridLines.clear();
		//qDeleteAll( _points );
		//_points.clear();

		//clear();

		//_currentMinAngle = data.angle_min;
		//_currentMaxAngle = data.angle_max;
		//_currentMaxRange = data.range_max;
		return;
	}

	//if ( _xaxis->scene() != this )
	//	addItem( _xaxis );
	//if ( _yaxis->scene() != this )
	//	addItem( _yaxis );
	//if ( _startAngleLine->scene() != this )
	//	addItem( _startAngleLine );
	//if ( _stopAngleLine->scene() != this )
	//	addItem( _stopAngleLine );

	//{
	//	float oldMinAngle = _currentMinAngle;
	//	float oldMaxAngle = _currentMaxAngle;
	//	float oldMaxRange = _currentMaxRange;

	//	_currentMinAngle = data.angle_min;
	//	_currentMaxAngle = data.angle_max;
	//	_currentMaxRange = data.range_max;

	//	if ( _currentMinAngle != oldMinAngle || _currentMaxAngle != oldMaxAngle )
	//	{
	//		resetAngleLines();
	//	}

	//	if ( _currentMaxRange != oldMaxRange )
	//	{
	//		resetGrid();
	//	}
	//}

	if( _points.size() < rangesSize )
	{
		int i = _points.size();
		_points.resize( rangesSize );
		for( ; i < _points.size(); ++i )
		{
			_points[i] = addEllipse( -1, -1, 2, 2 );
		}
	}
	else if( _points.size() > rangesSize )
	{
		for( int i = rangesSize; i < _points.size(); ++i )
		{
			delete _points[i];
		}
		_points.resize( rangesSize );
	}

	for( int i=0; i<rangesSize; ++i )
	{
		qreal angle = data.angle_min + data.angle_increment * i;

		rec::math::Vector2D v;
		v[0] = ranges[i] * cos( angle );
		v[1] = ranges[i] * sin( angle );

		rec::math::Vector2D vodom = _transform->fromLaserToOdom( v );

		_points[i]->setPos( _scale * vodom[0], _scale * vodom[1] );

		QColor color = Qt::darkGray;
		if( i < intensitiesSize )
		{
			if ( static_cast< unsigned int >( intensities[i] ) >= 1 )
			{
				color = Qt::yellow;
			}
		}
		_points[i]->setBrush( color );
		_points[i]->setPen( color );
	}
}

void Scene::setMarkers( const QVector< Marker >& markers )
{
	if( _markers.size() < markers.size() )
	{
		int i = _markers.size();
		_markers.resize( markers.size() );
		for( ; i < _markers.size(); ++i )
		{
			_markers[i] = addEllipse( -3, -3, 6, 6 );
			_markers[i]->setPen( QPen( QColor( Qt::green ), 2 ) );
		}
	}
	else if( _markers.size() > markers.size() )
	{
		for( int i = markers.size(); i < _markers.size(); ++i )
		{
			delete _markers[i];
		}
		_markers.resize(  markers.size() );
	}

	for( int i=0; i<_markers.size(); ++i )
	{
		_markers[i]->setPos( _scale * markers[i].position[0],  _scale * markers[i].position[1] );
	}
}

void Scene::setStations( const QVector< Station >& stations )
{
	if( _stations.size() < stations.size() )
	{
		int i = _stations.size();
		_stations.resize( stations.size() );
		_stationNormals.resize( stations.size() );
		for( ; i < _stations.size(); ++i )
		{
			_stations[i] = addPolygon( QPolygonF() );
			_stations[i]->setPen( QPen( QColor( Qt::red ), 1 ) );

			_stationNormals[i] = addLine( 0, 0, 0, 0, QPen( QColor( Qt::red ), 1 ) );
		}
	}
	else if( _stations.size() > stations.size() )
	{
		for( int i = stations.size(); i < _stations.size(); ++i )
		{
			delete _stations[i];
			delete _stationNormals[i];
		}
		_stations.resize(  stations.size() );
		_stationNormals.resize(  stations.size() );
	}

	for( int i=0; i<_stations.size(); ++i )
	{
		const Station& s = stations[i];

		rec::math::Vector2D widthVec = rec::math::rotate( s.normal, rec::math::PI_2 );
		widthVec *= s.width;
		widthVec /= 2;

		rec::math::Vector2D p0 = s.position - widthVec;
		rec::math::Vector2D p1 = s.position + widthVec;
		rec::math::Vector2D p2 = p1 - s.normal;
		rec::math::Vector2D p3 = p0 - s.normal;


		QPolygonF p;
		p << _scale * QPointF( p0[0], p0[1] ) << _scale * QPointF( p1[0], p1[1] ) <<  _scale * QPointF( p2[0], p2[1] ) << _scale * QPointF( p3[0], p3[1] );
		_stations[i]->setPolygon( p );

		_stationNormals[i]->setLine( _scale * s.position[0], _scale * s.position[1], _scale * (s.position[0]+s.normal[0]), _scale * (s.position[1]+s.normal[1]) );
	}
}

void Scene::readingsEvent( double x, double y, double phi, float vx, float vy, float omega, unsigned int sequence )
{
	_robotino->setPos( _scale * x, _scale * y );
	_robotino->setRotation( rec::math::rad2deg( phi ) );

	_robotinoNormal->setLine( _scale * x, _scale * y, _scale * (x + cos(phi)), _scale * (y + sin(phi)) );
}

RobotinoItem::RobotinoItem( QGraphicsItem* parent )
: QGraphicsPixmapItem( parent )
{
	QPixmap pix = QPixmap( ":/robotino_top.png" );
	setPixmap( pix );
	setOffset( - pix.width() / 2, - pix.height() / 2 );
}
