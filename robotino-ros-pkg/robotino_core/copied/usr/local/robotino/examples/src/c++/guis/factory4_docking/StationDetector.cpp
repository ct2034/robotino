#include "StationDetector.h"
#include "MyCom.h"
#include "Transform.h"
#include "Log.h"
#include <rec/math/utils.h>

double station_min_width = 0.5;
double station_max_width = 2.0;
int marker_min_num_scan_points = 5;

bool station_sort( const Station& a, const Station& b )
{
	return ( rec::math::norm2( a.position ) < rec::math::norm2( b.position ) );
}

StationDetector::StationDetector(Transform* transform )
: _transform( transform )
{
	reset();
}

void StationDetector::reset()
{
	_stations.clear();

	//Station s;
	//s.position[0] = 0.8;
	//s.position[1] = 0.3;

	//float a = rec::math::deg2rad( 20 );
	//s.normal[0] = cos(a);
	//s.normal[1] = sin(a);

	//s.width = 0.7;

	//_stations.append( s );
}

const QVector<Marker>& StationDetector::markers() const
{
	return _markers;
}

const QVector<Station>& StationDetector::stations() const
{
	return _stations;
}

void StationDetector::scanEvent( const rec::robotino::api2::LaserRangeFinderReadings& data )
{
	if( false == _transform->isInitialized() )
	{
		return;
	}

	unsigned int rangesSize;
	const float* ranges;
	data.ranges( &ranges, &rangesSize );

	unsigned int intensitiesSize;
	const float* intensities;
	data.intensities( &intensities, &intensitiesSize );

	_markers.clear();

	int seqStartIndex = -2;
	int seqStopIndex = -2;

	for( int i = 0; i< (int)intensitiesSize; ++i )
	{
		if( intensities[i] > 0 )
		{
			if( 1 == i - seqStopIndex )
			{
				seqStopIndex = i;
			}
			else
			{
				seqStartIndex = i;
				seqStopIndex = i;
			}
		}
		else
		{
			int num_scan_points = seqStopIndex - seqStartIndex + 1;

			if( num_scan_points >= marker_min_num_scan_points )
			{
				float meanAngle = data.angle_min + 0.5f * static_cast<float>( seqStartIndex + seqStopIndex ) * data.angle_increment;

				float mean_distance = 0;
				for( int i = seqStartIndex; i<=seqStopIndex; ++i )
				{
					mean_distance += ranges[i];
				}
				mean_distance /= static_cast<float>( num_scan_points );

				rec::math::Vector2D laserPoint = rec::math::polarToVector2D( meanAngle, mean_distance );

				Marker m;
				m.position = _transform->fromLaserToOdom( laserPoint );
				m.numScanPoints = num_scan_points;

				_markers << m;
			}

			seqStartIndex = -2;
			seqStopIndex = -2;
		}
	}

	Q_EMIT markersAt( _markers );

	QVector<Station> newStations;

	if( _markers.size() > 0 )
	{
		/*stations*/
		for( int i=1; i<_markers.size(); ++i )
		{
			const rec::math::Vector2D& p0 = _markers[i-1].position;
			const rec::math::Vector2D& p1 = _markers[i].position;

			const rec::math::Vector2D p0_to_p1 = p1 - p0;

			float width = rec::math::norm2( p0_to_p1 );

			if( width > station_min_width && width < station_max_width )
			{
				const rec::math::Vector2D station_mid_point = p0 + p0_to_p1 / 2.0f;

				Station s;
				s.position = station_mid_point;
				s.normal = rec::math::rotate( p0_to_p1, rec::math::PI_2 );
				s.normal /= rec::math::norm2( s.normal );

				s.width = width;

				newStations << s;

				++i;
			}
		}
	}

	qSort( newStations.begin(), newStations.end(), station_sort );

	if( _stations.isEmpty() )
	{
		_stations = newStations;
	}
	else
	{
		if( newStations.size() > 0 )
		{
			Station& oldStation = _stations[0];
			const Station& newStation = newStations[0];

			const float max_width_difference = 0.05f;
			if( fabs( oldStation.width - newStation.width ) < max_width_difference )
			{
				rec::math::Vector2D deltapos = newStation.position - oldStation.position;
				deltapos /= 10;
				oldStation.position += deltapos;

				rec::math::Vector2D deltanorm = newStation.normal - oldStation.normal;
				deltanorm /= 10;
				oldStation.normal += deltanorm;
				oldStation.normal /= rec::math::norm2( oldStation.normal );
			}
			else
			{
				recQtLog.log( QString("Stations width difference to big %1").arg( fabs( oldStation.width - newStation.width ) ) );
			}
		}
	}

	Q_EMIT stationsAt( _stations );

	//for( int i=0; i<_stations.size(); ++i )
	//{
	//	recQtLog.log( QString("Station %1: mid=%2,%3 distance=%4m").arg( i ).arg( _stations[i].position[0] ).arg( _stations[i].position[1] ).arg( rec::math::norm2( _stations[i].position ) ) );
	//}
}
