#ifndef _REC_ROBOTINO3_FLEETCOM_POSITION_H_
#define _REC_ROBOTINO3_FLEETCOM_POSITION_H_

#include <QtCore>

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class Position
			{
			public:
				Position()
					: id( invalidId )
					, x( 0.0 )
					, y( 0.0 )
					, phi( 0.0 )
					, numBelts( 0 )
					, distance( 0 )
				{
				}

				Position(int id_, double x_, double y_, double phi_, int numBelts_, int distance_)
					: id(id_)
					, x(x_)
					, y(y_)
					, phi(phi_)
					, numBelts(numBelts_)
					, distance(distance_)
				{
				}

				int id;
				double x;
				double y;
				double phi;
				int numBelts;
				double distance;

				bool operator< (const Position& other ) const
				{
					return ( id < other.id ); 
				}

				static const int invalidId = -1;
			};

			typedef QVector<Position> PositionList;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_POSITION_H_
