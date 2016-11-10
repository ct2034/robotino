//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#ifndef _REC_LINEARAPPROXIMATOR_H_
#define _REC_LINEARAPPROXIMATOR_H_

#include <QVector>
#include <QPointF>

namespace rec
{
	template< typename InputIterator > double linearapproximator( InputIterator iter, InputIterator end, const double x )
	{
		double y = 0.0;

		if( iter != end )
		{
			if( x < (*iter).x() )
			{
				y = (*iter).y();
			}
			else
			{
				while( end != iter )
				{
					const QPointF& p = (*iter);
					++iter;

					if( x >= p.x() )
					{
						if( end != iter )
						{
							if( x < (*iter).x() )
							{
								const QPointF& p2 = (*iter);
								double dx = p2.x() - p.x();
								double dy = p2.y() - p.y();

								if( 0.0 == dx )
								{
									y = p.y();
								}
								else
								{
									double a = dy / dx;
									y = a * ( x - p.x() ) + p.y();
								}
							}
						}
						else
						{
							y = p.y();
						}
					}
				}
			}
		}

		return y;
	}

	inline double linearapproximator( const QVector< QPointF >& vec, const double x )
	{
		return linearapproximator( vec.constBegin(), vec.constEnd(), x );
	}

	inline double linearapproximator( const QPointF* pointArray, unsigned int size, const double x )
	{
		return linearapproximator( pointArray, pointArray + size, x );
	}
}

#endif //_REC_CORE_LINEARAPPROXIMATOR_H_
