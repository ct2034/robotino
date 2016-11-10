//  Copyright (C) 2004-2013, Robotics Equipment Corporation GmbH

#ifndef _REC_MATH_LINEARAPPROXIMATOR_H_
#define _REC_MATH_LINEARAPPROXIMATOR_H_

#include "rec/math/Vector2D.h"
#include <vector>

#ifdef HAVE_QT
#endif

namespace rec
{
	namespace math
	{
		template< typename InputIterator > Real linearapproximator( InputIterator iter, InputIterator end, const Real x )
		{
			Real y = 0.0;

			if( iter != end )
			{
				if( x < (*iter)[0] )
				{
					y = (*iter)[1];
				}
				else
				{
					while( end != iter )
					{
						const Vector2D& p = (*iter);
						++iter;

						if( x >= p[0] )
						{
							if( end != iter )
							{
								if( x < (*iter)[0] )
								{
									const Vector2D& p2 = (*iter);
									Real dx = p2[0] - p[0];
									Real dy = p2[1] - p[1];

									if( 0.0 == dx )
									{
										y = p[1];
									}
									else
									{
										Real a = dy / dx;
										y = a * ( x - p[0] ) + p[1];
									}
								}
							}
							else
							{
								y = p[1];
							}
						}
					}
				}
			}

			return y;
		}

		inline Real linearapproximator( const std::vector<Vector2D>& vec, const Real x )
		{
			return linearapproximator( vec.begin(), vec.end(), x );
		}

		inline Real linearapproximator( const Vector2D* pointArray, unsigned int size, const Real x )
		{
			return linearapproximator( pointArray, pointArray + size, x );
		}

#ifdef HAVE_QT
#include <QVector>
		inline Real linearapproximator( const QVector<Vector2D>& vec, const Real x )
		{
			return linearapproximator( vec.begin(), vec.end(), x );
		}
#endif
	}
}

#endif //_REC_MATH_LINEARAPPROXIMATOR_H_
