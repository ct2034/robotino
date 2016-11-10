#ifndef _REC_MATH_UTILS_H_
#define _REC_MATH_UTILS_H_

#include "rec/math/defines.h"
#include <cmath>
#include <string.h>

#ifndef FUNCTION_IS_NOT_USED
#ifdef __GNUC__
#define FUNCTION_IS_NOT_USED __attribute__ ((unused))
#else
#define FUNCTION_IS_NOT_USED
#endif
#endif

namespace rec
{
	namespace math
	{
		static FUNCTION_IS_NOT_USED Real deg2rad( Real deg )
		{
			return PI * deg / 180.0f;
		}

		static FUNCTION_IS_NOT_USED Real rad2deg( Real rad )
		{
			return 180.0f * rad / PI;
		}

		static FUNCTION_IS_NOT_USED bool realLess( const Real a, const Real b )
		{
			if( a < b - RealEpsilon )
			{
				return true;
			}

			return false;
		}

		static FUNCTION_IS_NOT_USED bool realGreater( const Real a, const Real b )
		{
			if( a > b + RealEpsilon )
			{
				return true;
			}

			return false;
		}

		static FUNCTION_IS_NOT_USED bool realEqual( const Real a, const Real b )
		{
			if( ( a >= 0 && b >= 0 ) || ( a < 0 && b < 0 ) )
			{
				if( fabs( a - b ) < RealEpsilon )
				{
					return true;
				}
			}

			return false;
		}

		static FUNCTION_IS_NOT_USED Real mapToMinusPItoPI( const Real angle )
		{
			Real angleOverflow = static_cast<float>( static_cast<int>( angle / PI ) );

			if( angleOverflow > 0.0f )
			{
				angleOverflow = ceil( angleOverflow / 2.0f );
			}
			else
			{
				angleOverflow = floor( angleOverflow / 2.0f );
			}

			Real result = angle - 2 * PI * angleOverflow;

			return result;
		}

		static FUNCTION_IS_NOT_USED void eulerToAxisAngle( float heading, float attitude, float bank, float& angle, float& x, float& y, float& z )
		{
			float c1 = cos( heading / 2 );
			float c2 = cos( attitude / 2 );
			float c3 = cos( bank / 2 );
			float s1 = sin( heading / 2 );
			float s2 = sin( attitude / 2 );
			float s3 = sin( bank / 2 );

			x = s1 * s2 * c3 + c1 * c2 * s3;
			y = s1 * c2 * c3 + c1 * s2 * s3;
			z = c1 * s2 * c3 - s1 * c2 * s3;

			float n = sqrt( x * x + y * y + z * z );
			if ( n == 0 )
			{
				x = 1;
				y = 0;
				z = 0;
				angle = 0;
			}
			else
			{
				x /= n;
				y /= n;
				z /= n;
				angle = 2 * acos( c1 * c2 * c3 - s1 * s2 * s3 );
			}

		}

		static FUNCTION_IS_NOT_USED float round( float value )
		{
			float dummy;
			if ( fabs( std::modf( value, &dummy ) ) >= 0.5f )
				value = value >= 0 ? ceil( value ) : floor( value );
			else
				value = value >= 0 ? floor( value ) : ceil( value );
			return value;
		}

		static FUNCTION_IS_NOT_USED void quaternionToEuler( const float quat[4], float& heading, float& attitude, float& bank )
		{
			// good reference: http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/index.htm
			float q[4];
			memcpy( q, quat, 4 * sizeof( float ) );
			for( int i = 0; i < 4; i++ )
			{
				if ( std::fabs( q[i] ) < 0.00001f )
					q[i] = 0;
				else if ( std::fabs( q[i] - 0.5f ) < 0.00001f )
					q[i] = 0.5f;
				else if ( std::fabs( q[i] + 0.5f ) < 0.00001f )
					q[i] = -0.5f;
				else if ( std::fabs( q[i] - 1.f ) < 0.00001f )
					q[i] = 1.f;
				else if ( std::fabs( q[i] + 1.f ) < 0.00001f )
					q[i] = -1.f;
				else if ( std::fabs( q[i] - SQRT1_2 ) < 0.00001f )
					q[i] = static_cast< float >( SQRT1_2 );
				else if ( std::fabs( q[i] + SQRT1_2 ) < 0.00001f )
					q[i] = - static_cast< float >( SQRT1_2 );
			}

			attitude = asin( 2 * q[0] * q[1] + 2 * q[2] * q[3] );

			float testValue = q[0] * q[1] + q[2] * q[3];

			if( std::fabs( 0.5 - testValue ) < 0.00001f )
			{
				heading = 2 * atan2( q[0], q[3] );
				bank = 0;
			}
			else if( std::fabs( 0.5 + testValue ) < 0.00001f )
			{
				heading = -2 * atan2( q[0], q[3] );
				bank = 0;
			}
			else
			{
				heading = atan2( 2 * q[1] * q[3] - 2 * q[0] * q[2] , 1 - 2 * q[1] * q[1] - 2 * q[2] * q[2] );
				bank = atan2( 2 * q[0] * q[3] - 2 * q[1] * q[2] , 1 - 2 * q[0] * q[0] - 2 * q[2] * q[2] );
			}
		}

		static FUNCTION_IS_NOT_USED void eulerToQuaternion( float heading, float attitude, float bank, float quat[4] )
		{
			float c1 = cos( heading / 2 );
			float c2 = cos( attitude / 2 );
			float c3 = cos( bank / 2 );
			float s1 = sin( heading / 2 );
			float s2 = sin( attitude / 2 );
			float s3 = sin( bank / 2 );
			quat[0] = s1 * s2 * c3 + c1 * c2 * s3;
			quat[1] = s1 * c2 * c3 + c1 * s2 * s3;
			quat[2] = c1 * s2 * c3 - s1 * c2 * s3;
			quat[3] = c1 * c2 * c3 - s1 * s2 * s3;
		}
	}
}

#endif //_REC_MATH_UTILS_H_
