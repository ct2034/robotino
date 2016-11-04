#ifndef _REC_NSTAR2_REPORT_H_
#define _REC_NSTAR2_REPORT_H_

#include <string.h>
#include <cmath>

namespace rec
{
	namespace nstar2
	{
		class Report
		{
		public:
			Report( const int* ids, const float* xs, const float* ys, const int* mags, int size )
				: _size( size )
			{
				_ids = new int[_size];
				memcpy( _ids, ids, _size * sizeof( int ) );

				_xs = new float[_size];
				memcpy( _xs, xs, _size * sizeof( float ) );

				_ys = new float[_size];
				memcpy( _ys, ys, _size * sizeof( float ) );

				_mags = new int[_size];
				memcpy( _mags, mags, _size * sizeof( int ) );
			}

			~Report()
			{
				delete [] _ids;
				delete [] _xs;
				delete [] _ys;
				delete [] _mags;
			}

			int size() const { return _size; }

			int id( int index ) const
			{
				return _ids[index];
			}

			float x( int index ) const
			{
				return _xs[index];
			}

			float y( int index ) const
			{
				return _ys[index];
			}

			int mag( int index ) const
			{
				return _mags[index];
			}

			/**
			@param x0 x-coordinate of spot 0
			@param y0 y-coordinate of spot 0
			@param x1 x-coordinate of spot 1
			@param y1 y-coordinate of spot 1
			@param ceilingHeight Distance from Northstar detector to ceiling in meters
			@param x x-position of Northstar detector in world coordinates in meters
			@param y y-position of Northstar detector in world coordinates in meters
			@param theta Rotattion of Northstar detector in world coordinates in radiants
			*/
			static void calculate_pose( float x0, float y0, float x1, float y1, float ceilingHeight, float* x, float* y, float* theta )
			{
				float A = x1-x0;
				float B = y1-y0;
				float C = x1+x0;
				float D = y1+y0;

				*theta = atan2(-B,A);
				float sint = sin(*theta);
				float cost = cos(*theta);
				*x = 0.5f * (-C*cost+D*sint);
				*y = 0.5f * (-C*sint-D*cost);

				float k = 77.0777f / ( 2.0f * 266.0f ) * ceilingHeight;

				*x /= k;
				*y /= k;
			}

		private:
			int _size;
			int* _ids;
			float* _xs;
			float* _ys;
			int* _mags;
		};
	}
}

#endif //_REC_NSTAR2_REPORT_H_
