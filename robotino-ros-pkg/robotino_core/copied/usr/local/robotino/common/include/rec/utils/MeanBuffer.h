//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#ifndef _REC_MEANBUFFER_H_
#define _REC_MEANBUFFER_H_

namespace rec
{
	template <class T> class MeanBuffer
	{
	public:
		MeanBuffer<T>( unsigned int size = 50, T startValue = 0 )
			: _size( size )
			, _buffer( new T[ _size ] )
		{
			reset( startValue );
		}

		~MeanBuffer()
		{
			delete [] _buffer;
		}

		void reset( T startValue = 0 )
		{
			_currentIndex = 0;
			_bufferSum = 0;
			_bufferMean = startValue;

			for( unsigned int i=0; i<_size; i++ )
			{
				_buffer[i] = startValue;
			}
			_bufferSum = startValue * _size;
		}

		void add( T value )
		{
			_currentIndex = ( _currentIndex + 1 ) % _size;
			_bufferSum -= _buffer[ _currentIndex ];
			_buffer[ _currentIndex ] = value;
			_bufferSum += value;
			_bufferMean = _bufferSum / _size;
		}

		T mean() const
		{
			return _bufferMean;
		}

		T sum() const
		{
			return _bufferSum;
		}

	private:
		const unsigned int _size;
		T* _buffer;
		unsigned int _currentIndex;
		T _bufferSum;
		T _bufferMean;
	};
}

#endif
