#ifndef _REC_GRAPPLER_MESSAGEBUFFER_H_
#define _REC_GRAPPLER_MESSAGEBUFFER_H_

#include <QtCore>

namespace rec
{
	namespace grappler
	{
		namespace serial
		{
			class MessageBuffer
			{
			public:
				MessageBuffer()
				{
				}

				MessageBuffer( const MessageBuffer& other )
				{
					QMutexLocker lk2( &other.mutex );
					message = other.message;
				}

				MessageBuffer& operator=( const MessageBuffer& other )
				{
					QMutexLocker lk( &mutex );
					QMutexLocker lk2( &other.mutex );
					message = other.message;
					return *this;
				}

				mutable QMutex mutex;
				QByteArray message;
			};
		}
	}
}

#endif //_REC_GRAPPLER_MESSAGEBUFFER_H_
