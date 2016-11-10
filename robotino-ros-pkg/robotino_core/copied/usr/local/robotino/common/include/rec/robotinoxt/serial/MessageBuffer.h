#ifndef _REC_ROBOTINOXT_MESSAGEBUFFER_H_
#define _REC_ROBOTINOXT_MESSAGEBUFFER_H_

#include <QtCore>

namespace rec
{
	namespace robotinoxt
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
					messages = other.messages;
				}

				MessageBuffer& operator=( const MessageBuffer& other )
				{
					QMutexLocker lk( &mutex );
					QMutexLocker lk2( &other.mutex );
					messages = other.messages;
					return *this;
				}

				mutable QMutex mutex;
				QQueue< QByteArray > messages;
			};
		}
	}
}

#endif //_REC_ROBOTONOXT_MESSAGEBUFFER_H_
