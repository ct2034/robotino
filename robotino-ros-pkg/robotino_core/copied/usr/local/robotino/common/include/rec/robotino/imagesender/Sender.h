#ifndef _REC_ROBOTINO_IMAGESENDER_SENDER_H_
#define _REC_ROBOTINO_IMAGESENDER_SENDER_H_

#include <QtCore>
#include <QtNetwork>

#include "rec/robotino/imagesender/defines.h"

namespace rec
{
	namespace robotino
	{
		namespace imagesender
		{
			class REC_ROBOTINO_IMAGESENDER_EXPORT Sender : public QUdpSocket
			{
				Q_OBJECT
			public:
				typedef enum { IdleState, SendingState } State;

				Sender( QObject* parent );

				State state() const { return _state; }

				/**
				Stop sending current image data. state() will return IdleState after calling stop().
				*/
				void stop();

			public Q_SLOTS:
				void setRawImageData( const QByteArray& data,
																unsigned int width,
																unsigned int height,
																unsigned int numChannels,
																unsigned int bitsPerChannel,
																unsigned int step );

				void setJpgImageData( const QByteArray& data );

				void setReceiver( quint32 address, quint16 port );

			Q_SIGNALS:
				void imageSendingCompleted();

			private Q_SLOTS:
				void on_timer_timeout();

			private:
				static const int _startSequenceSize;
				static const qint8 _startSequence[10];

				static const int _stopSequenceSize;
				static const qint8 _stopSequence[10];

				static const int _headerSize;

				void setImageData( bool isJpg,
					const QByteArray& data,
					unsigned int width,
					unsigned int height,
					unsigned int numChannels,
					unsigned int bitsPerChannel,
					unsigned int step );

				QHostAddress _receiver;
				quint16 _receiverPort;

				QByteArray _imageData;
				qint32 _bytesWritten;

				qint32 _partSize;

				State _state;

				QTimer* _timer;
			};
		}
	}
}

#endif //_REC_ROBOTINO_IMAGESENDER_SENDER_H_
