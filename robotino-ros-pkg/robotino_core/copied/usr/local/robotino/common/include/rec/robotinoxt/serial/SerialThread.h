#ifndef _REC_ROBOTINOXT_SERIALTHREAD_H_
#define _REC_ROBOTINOXT_SERIALTHREAD_H_

#include <QtCore>
#include "rec/robotinoxt/defines.h"
#include "rec/serialport/SerialPort.h"
#include "rec/robotinoxt/serial/MessageBuffer.h"

Q_DECLARE_METATYPE( QVector< qint16 > );
Q_DECLARE_METATYPE( QVector< float > );

namespace rec
{
	namespace robotinoxt
	{
		namespace serial
		{
			class SerialWriteThread;

			class REC_ROBOTINOXT_EXPORT SerialThread : public QThread
			{
				Q_OBJECT
			public:
				SerialThread( QObject* parent );
				~SerialThread();

				/**
				Automatically find the correct COM port.
				*/
				bool open();

				bool open( const QString& port );

				QString port() const { return _portStr; }

				void close();

				bool isOpen() const;

				void sendResetDevice( bool enterUSBBootloader, bool queued = true );

				void sendGetInfo();

				void sendSetPressures( const QVector< qint16 >& pressures );

				void sendSetDOut( unsigned int number, bool enabled );

				void sendSetCompressorsEnabled( bool enabled );

			Q_SIGNALS:
				void serialPortError( const QString& /*error*/ );
				void infoReceived( int, int, int );
				void statusReceived( const QVector< qint16 >& /*pressures*/, bool /*pressureSensor*/, const QVector< float >& /*potis*/ );

			private:
				void run();

				bool _run;

				SerialWriteThread* _serialWriteThread;

				rec::serialport::SerialPort _serialPort;
				QString _portStr;
			};

			class SerialWriteThread : public QThread
			{
				Q_OBJECT
			public:
				SerialWriteThread( QObject* parent, rec::serialport::SerialPort* serialPort );

				void start();

				void stop();

				void sendResetDevice( bool enterUSBBootloader );

				void sendGetInfo();

				void sendSetPressures( const QVector< qint16 >& pressures );

				void sendSetDOut( unsigned int number, bool enabled );

				void sendSetCompressorsEnabled( bool enabled );

			private:
				void run();

				QMutex _serialPortMutex;
				rec::serialport::SerialPort* _serialPort;
				bool _run;

				QVector< rec::robotinoxt::serial::MessageBuffer > _messages;
				QSemaphore _sendSemaphore;
			};
		}
	}
}

#endif // _REC_ROBOTINOXT_SERIALTHREAD_H_
