#ifndef _REC_GRAPPLER_SERIALTHREAD_H_
#define _REC_GRAPPLER_SERIALTHREAD_H_

#include <QtCore>
#include "rec/grappler/defines.h"
#include "rec/serialport/SerialPort.h"
#include "rec/grappler/HardwareInfo.h"
#include "rec/grappler/serial/MessageBuffer.h"

namespace rec
{
	namespace grappler
	{
		namespace serial
		{
			class SerialWriteThread;

			class REC_GRAPPLER_EXPORT SerialThread : public QThread
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

				void sendDynamixelCommand( const QByteArray& message );

				void sendGetDetectedServos();

				void sendGetAllPositions();

				void sendGetAllAxesLimits();

				void sendAllPositions( const rec::grappler::HardwareInfo& info );

				void setAutoUpdateEnabled( bool enable );

				bool isOpen() const;

				void sendEnablePower( int channel, bool enable );
	
				void sendToggleTorque();

				void sendResetDevice( bool enterUSBBootloader );

				void sendGetInfo();

			Q_SIGNALS:
				void serialPortError( const QString& /*error*/ );
				void statusReceived( int /*id*/, int /*error*/, const QByteArray& /*data*/ );
				void statusTimeout( int /*channel*/ );
				void enableMotors( bool /*enable*/ );
				void storePosition( const rec::grappler::HardwareInfo& /*info*/ );

				void servosDetected( const rec::grappler::HardwareInfo& /*info*/ );
				void allServoPositions( const rec::grappler::HardwareInfo& /*info*/ );
				void allAxesLimits( const rec::grappler::HardwareInfo& /*info*/ );

				void infoReceived( int, int, int );

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

				void sendDynamixelCommand( const QByteArray& command );

				void sendGetDetectedServos();

				void sendAllPositions( const QByteArray& message );

				void sendGetAllPositions();

				void sendGetAllAxesLimits();

				void setAutoUpdateEnabled( bool enable );

				void start();

				void stop();

				void sendEnablePower( int channel, bool enable );
	
				void sendToggleTorque();

				void sendResetDevice( bool enterUSBBootloader );

				void sendGetInfo();

			private:
				void run();

				QMutex _serialPortMutex;
				rec::serialport::SerialPort* _serialPort;
				bool _run;

				QVector< rec::grappler::serial::MessageBuffer > _messages;
				QSemaphore _sendSemaphore;
			};
		}
	}
}

#endif //_SERIALTHREAD_H_
