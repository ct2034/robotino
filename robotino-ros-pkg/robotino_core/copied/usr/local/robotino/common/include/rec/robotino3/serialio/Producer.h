#ifndef _REC_ROBOTINO3_SERIALIO_PRODUCER_H_
#define _REC_ROBOTINO3_SERIALIO_PRODUCER_H_

#include <QtCore>
#include "rec/robotino3/serialio/Tag.h"

namespace rec
{
	namespace serialport
	{
		class SerialPort;
	}

	namespace robotino3
	{
		namespace serialio
		{
			class ProducerDelegate
			{
			public:
				ProducerDelegate()
				{
				}

				virtual ~ProducerDelegate()
				{
				}

				virtual int producerDelegateVerbosity() const { return 0; };
				virtual void producerDelegateLog( const QString& message ) {};
			};

			class Producer
			{
			public:
				Producer( rec::serialport::SerialPort* serial, ProducerDelegate* delegate );

				/**
				* @throws rec::serialport::SerialPortException
				*/
				void transmit();

				void clearTXBuffer();

				TagMap& operator<<( const TagPointer& );

			private:
				QByteArray produce() const;

				rec::serialport::SerialPort* _serial;
				ProducerDelegate* _delegate;

				TagMap _txBuffer;
				QVector<QString> _sigVec;
			};
		}
	}
}

#endif //_REC_ROBOTINO3_SERIALIO_PRODUCER_H_
