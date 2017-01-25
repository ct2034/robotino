#ifndef _REC_NSTAR2_PRODUCER_H_
#define _REC_NSTAR2_PRODUCER_H_

#include <QtCore>
#include "rec/nstar2/tag/Tag.h"

namespace rec
{
	namespace serialport
	{
		class SerialPort;
	}

	namespace nstar2
	{
		class Com;

		class Producer
		{
		public:
			Producer( rec::serialport::SerialPort* serial, Com* com );

			/**
			* @throws rec::serialport::SerialPortException
			*/
			void send_startup();

			/**
			* @throws rec::serialport::SerialPortException
			*/
			void send_set_automatic_report( bool enable );

			void send_get_info();

			void send_set_nodeids( const QList<int>& nodeids );

			void send_set_baudrate( int baudrate );

		private:
			QByteArray produce( const tag::TagList& tagList ) const;
			QByteArray produce_startup() const;
			QByteArray produce_set_automatic_report( bool enable ) const;
			QByteArray produce_get_info() const;
			QByteArray produce_set_nodeids( const QList<int>& nodeids ) const;
			QByteArray produce_set_baudrate( int baudrate ) const;

			rec::serialport::SerialPort* _serial;
			Com* _com;
		};
	}
}

#endif //_REC_NSTAR2_PRODUCER_H_
