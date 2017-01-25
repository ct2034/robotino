#ifndef _REC_NSTAR2_COM_H_
#define _REC_NSTAR2_COM_H_

#include "rec/nstar2/Report.h"

namespace rec
{
	namespace serialport
	{
		class SerialPort;
	}

	namespace nstar2
	{
		class Producer;
		class Parser;

		class Com
		{
		public:
			Com();
			virtual ~Com();

			void setVerbosity( int verbosity );
			int verbosity() const { return _verbosity; }

			bool open( const char* device );
			void close();

			const char* version() const;

			unsigned int speed() const;

			bool setAutomaticReportEnabled( bool enable );

			bool getSensorInfo();

			bool setNodeIDs( const int* nodeids, int size );

			bool setBaudrate( int baudrate );

			bool parse();

			virtual void log( const char* message ) const;

			virtual void report( const Report& report );

		private:
			rec::serialport::SerialPort* _serial;
			Producer* _producer;
			Parser* _parser;

			int _verbosity;
			char* _version;
		};
	}
}

#endif //_REC_NSTAR2_COM_H_
