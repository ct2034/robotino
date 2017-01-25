#ifndef _REC_ROBOTINO3_SERIALIO_COM_H_
#define _REC_ROBOTINO3_SERIALIO_COM_H_

#include "rec/robotino3/serialio/Parser.h"
#include "rec/robotino3/serialio/Producer.h"
#include "rec/robotino3/serialio/Decoder.h"

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
			class Com : public rec::robotino3::serialio::ParserDelegate, public rec::robotino3::serialio::ProducerDelegate
			{
			public:
				Com( rec::robotino3::serialio::Decoder* decoder );
				
				virtual ~Com();

#ifdef WIN32
				/**
				@param vendorId
				@param productId
				"USB\\VID_1E29&PID_040D"
				*/
				void setUsbId( int vendorId, int productId );
#else
				/**
				@param name "REC_GmbH_Robotino_3"
				*/
				void setUsbName( const QString& name );
#endif

				bool isOpen() const;

				bool open();

				bool open( const char* deviceName );

				void close();

				void setVerbosity( int verbosity );

				int verbosity() const;
				
				void setReadTimeout( unsigned int timeout );

				void getHardwareVersion();
				void getSoftwareVersion();

				/**
				@return Returns number of TAGs received.
				*/
				int parse();

				/**
				Transmit current message buffer to microcontroller
				*/
				void transmit();

				/**
				Clear TX buffer
				*/
				void clearTXBuffer();

				void produce( const TagHelper& p );

			protected:
				virtual void hardwareVersionCb( const char* version );
				virtual void softwareVersionCb( const char* version );
				virtual void logCb( const char* message );

				virtual void parse_i( rec::robotino3::serialio::TagPointer p ) = 0;
			private:

				rec::serialport::SerialPort* _serial;
				rec::robotino3::serialio::Producer* _producer;
				rec::robotino3::serialio::Parser* _parser;
				rec::robotino3::serialio::Decoder* _decoder;

				int _verbosity;
#ifdef WIN32
				QString _usbId;
#else
				QString _usbName;
#endif
				int parserDelegateVerbosity() const;
				void parserDelegateLog( const QString& message );
				int producerDelegateVerbosity() const;
				void producerDelegateLog( const QString& message );
			};
		}
	}
}

#endif //_REC_ROBOTINO3_SERIALIO_COM_H_
