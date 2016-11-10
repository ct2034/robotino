#ifndef _REC_CRUIZCORE_COM_H_
#define _REC_CRUIZCORE_COM_H_

#include <QtCore>
#include "rec/serialport/Port.h"

namespace rec
{
	namespace cruizcore
	{
		class ComImpl;

		class
#ifdef WIN32
#  ifdef rec_cruizcore_EXPORTS
		__declspec(dllexport)
#  endif
#endif
		Com
		{
		public:
			Com();

			virtual ~Com();

			/**
			* @param port The port CruizCore is connected to. If port is UndefinedPort this method scan the available ports for a connected CruizCore gyroscope.
			*/
			bool open( rec::serialport::Port port = rec::serialport::UNDEFINED );

			bool open( const QString& port );

			bool isRunning() const;

			void close();

			QString portString() const;

			rec::serialport::Port port() const;

			/**
			* @param rate Rate in rad/s
			* @param angle Angle in rad
			*/
			virtual void dataReceivedEvent( float rate, float angle );

			virtual void errorReport( const char* error );

		private:
			ComImpl* _impl;
		};
	}
}

#endif
