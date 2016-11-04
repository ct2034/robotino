#ifndef _REC_LASERRANGEFINDER_SCANNER_H_
#define _REC_LASERRANGEFINDER_SCANNER_H_

#include "rec/laserrangefinder/defines.h"
#include "rec/LaserScanMessage.h"

#include <QVariant>

#include <string>
#include <map>

namespace rec
{
	namespace laserrangefinder
	{
		class REC_LASERRANGEFINDER_EXPORT Scanner
		{
		public:
			Scanner();

			virtual ~Scanner();


			/**
			@brief sets beginning and end of the scanner's field of view.
			Angles range from -180 to -180. If the specified values exceed the
			scanners field of view, the configuration is set to the next value inside
			the scanners fov specs.
			@param angleBegin must be in interval [-180, angleEnd]
			@param angleEnd must be in interval [angleBegin, +180]
			**/
			virtual void setFieldOfView( const double minimumAngle, const double maximumAngle) = 0;

			/**
			@throws nothing
			*/
			virtual bool isDeviceOpen() const = 0;

			/**
			@return Returns true on success. Returns false otherwise.
			@throws nothing
			*/
			virtual bool openDevice( const std::string& port, unsigned int speed ) = 0;

			/**
			@throws nothing
			*/
			virtual void closeDevice() = 0;

			/**
			@throws nothing
			*/
			void setCustomConfiguration( const std::string& name, const QVariant& value )
			{
				_customConfiguration[ name ] = value;
			}

		protected:
			std::map< std::string, QVariant > _customConfiguration;
		};

	}
}

#endif //_REC_LASERRANGEFINDER_SCANNER_H_
