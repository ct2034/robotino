#ifndef _REC_LASERRANGEFINDER_HOKUYO_H_
#define _REC_LASERRANGEFINDER_HOKUYO_H_

#include "rec/laserrangefinder/defines.h"
#include "rec/laserrangefinder/Scanner.h"
#include "rec/LaserScanMessage.h"
#include "hokuyo/HokuyoDriver.h"

#include <QTime>

namespace rec
{
	namespace laserrangefinder
	{
		class REC_LASERRANGEFINDER_EXPORT Hokuyo : public Scanner, private HokuyoDriver
		{
			
		public:
			Hokuyo();
			virtual ~Hokuyo();

			void setFieldOfView( const double minimumAngle, const double maximumAngle );

			bool isDeviceOpen() const;

			bool openDevice( const std::string& port, unsigned int speed );

			void closeDevice();

			virtual void logCallback( const std::string& message );

			virtual void scanCallback( const rec::LaserScanMessage& scan );

		private:
			void infoMessage( const std::string& message );
			void warnMessage( const std::string& message );
			void debugMessage( const std::string& message );
			void errorMessage( const std::string& message );
			void hokuyoScanCallback( const hokuyo::LaserScan& scan );

			QTime _scan_timer;
		};
	}
}

#endif //_REC_LASERRANGEFINDER_HOKUYO_H_
