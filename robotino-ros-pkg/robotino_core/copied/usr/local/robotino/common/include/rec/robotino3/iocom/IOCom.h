#ifndef _REC_ROBOTINO3_IOCOM_H_
#define _REC_ROBOTINO3_IOCOM_H_

#include "rec/robotino3/serialio/Com.h"

namespace rec
{
	namespace robotino3
	{
		namespace iocom
		{
			class Decoder;

			class IOCom : public rec::robotino3::serialio::Com
			{
			public:
				IOCom();
				
				/**
				See http://wiki.openrobotino.org/index.php?title=Robotino3_IO_protocol
				*/
				void getDistanceSensorReadings();
				void setMotorSpeed( int motor, int speed );
				void getAllMotorSpeeds();
				void setMotorPosition( int motor, int position );
				void getAllMotorPositions();
				void setMotorPidParameters( int motor, float kp, float ki, float kd );
				void getAllMotorPidParameters();
				void setAllDigitalOutputs( unsigned char data );
				void setAllRelays( unsigned char data );
				void setOdometry( float x, float y, float rot );
				void setOdometryRotation( float rot );
				void getOdometry();
				void getAllMotorCurrentReadings();
				void getAllAnalogInputs();
				void getAllDigitalInputs();
				void getBumper();
				void getPowerButton();
				void setFpgaPower( bool on );
				void getFpgaPower();
				void getPwrOkState();
				void setPwrOkState( bool high );
				void setPwm( int channel, int ratio );
				void setMotorOn( int motor, bool on );
				void setPwrBtn( bool high );
				void setSysReset( bool high );
				void getComExpressStates();
				void getAllMotorReadings();
				void getIpAddress();
				void setIpAddress( unsigned int address, unsigned int mask );
				void setEmergencyBumper( bool on );
				void setMotorMode( int motor, int mode );
				void resetLpc( bool enterUSBBootloader );
				void powerOff();
				void setMotorAccelLimits( int motor, float minaccel, float maxaccel );
				void getMotorAccelLimits( int motor );
				void getGyroZAngle();
				void getCanMsg();
				void canMsg( unsigned short id, unsigned char dlc, unsigned char* data );
				void setNrst( bool on );
				void getNrst();
				void setBoot( int val );
				void getBoot();
				void chargerGetVersion( int chargerID );
				void chargerClearError( int chargerID );
				void getBatteryMin();
				void setBatteryMin( float pb, float nimh );
				void getGyroData();
				void getGPAIN( int channel );
				void getVersionBits();
				void getGyroParam();
				void setGyroBias( float bias );
				void setGyroScale( float scale );

				virtual void distanceSensorReadingsCb( const float* readings, const int size );
				virtual void allMotorSpeedsCb( const int* speeds, const int size );
				virtual void allMotorPositionsCb( const int* positions, const int size );
				virtual void allMotorPidParametersCb( const float* kp, const float* ki, const float* kd, const int size );
				virtual void odometryCb( float x, float y, float rot );
				virtual void allMotorCurrentReadingsCb( const float* readings, const int size );
				virtual void allAnalogInputsCb( const float* readings, const int size );
				virtual void allDigitalInputsCb( unsigned char value );
				virtual void bumperCb( bool value );
				virtual void powerButtonCb( bool value );
				virtual void fpgaPowerCb( bool value );
				virtual void pwrOkStateCb( bool high );
				virtual void comExpressStatesCb( bool sus_s3, bool sus_s4, bool sus_s5, bool thrm, bool thrmtrip );
				virtual void allMotorReadingsCb( const int* speeds, const int* positions, const float* currents, const int size );
				virtual void ipAddressCb( const unsigned int address, const unsigned int netmask );
				virtual void powerSourceReadingsCb( float battery_voltage, float system_current, bool ext_power, int num_chargers, const char* batteryType, bool batteryLow, int batteryLowShutdownCounter );
				virtual void motorAccelLimitsCb( int motor, float minaccel, float maxaccel );
				virtual void gyroZAngleCb( float angle, float velocity );
				virtual void canMsgCb( unsigned short id, unsigned char dlc, const unsigned char* data );
				virtual void nrstCb( bool on );
				virtual void bootCb( int val );
				virtual void configResetCb( bool reset );
				virtual void chargerInfoCb( int chargerID, unsigned int time, float batteryVoltage, float chargingCurrent, float bat1temp, float bat2temp, int state_number, const char* state );
				virtual void chargerVersionCb( int chargerID, int major, int minor, int patch );
				virtual void chargerErrorCb( int chargerID, unsigned int time, const char* message );
				virtual void batteryMinCb( float voltage );
				virtual void gyroDataCb( const unsigned int* stamps, const int stampsSize, const float* omegas, const int omegasSize );
				virtual void gpainCb( int channel, float voltage );
				virtual void versionBitsCb( bool version0, bool version1 );
				virtual void gyroParamCb( float bias, float scale );
				virtual void infoCb( const char* message );
				virtual void warningCb( const char* message );
				virtual void errorCb( const char* message );

			private:
				void parse_i( rec::robotino3::serialio::TagPointer p );
			};
		}
	}
}

#endif //_REC_ROBOTINO3_IOCOM_H_
