#ifndef REC_VEMA_VEMA_H
#define REC_VEMA_VEMA_H

#include "rec/vema/VEMAStatus.h"
#include "rec/vema/VEMAError.h"
#include "rec/vema/Exception.h"
#include "rec/pcan/CanMessage.h"

#include <map>

namespace rec
{
	namespace pcan
	{
		class PCanDev;
	}

	namespace vema
	{

		/*!
		* Enumeration holding the VEMA components that can
		* be accessed by sending a single command.
		*/
		enum VEMAComponent {
			Channel1 = 0x01,
			Channel2,
			Channel3,
			Channel4,
			Channel5,
			Channel6,
			Channel7,
			Channel8,
			ChannelGroup1 = 0x0D,
			ChannelGroup2 = 0x0E,
			AllChannels = 0x0F
		};

		/**
		* An object representing a VEMA valve terminal.
		*/
		class VEMA 
		{

		public:

			VEMA();

			explicit VEMA(uint32 canID);

			virtual ~VEMA();

			/*!
			* Sends a command to the VEMA board via CAN.
			*/
			void sendCmd(unsigned char* data, unsigned int length);

			/*!
			* Sets the VEMA PCB CAN ID.
			*/
			inline void setCANId(uint32 id);

			/*!
			* Starts the closed loop control for the appropriate channel.
			*/
			void startClosedLoop(VEMAComponent chnlNr);

			/*!
			* Stops the closed loop control for the appropriate channel.
			*/
			void stopClosedLoop(VEMAComponent chnlNr);

			/*!
			* Determines the status of a channel.
			*/
			VEMAStatus getStatus(VEMAComponent chnlNr);

			/*!
			* Determines the error state of a channel.
			*/
			VEMAError getError(VEMAComponent chnlNr);

			/*!
			* Determines the current pressure of a channel.
			*/
			int32 getPressure(VEMAComponent chnlNr);

			/*!
			* Determines the pressures of all channels.
			*/
			void getAllPressures(int32* values);

			/*!
			* Sets the pressure set point of the appropriate channel
			* and determines the current pressure.
			*/
			int32 setPressure(VEMAComponent chnlNr, int32 pressure);

			/*!
			* Sets the pressure set point of all channels.
			*/
			void setAllPressures(int* values);

			/*!
			* Resets the error byte of the appropriate valve channel.
			*/
			void resetError(VEMAComponent chnlNr);

			/*!
			* Pressurizes the appropriate valve.
			*/
			void pressurize(VEMAComponent chnlNr);

			/*!
			* Exhausts the appropriate valve.
			*/
			void exhaust(VEMAComponent chnlNr);

			/*!
			* Closes the appropriate valve.
			*/
			void closeValve(VEMAComponent chnlNr);

			/*!
			* Starts the calibration a the appropriate channel.
			*/
			void startCalibration(VEMAComponent chnlNr);

			/*!
			* Sets the calibration for the appropriate channel.
			*/
			void setCalibration(VEMAComponent chnlNr);

			/*!
			* Sets the appropriate digital output.
			*/
			void setDigitalOut(unsigned int nr);

			/*!
			* Resets (clears) the appropriate digital output.
			*/
			void resetDigitalOut(unsigned int nr);

			virtual void log( const std::string& message ) { }


		private:

			enum {
				CMD_START_CLOSED_LOOP = 0x10,
				CMD_STOP_CLOSED_LOOP = 0x20,
				CMD_SET_PRESSURE = 0x30,
				CMD_GET_PRESSURE = 0x40,
				CMD_GET_STATUS = 0x50,
				CMD_GET_ERROR = 0x60,
				CMD_PRESSURIZE = 0x70,
				CMD_EXHAUST = 0x80,
				CMD_CLOSE_VALVE = 0x90,
				CMD_SET_DIGITAL_OUT = 0xA0,
				CMD_RESET_DIGITAL_OUT = 0xB0,
				CMD_START_CALIBRATION = 0xC0,
				CMD_SET_CALIBRATION = 0xD0,
				CMD_RESET_ERROR = 0xE0
			};

			static const unsigned int NUM_CHANNELS = 8;
			static const unsigned int RESPONSE_TIME = 25;	// milliseconds

			void sendSimpleCmd(VEMAComponent chnlNr, unsigned char cmd);

			void send(pcan::CanMessage& txMsg, pcan::CanMessage& rxMsg);

			inline void checkTarget(VEMAComponent target) const;

			unsigned char checksum(unsigned char* data, unsigned int length);

			rec::pcan::PCanDev* _canDevice;
			uint32 _canID;

			std::map< int, int > _intern2extern;
			std::map< int, int > _extern2intern;
		};

	}
}

#endif	// REC_VEMA_VEMA_H
