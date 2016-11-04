#ifndef REC_PCAN_PCAN_PCANDEV_H
#define REC_PCAN_PCAN_PCANDEV_H

#include "rec/pcan/Types.h"
#include "rec/pcan/CanMessage.h"

namespace rec
{
	namespace pcan
	{
		/*!
		* Enumeration holding the CAN baud rates.
		*/
		enum CANBaudRate {
			br1M	= 0x0014,
			br500K	= 0x001C,
			br250K	= 0x011C,
			br125K	= 0x031C,
			br100K	= 0x432F,
			br50K	= 0x472F,
			br20K	= 0x532F,
			br10K	= 0x672F,
			br5K	= 0x7F7F
		};

		/*!
		*  CAN Error and status values
		*/
		enum CANResult
		{
			canOK				= 0x0000,		/*!< No error */
			canTxFull			= 0x0001,   	/*!< Send buffer of the controller is full */
			canRxOverrun		= 0x0002,   	/*!< Overrun in receive buffer of the controller */
			canBusLight			= 0x0004,   	/*!< Bus error: an error count reached the limit */
			canBusHeavy			= 0x0008,   	/*!< Bus error: an error count reached the limit */
			CanBusOff			= 0x0010,   	/*!< Bus error: CAN controller went to 'Bus-Off' */
			canRxQueueEmpty		= 0x0020,   	/*!< RcvQueue is empty */
			canRxQueueOverrun	= 0x0040,   	/*!< RcvQueue was read to late */
			canTxQueueFull		= 0x0080,   	/*!< Send queue is full */
			canRegTestFailed	= 0x0100,   	/*!< RegisterTest of the controller failed */
			canNoVxD			= 0x0200,   	/*!< Problem with localization of the VxD (Windows only) */
			canErrResource		= 0x2000,   	/*!< Error on creating a resource (FIFO, client, timeout) */
			canErrParam			= 0x4000,   	/*!< Parameter not permitted */
			canErrParamVal		= 0x8000,   	/*!< Invalid parameter value */
			canErrHandle		= 0x1C00,   	/*!< Mask for all handle errors */
		};

		/*!
		* Forward declaration of class PCanDevImpl.
		*/
		class PCanDevImpl;

		/*!
		* Class to access a PEAK Systems GmbH PCAN USB device.
		*/
		class PCanDev
		{
		public:

			PCanDev();

			~PCanDev();

			bool busReady();

			void write(CanMessage& message);

			void read(CanMessage& message);

			std::string versionInfo() const;

			void msgFilter(uint32 fromID, uint32 toID, CANMsgType msgType);

			void resetFilter();

			virtual void log( const std::string& message ) { };

		private:
			PCanDevImpl* _impl;
		};

	}
}

#endif	// REC_PCAN_PCAN_PCANDEV_H
