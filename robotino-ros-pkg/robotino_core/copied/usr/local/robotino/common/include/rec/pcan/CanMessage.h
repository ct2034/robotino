#ifndef REC_PCAN_PCAN_CANMESSAGE_H
#define REC_PCAN_PCAN_CANMESSAGE_H

#include "rec/pcan/Types.h"
#include "rec/pcan/Exception.h"
#include <string.h>
#include <sstream>
#include <iomanip>

namespace rec
{
	namespace pcan
	{

		/*!
		* Enumeration holding the CAN frame types.
		*/
		enum CANMsgType {
			mtStandard = 0,
			mtExtended = 1
		};

		/*!
		* CAN Message
		*/
		class CanMessage
		{
		public:
			/*!
			* Constructor
			*/
			CanMessage()
				: _id(0)
				, _type(mtStandard)
				, _length(0)
			{
			}

			/*!
			* Constructor
			*/
			CanMessage(uint32 id, CANMsgType type, unsigned int length = 0, unsigned char* data = NULL)
				: _id(id)
				, _type(type)
				, _length(length)
			{
				/* Copy data. */
				if (_length > 0)
					memcpy(_data, data, _length);
			}

			/*!
			* Destructor
			*/
			~CanMessage()
			{
			}

			/*!
			* Returns the CAN identifier of the message.
			*/
			inline uint32 id() const
			{
				return _id;
			}

			/*!
			* Sets the CAN identifier of the message.
			*/
			inline void setId(uint32 id)
			{
				_id = id;
			}

			/*!
			* Returns the CAN message type.
			*/
			inline CANMsgType type() const
			{
				return _type;
			}

			/*!
			* Sets the CAN message type.
			*/
			inline void setType(CANMsgType type)
			{
				_type = type;
			}

			/*!
			* Returns the CAN message length.
			*/
			inline unsigned int length() const
			{
				return _length;
			}

			/*!
			* Returns a pointer to the CAN message data.
			*/
			inline unsigned char* data()
			{
				return _data;
			}

			/*!
			* Returns a single CAN message data byte.
			*/
			inline unsigned char data(unsigned int idx) const
			{
				if (idx < _length)
					return _data[idx];
				else
					throw pcan::Exception("Invalid data index");
			}

			/*!
			* Sets the CAN message data and length.
			*/
			inline void setData(unsigned char* data, unsigned int length)
			{
				_length = (length <= 8) ? length : 8;
				memcpy(_data, data, _length);
			}

			/*!
			* Sets a specific byte of the CAN message data.
			*/
			inline void setData(unsigned int byteNr, unsigned char data)
			{
				if (byteNr > 8)
					throw pcan::Exception("Invalid data index");

				_data[byteNr] = data;
			}

			std::string toString() const
			{
				std::ostringstream os;
				os << std::hex;
				os.width( 2 );
				for( int i=0; i<8; ++i )
				{
					os << "0x" << (unsigned int)_data[i];
					if( i<7 )
					{
						os << " ";
					}
				}

				return os.str();
			}

		private:
			uint32 _id;
			CANMsgType _type;
			unsigned int _length;
			unsigned char _data[8];
		};
	}
}

#endif	// REC_PCAN_PCAN_CANMESSAGE_H
