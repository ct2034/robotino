#ifndef _REC_JOYSTICK_JOYSTICK_H_
#define _REC_JOYSTICK_JOYSTICK_H_

#include <vector>
#include <string>

namespace rec
{
	namespace joystick
	{
		class JoystickImpl;

		class Joystick
		{
		public:
			class State
			{
			public:
				static const unsigned int numAxes = 16;
				static const unsigned int numButtons = 16;

				State()
					: axes( numAxes )
					, buttons( numButtons )
				{
					reset();
				}

				void reset()
				{
					isConnected = false;
					for( unsigned int i = 0; i < numAxes; ++i )
					{
						axes[i] = 0;
						buttons[i] = false;
					}
				}

				bool isConnected;
				std::vector< int > axes;
				std::vector< bool > buttons;
			};

			Joystick();
			~Joystick();

			std::vector< std::string > getAvailableJoysticks() const;
			void selectDevice( int devIndex );

			/**
			@throws nothing
			*/
			void getCapabilities( int* axes, int* buttons ) const;

			/**
			@return If no device is selected or if the device name can not be retrieved returns an empty string.
			        Otherwise returns the device instance name.
			@throws nothing
			*/
			std::string getDeviceInstanceName() const;

			/**
			@throws nothing
			*/
			void getState( State* state ) const;

		private:
			JoystickImpl* _impl;
		};

		class JoystickException : public std::exception
		{
		public:
			JoystickException( const std::string& message_ )
				: message( message_ )
			{
			}

			virtual ~JoystickException() throw ()
			{
			}

			virtual const char* what() const throw ()
			{
				return message.c_str();
			}

			std::string message;
		};
	}
}

#endif
