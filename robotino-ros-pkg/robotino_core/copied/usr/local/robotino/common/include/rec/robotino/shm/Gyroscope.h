//  Copyright (C) 2004-2009, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_SHM_GYROSCOPE_H_
#define _REC_ROBOTINO_SHM_GYROSCOPE_H_

#include "rec/robotino/daemons/Log.h"

#include <QtCore>

#define REC_ROBOTINO_SHM_GYROSCOPE_KEY "REC_ROBOTINO_SHM_GYROSCOPE_KEY"

namespace rec
{
	namespace robotino
	{
		namespace shm
		{
			class Gyroscope
			{
			public:
				typedef enum {
					CruizCore_XG1010
				} HardwareModel;

				Gyroscope()
					: _data( NULL )
					, _sequenceCounter( 0 )
				{
				}

				~Gyroscope()
				{
					detach();
				}

				void attach( bool doCreate = false )
				{
					detach();
					_sequenceCounter = 0;

					_shm.setKey( REC_ROBOTINO_SHM_GYROSCOPE_KEY );

					if( doCreate )
					{
						if( false == _shm.create( sizeof( Data ) ) )
						{
							rec::robotino::daemons::Log::singleton()->log( QString("Gyroscope SHM: %1").arg(_shm.errorString()) , 0 );
						}
						else
						{
							rec::robotino::daemons::Log::singleton()->log( "Gyroscope SHM", 0 );
						}

						if( QSharedMemory::AlreadyExists == _shm.error() )
						{
							_shm.attach();
						}
					}
					else
					{
						_shm.attach();
					}

					_data = static_cast<Data*>( _shm.data() );

					if( _data )
					{
						if( doCreate )
						{
							if( _shm.lock() )
							{
								_data->angleX = 0.0f;
								_data->angleY = 0.0f;
								_data->angleZ = 0.0f;
								_data->accelX = 0.0f;
								_data->accelY = 0.0f;
								_data->accelZ = 0.0f;
								_data->hwmodel = CruizCore_XG1010;
								_data->rate = 0.0f;
								_data->sequenceCounter = 0;
								_shm.unlock();
							}
							else
							{
								rec::robotino::daemons::Log::singleton()->log( "Error locking Gyroscope SHM", 0 );
							}
						}
					}
					else
					{
						rec::robotino::daemons::Log::singleton()->log( "Gyroscope SHM is NULL", 0 );
					}
				}

				void detach()
				{
					_data = NULL;

					if( _shm.isAttached() )
					{
						_shm.detach();
						rec::robotino::daemons::Log::singleton()->log( "Detached from Gyroscope SHM", 0 );
					}
				}

				bool isNewDataAvailable()
				{
					if( _data )
					{
						if( _data->sequenceCounter != _sequenceCounter )
						{
							_sequenceCounter = _data->sequenceCounter;
							return true;
						}
						else
						{
							return false;
						}
					}
					else
					{
						return false;
					}
				}

				bool lock()
				{
					if( _data )
					{
						return _shm.lock();
					}
					else
					{
						return false;
					}
				}

				void unlock()
				{
					_shm.unlock();
				}


				float& angleX() { return _data->angleX; }
				float& angleY() { return _data->angleY; }
				float& angleZ() { return _data->angleZ; }
				float& accelX() { return _data->accelX; }
				float& accelY() { return _data->accelY; }
				float& accelZ() { return _data->accelZ; }
				int& hwmodel() { return _data->hwmodel; }
				float& rate() { return _data->rate; }

				unsigned int& sequenceCounter() { return _data->sequenceCounter; }

			private:
				QSharedMemory _shm;

				unsigned int _sequenceCounter;

				struct Data
				{
					float angleX;
					float angleY;
					float angleZ;
					float accelX;
					float accelY;
					float accelZ;
					float rate;
					int hwmodel;
					unsigned int sequenceCounter;
				};

				Data* _data;
			};
		}
	}
}

#endif //_REC_ROBOTINO_SHM_GYROSCOPE_H_
