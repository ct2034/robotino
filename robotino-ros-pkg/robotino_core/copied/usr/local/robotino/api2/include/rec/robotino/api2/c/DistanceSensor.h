//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_API2_C_DISTANCESENSOR_H_
#define _REC_ROBOTINO_API2_C_DISTANCESENSOR_H_

#include "rec/robotino/api2/c/globals.h"
#include "rec/robotino/api2/c/Com.h"

/** \file DistanceSensor.h
    \brief In "rec/robotino/api2/c/DistanceSensor.h" you can find functions for reading Robotino's bumper.

		Use DistanceSensor_construct() to create a new distance sensor object. Associate the bumper object with a com object using DistanceSensor_setComId().
		Use DistanceSensor_value() to read the distance sensor's state.
*/

/** DistanceSensorId */
typedef int DistanceSensorId;

/** Invalid DistanceSensorId is -1 */
#define INVALID_DISTANCESENSORID -1

/**
Construct an distance sensor object for input number 0.
@return Returns the ID of the newly constructed distance sensor object.
*/
DLLEXPORT DistanceSensorId DistanceSensor_constructDefault( void );

/**
Construct an distance sensor object
@return Returns the ID of the newly constructed distance sensor object.
@param n	The input number. Range [0; numDistanceSensors()-1]
*/
DLLEXPORT DistanceSensorId DistanceSensor_construct( unsigned int n );

/**
Destroy the Digital input object assigned to id
@param id The id of the distance sensor object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given DistanceSensorId is invalid.
*/
DLLEXPORT BOOL DistanceSensor_destroy( DistanceSensorId id );

/**
Associated a distance sensor object with a communication interface, i.e. binding the distance sensor to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given DistanceSensorId or ComId is invalid.
*/
DLLEXPORT BOOL DistanceSensor_setComId( DistanceSensorId id, ComId comId );

/**
 * Sets the number of this distance sensor.
 *
 * @param id The id of the distance sensor object to be set
 * @param n	The input number. Range [0 - numDistanceSensors()]
 * @throws	RobotinoException if the given sensor number is invalid.
 */
DLLEXPORT BOOL DistanceSensor_setSensorNumber( DistanceSensorId id, unsigned int n );

/**
* @return Returns the number of distance sensors.
*/
DLLEXPORT unsigned int numDistanceSensors();


/**
Returns the current value of the specified input device.
@param id The id of the distance sensor object to be read
@return	The current value of the specified distance sensor. Returns -1 if the given DistanceSensorId is invalid or if this distance sensor is not connected to a valid com object.
*/
DLLEXPORT float DistanceSensor_voltage( DistanceSensorId id );

/**
Returns the heading of this distance sensor.
@param id The id of the distance sensor object to be read
@return	The heading in degrees. [0; 360]
*/
DLLEXPORT unsigned int DistanceSensor_heading( DistanceSensorId id );

DLLEXPORT int DistanceSensor_num_objects( void );

#endif //_REC_ROBOTINO_API2_C_DISTANCESENSOR_H_
