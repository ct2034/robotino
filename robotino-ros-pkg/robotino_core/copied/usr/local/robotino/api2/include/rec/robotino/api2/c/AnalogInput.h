//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_API2_C_ANALOGINPUT_H_
#define _REC_ROBOTINO_API2_C_ANALOGINPUT_H_

#include "rec/robotino/api2/c/globals.h"
#include "rec/robotino/api2/c/Com.h"

/** \file AnalogInput.h
    \brief In "rec/robotino/api2/c/AnalogInput.h" you can find functions for reading Robotino's bumper.

		Use AnalogInput_construct() to create a new analog input object. Associate the bumper object with a com object using AnalogInput_setComId().
		Use AnalogInput_value() to read the analog input's state.
*/

/** AnalogInputId */
typedef int AnalogInputId;

/** Invalid AnalogInputId is -1 */
#define INVALID_ANALOGINPUTID -1

/**
Construct an analog input object
@return Returns the ID of the newly constructed analog input object.
@param n	The input number. Range [0; numAnalogInputs()-1]
*/
DLLEXPORT AnalogInputId AnalogInput_construct( unsigned int n );

/**
Destroy the analog input object assigned to id
@param id The id of the analog input object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given AnalogInputId is invalid.
*/
DLLEXPORT BOOL AnalogInput_destroy( AnalogInputId id );

/**
Associated a analog input object with a communication interface, i.e. binding the analog input to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given AnalogInputId or ComId is invalid.
*/
DLLEXPORT BOOL AnalogInput_setComId( AnalogInputId id, ComId comId );

/**
 * Sets the number of this analog input device.
 *
 * @param id The id of the digital input object to be set
 * @param n	The input number. Range [0 - numAnalogInputs()]
 * @throws	Returns TRUE (1) on success otherwise FALSE (0)
 */
DLLEXPORT BOOL AnalogInput_setInputNumber( AnalogInputId id, unsigned int n );

/**
* @return Returns the number of analog inputs.
*/
DLLEXPORT unsigned int numAnalogInputs();


/**
Returns the current value of the specified input device.
@param id The id of the analog input object to be read
@return	The current value of the specified analog input. Returns -1 if the given AnalogInputId is invalid or if this analog input is not connected to a valid com object.
*/
DLLEXPORT float AnalogInput_value( AnalogInputId id );

#endif //_REC_ROBOTINO_API2_C_ANALOGINPUT_H_
