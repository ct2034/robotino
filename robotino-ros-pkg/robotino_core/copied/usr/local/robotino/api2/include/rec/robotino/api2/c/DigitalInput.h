//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_API2_C_DIGITALINPUT_H_
#define _REC_ROBOTINO_API2_C_DIGITALINPUT_H_

#include "rec/robotino/api2/c/globals.h"
#include "rec/robotino/api2/c/Com.h"

/** \file DigitalInput.h
    \brief In "rec/robotino/api2/c/DigitalInput.h" you can find functions for reading Robotino's bumper.

		Use DigitalInput_construct() to create a new Digital input object. Associate the bumper object with a com object using DigitalInput_setComId().
		Use DigitalInput_value() to read the Digital input's state.
*/

/** DigitalInputId */
typedef int DigitalInputId;

/** Invalid DigitalInputId is -1 */
#define INVALID_DIGITALINPUTID -1

/**
Construct an digital input object
@return Returns the ID of the newly constructed digital input object.
@param n	The input number. Range [0; numDigitalInputs()-1]
*/
DLLEXPORT DigitalInputId DigitalInput_construct( unsigned int n );

/**
Destroy the Digital input object assigned to id
@param id The id of the digital input object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given DigitalInputId is invalid.
*/
DLLEXPORT BOOL DigitalInput_destroy( DigitalInputId id );

/**
Associated a digital input object with a communication interface, i.e. binding the digital input to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given DigitalInputId or ComId is invalid.
*/
DLLEXPORT BOOL DigitalInput_setComId( DigitalInputId id, ComId comId );
/**
 * Sets the number of this digital input device.
 *
 * @param id The id of the digital input object to be set
 * @param n	The input number. Range [0 - numDigitalInputs()]
 * @throws	RobotinoException if the given input number is invalid.
 */
DLLEXPORT BOOL DigitalInput_setInputNumber( DigitalInputId id, unsigned int n );


/**
* @return Returns the number of digital inputs.
*/
DLLEXPORT unsigned int numDigitalInputs();

/**
Returns the current value of the specified input device.
@param id The id of the digital input object to be read
@return	The current value of the specified digital input
*/
DLLEXPORT BOOL DigitalInput_value( DigitalInputId id );

#endif //_REC_ROBOTINO_API2_C_DIGITALINPUT_H_
