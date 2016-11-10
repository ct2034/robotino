//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_API2_C_DIGITALOUTPUT_H_
#define _REC_ROBOTINO_API2_C_DIGITALOUTPUT_H_

#include "rec/robotino/api2/c/globals.h"
#include "rec/robotino/api2/c/Com.h"

/** \file DigitalOutput.h
    \brief In "rec/robotino/api2/c/DigitalOutput.h" you can find functions for reading Robotino's bumper.

		Use DigitalOutput_construct() to create a new digital output object. Associate the bumper object with a com object using DigitalOutput_setComId().
		Use DigitalOutput_value() to read the digital output's state.
*/

/** DigitalOutputId */
typedef int DigitalOutputId;

/** Invalid DigitalOutputId is -1 */
#define INVALID_DIGITALOUTPUTID -1

/**
Construct an digital output object
@param n	The input number. Range [0; numDigitalOutputs()-1]
@return Returns the ID of the newly constructed digital output object.
*/
DLLEXPORT DigitalOutputId DigitalOutput_construct( unsigned int n );

/**
Destroy the digital output object assigned to id
@param id The id of the digital output object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given DigitalOutputId is invalid.
*/
DLLEXPORT BOOL DigitalOutput_destroy( DigitalOutputId id );

/**
Associated a digital output object with a communication interface, i.e. binding the digital output to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given DigitalOutputId or ComId is invalid.
*/
DLLEXPORT BOOL DigitalOutput_setComId( DigitalOutputId id, ComId comId );

/**
 * Sets the number of this digital output device.
 *
 * @param id The id of the digital input object to be set
 * @param n	The output number. Range [0 - numAnalogInputs()]
 * @return Returns TRUE (1) on success otherwise FALSE (0)
 */
DLLEXPORT BOOL DigitalOutput_setOutputNumber( DigitalOutputId id, unsigned int n );

/**
* @return Returns the number of digital outputs.
*/
DLLEXPORT unsigned int numDigitalOutputs();


/**
Sets the value of this digital output device.
@param id The id of the digital output object to be written
@param on Turn the output on or off
@return Returns TRUE (1) on success otherwise FALSE (0)
*/
DLLEXPORT BOOL DigitalOutput_setValue( DigitalOutputId id, BOOL on );

#endif //_REC_ROBOTINO_API2_C_DIGITALOUTPUT_H_
