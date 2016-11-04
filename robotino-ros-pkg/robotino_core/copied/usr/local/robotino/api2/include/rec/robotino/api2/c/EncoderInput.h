//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_API2_C_ENCODERINPUT_H_
#define _REC_ROBOTINO_API2_C_ENCODERINPUT_H_

#include "rec/robotino/api2/c/globals.h"
#include "rec/robotino/api2/c/Com.h"

/** \file EncoderInput.h
    \brief In "rec/robotino/api2/c/EncoderInput.h" you can find functions for reading Robotino's bumper.

		Use EncoderInput_construct() to create a new encoder input object. Associate the bumper object with a com object using EncoderInput_setComId().
		Use EncoderInput_value() to read the encoder input's state.
*/

/** EncoderInputId */
typedef int EncoderInputId;

/** Invalid EncoderInputId is -1 */
#define INVALID_ENCODERINPUTID -1

/**
Construct an encoder input object
@return Returns the ID of the newly constructed encoder input object.
*/
DLLEXPORT EncoderInputId EncoderInput_construct();

/**
Destroy the encoder input object assigned to id
@param id The id of the encoder input object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given EncoderInputId is invalid.
*/
DLLEXPORT BOOL EncoderInput_destroy( EncoderInputId id );

/**
Associated a encoder input object with a communication interface, i.e. binding the encoder input to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given EncoderInputId or ComId is invalid.
*/
DLLEXPORT BOOL EncoderInput_setComId( EncoderInputId id, ComId comId );

/**
Set the current position to zero.
@param id The id of the encoder input object to be read
@return	Returns TRUE (1) on success. Returns FALSE (0) if the given EncoderInputId is invalid.
*/
DLLEXPORT BOOL EncoderInput_resetPosition( EncoderInputId id );

/**
@param id The id of the encoder input object to be read
@return Actual position in ticks since power on or EncoderInput_resetPosition
*/
DLLEXPORT int EncoderInput_position( EncoderInputId id );

/**
@param id The id of the encoder input object to be read
@return The actual velocity in ticks/s
*/
DLLEXPORT int EncoderInput_velocity( EncoderInputId id );

#endif //_REC_ROBOTINO_API2_C_ENCODERINPUT_H_
