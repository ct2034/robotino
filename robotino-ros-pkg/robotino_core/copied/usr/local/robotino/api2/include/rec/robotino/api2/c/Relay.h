//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_API2_C_RELAY_H_
#define _REC_ROBOTINO_API2_C_RELAY_H_

#include "rec/robotino/api2/c/globals.h"
#include "rec/robotino/api2/c/Com.h"

/** \file Relay.h
\brief In "rec/robotino/api2/c/Relay.h" you can find functions for reading Robotino's bumper.

Use Relay_construct() to create a new relay object. Associate the relay object with a com object using Relay_setComId().
Use Relay_setSetPointSpeed() to set the relay's speed.
*/

/** RelayId */
typedef int RelayId;

/** Invalid RelayId is -1 */
#define INVALID_RELAYID -1

/**
Construct an relay object
@param number	number of this relay. Range [0; numRelays()-1].
@return Returns the ID of the newly constructed relay object.
*/
DLLEXPORT RelayId Relay_construct( unsigned int number );

/**
Destroy the relay object assigned to id
@param id The id of the relay object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given RelayId is invalid.
*/
DLLEXPORT BOOL Relay_destroy( RelayId id );

/**
Associated a relay object with a communication interface, i.e. binding the relay to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given RelayId or ComId is invalid.
*/
DLLEXPORT BOOL Relay_setComId( RelayId id, ComId comId );

/**
 * Sets the relay number.
 *
 * @param id The id of the relay object to be set
 * @param n	The relay number. Range: [0 - numRelays()]
 * @throws	RobotinoException if relay number is invalid.
 */
DLLEXPORT BOOL Relay_setRelayNumber( RelayId id, unsigned int n );

/**
@return Returns the number of drive relays on Robotino
*/
DLLEXPORT unsigned int numRelays();

/**
Sets the setpoint speed of this relay.
@param id The id of the relay object.
@param on	The value the relay will be set to.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given RelayId is invalid.
*/
DLLEXPORT BOOL Relay_setValue( RelayId id, BOOL on );

#endif //_REC_ROBOTINO_API2_C_RELAY_H_
