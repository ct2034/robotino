//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_API2_C_NORTHSTAR_H_
#define _REC_ROBOTINO_API2_C_NORTHSTAR_H_

#include "rec/robotino/api2/c/globals.h"
#include "rec/robotino/api2/c/Com.h"

/** \file NorthStar.h
\brief In "rec/robotino/api2/c/NorthStar.h" you can find functions for reading Robotino's bumper.

Use NorthStar_construct() to create a new northstar object. Associate the northstar object with a com object using NorthStar_setComId().
*/

/** NorthStarId */
typedef int NorthStarId;

/** Invalid NorthStarId is -1 */
#define INVALID_NORTHSTARID -1

/**
Construct a northstar object
@return Returns the ID of the newlyructed northstar object.
*/
DLLEXPORT NorthStarId NorthStar_construct();

/**
Destroy the northstar object assigned to id
@param id The id of the northstar object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given NorthStarId is invalid.
*/
DLLEXPORT BOOL NorthStar_destroy( NorthStarId id );

/**
Associated a northstar object with a communication interface, i.e. binding the northstar to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given NorthStarId or ComId is invalid.
*/
DLLEXPORT BOOL NorthStar_setComId( NorthStarId id, ComId comId );

/**
* The sequence number is increased whenever th Northstar delivers new data.
@param id The id of the northstar object.
*/
DLLEXPORT unsigned int NorthStar_sequenceNo( NorthStarId id );

/**
* The current room id.
@param id The id of the northstar object.
*/
DLLEXPORT int NorthStar_roomId( NorthStarId id );

/**
* The number of visible spots.
@param id The id of the northstar object.
*/
DLLEXPORT unsigned int NorthStar_numSpotsVisible( NorthStarId id );

/**
* The current position in x direction. The scale depends on the calibration.
@param id The id of the northstar object.
*/
DLLEXPORT int NorthStar_posX( NorthStarId id );

/**
* The current position in y direction. The scale depends on the calibration.
@param id The id of the northstar object.
*/
DLLEXPORT int NorthStar_posY( NorthStarId id );

/**
* The current orientation in radians.
@param id The id of the northstar object.
*/
DLLEXPORT float NorthStar_posTheta( NorthStarId id );

/**
* The signal strength of spot 1.
@param id The id of the northstar object.
*/
DLLEXPORT unsigned int NorthStar_magSpot0( NorthStarId id );

/**
* The signal strength of spot 2.
@param id The id of the northstar object.
*/
DLLEXPORT unsigned int NorthStar_magSpot1( NorthStarId id );

/**
Set the room id.
@param id The id of the northstar object.
@param roomId The current room id.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given NorthStarId is invalid.
*/
DLLEXPORT BOOL NorthStar_setRoomId( NorthStarId id, int roomId );

/**
Deprecated function. Available only for compatibility reasons.
@param id The id of the northstar object.
@param calState Calibration state.
@return Returns FALSE (0)
*/
DLLEXPORT BOOL NorthStar_setCalState( NorthStarId id, unsigned int calState );

/**
Deprecated function. Available only for compatibility reasons.
@param id The id of the northstar object.
@param calFlag Calibration flag.
@return Returns FALSE (0)
*/
DLLEXPORT BOOL NorthStar_setCalFlag( NorthStarId id, unsigned int calFlag );

/**
Deprecated function. Available only for compatibility reasons.
@param id The id of the northstar object.
@param calDistance Calibration distance.
@return Returns FALSE (0)
*/
DLLEXPORT BOOL NorthStar_setCalDistance( NorthStarId id, unsigned int calDistance );

/**
Set the ceiling calibration.
@param id The id of the northstar object.
@param ceilingCal The distance from sensor to ceiling (or wherever the projector is projecting its pattern to) in meters.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given NorthStarId is invalid.
*/
DLLEXPORT BOOL NorthStar_setCeilingCal( NorthStarId id, float ceilingCal );

#endif //_REC_ROBOTINO_API2_C_NORTHSTAR_H_
