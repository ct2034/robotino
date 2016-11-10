//  Copyright (C) 2004-2010, Robotics Equipment Corporation GmbH

#ifndef _REC_GRAPPLER_DEFINES_H_
#define _REC_GRAPPLER_DEFINES_H_

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/*
 * Defines that help to correctly specify dllexport and dllimport on
 * Windows platforms.
 */

/*
#ifdef WIN32
#  ifdef rec_grappler_EXPORTS
#    define REC_GRAPPLER_EXPORT __declspec(dllexport)
#  else
#    define REC_GRAPPLER_EXPORT __declspec(dllimport)
#  endif // rec_grappler_EXPORTS
#else // WIN32
#  define REC_GRAPPLER_EXPORT
#endif // WIN32
*/

#  define REC_GRAPPLER_EXPORT

#endif // _REC_GRAPPLER_DEFINES_H_
