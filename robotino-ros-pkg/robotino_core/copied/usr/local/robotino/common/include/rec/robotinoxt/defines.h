//  Copyright (C) 2004-2010, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINOXT_DEFINES_H_
#define _REC_ROBOTINOXT_DEFINES_H_

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/*
 * Defines that help to correctly specify dllexport and dllimport on
 * Windows platforms.
 */

/*
#ifdef WIN32
#  ifdef rec_robotinoxt_EXPORTS
#    define REC_ROBOTINOXT_EXPORT __declspec(dllexport)
#  else
#    define REC_ROBOTINOXT_EXPORT __declspec(dllimport)
#  endif // rec_robotinoxt_EXPORTS
#else // WIN32
#  define REC_ROBOTINOXT_EXPORT
#endif // WIN32
*/

#  define REC_ROBOTINOXT_EXPORT

#endif // _REC_ROBOTINOXT_DEFINES_H_
