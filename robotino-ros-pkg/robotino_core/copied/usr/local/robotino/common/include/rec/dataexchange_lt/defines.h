//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_DATAEXCHANGE_DEFINES_H_
#define _REC_DATAEXCHANGE_DEFINES_H_

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/*
 * Defines that help to correctly specify dllexport and dllimport on
 * Windows platforms.
 */
/*
#ifdef WIN32
#  ifdef rec_dataexchange_EXPORTS
#    define REC_DATAEXCHANGE_EXPORT __declspec(dllexport)
#  else
#    define REC_DATAEXCHANGE_EXPORT __declspec(dllimport)
#  endif // rec_core_EXPORTS
#else // WIN32
#  define REC_DATAEXCHANGE_EXPORT
#endif // WIN32
*/

#  define REC_DATAEXCHANGE_EXPORT

#endif // _REC_DATAEXCHANGE_DEFINES_H_
