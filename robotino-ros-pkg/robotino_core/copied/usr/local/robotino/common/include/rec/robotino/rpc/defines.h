/*
Copyright (c) 2011, REC Robotics Equipment Corporation GmbH, Planegg, Germany
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.
- Neither the name of the REC Robotics Equipment Corporation GmbH nor the names of
  its contributors may be used to endorse or promote products derived from this software
  without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _REC_ROBOTINO_RPC_DEFINES_H_
#define _REC_ROBOTINO_RPC_DEFINES_H_

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/*
 * Defines that help to correctly specify dllexport and dllimport on
 * Windows platforms.
 */

#ifdef WIN32
#  ifdef rec_robotino_rpc_EXPORTS
#    define REC_ROBOTINO_RPC_EXPORT __declspec(dllexport)
#  else
#    define REC_ROBOTINO_RPC_EXPORT __declspec(dllimport)
#  endif // rec_rpc_EXPORTS
#else // WIN32
#  define REC_ROBOTINO_RPC_EXPORT
#endif // WIN32

#ifndef REC_ROBOTINO_RPC_FUNCTION_IS_NOT_USED
#ifdef __GNUC__
#define REC_ROBOTINO_RPC_FUNCTION_IS_NOT_USED __attribute__ ((unused))
#else
#define REC_ROBOTINO_RPC_FUNCTION_IS_NOT_USED
#endif
#endif

#ifdef REC_ROBOTINO_RPC_STATIC
#undef REC_ROBOTINO_RPC_EXPORT
#define	REC_ROBOTINO_RPC_EXPORT
#endif

#endif // _REC_ROBOTINO_RPC_DEFINES_H_
