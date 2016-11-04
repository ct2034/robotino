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

#ifndef _REC_RPC_SERIALIZATION_POSE2D_H_
#define _REC_RPC_SERIALIZATION_POSE2D_H_

#include "rec/rpc/serialization/Serializable.h"
#include "rec/rpc/serialization/Primitive.h"
#include "rec/rpc/defines.h"

#include <QtCore>

namespace rec
{
	namespace rpc
	{
		namespace serialization
		{
			/*! \brief Pre-defined serialization class that stores a 2D pose. */
			class REC_RPC_EXPORT Pose2D : public Serializable
			{
			public:
				/*! Creates a container storing position ( 0, 0 ) and rotation 0. */
				Pose2D();
				/*! Creates a container storing position ( x, y ) and rotation rotDeg (degrees). */
				Pose2D( const QPointF& pos, float rotDeg );

				/*! \brief X coordinate. */
				QPointF pos;

				/*! \brief Rotation (degrees). */
				float rotDeg;

			private:
				virtual void serialize( QDataStream& ) const;
				virtual void deserialize( QDataStream& );
			};

			/*! \brief Shared pointer type definition. */
			typedef QSharedPointer< Pose2D > Pose2DPtr;

			/*! \brief Serializable array type definition. */
			typedef Primitive< QVector< Pose2D > > Pose2DArray;
			/*! \brief Serializable array shared pointer type definition. */
			typedef QSharedPointer< Pose2DArray > Pose2DArrayPtr;
		}
	}
}

/*!
 *  \brief Complex data container member definition for 2D poses.
 *
 *  Place this macro into the declaration of a rec::rpc::serialization::Complex subclass to add a 2d pose.
 *  The value can be accessed via NAME() (const and non-const), shared pointers can be accessed via NAMEPtr() and NAMEPtrConst().
 *
 *  \param NAME Member name.
 *
 *  \sa rec::rpc::serialization::Complex
 */
#define DECLARE_POSE2D_MEMBER( NAME ) \
	private: \
		rec::rpc::serialization::Pose2DPtr _##NAME; \
		void create_##NAME() { _##NAME = rec::rpc::serialization::Pose2DPtr( new rec::rpc::serialization::Pose2D ); } \
	public: \
		const rec::rpc::serialization::Pose2D& NAME() const { return *_##NAME; } \
		rec::rpc::serialization::Pose2D& NAME() { return *_##NAME; } \
		void set##NAME( const rec::rpc::serialization::Pose2D& pos ) { *_##NAME = pos; } \
		const rec::rpc::serialization::Pose2DPtr& NAME##PtrConst() const { return _##NAME; } \
		rec::rpc::serialization::Pose2DPtr& NAME##Ptr() { return _##NAME; }

/*!
 *  \brief Function parameter type definition.
 *
 *  \param FUNCTIONNAME Name of the RPC function.
 */
#define DEFINE_POSE2D_PARAM( FUNCTIONNAME ) DEFINE_SERIALIZABLE( FUNCTIONNAME##Param, rec::rpc::serialization::Pose2D );

/*!
 *  \brief Function result type definition.
 *
 *  \param FUNCTIONNAME Name of the RPC function.
 */
#define DEFINE_POSE2D_RESULT( FUNCTIONNAME ) DEFINE_SERIALIZABLE( FUNCTIONNAME##Result, rec::rpc::serialization::Pose2D );

/*!
 *  \brief Topic data type definition.
 *
 *  \param TOPICNAME Name of the topic.
 */
#define DEFINE_POSE2D_TOPICDATA( TOPICNAME ) DEFINE_SERIALIZABLE( topic##TOPICNAME##Data, rec::rpc::serialization::Pose2D );

/*!
 *  \brief Complex data container member definition for arrays of 2D poses.
 *
 *  Place this macro into the declaration of a rec::rpc::serialization::Complex subclass to add an array of 2D poses.
 *  The value can be accessed via NAME() (const and non-const), shared pointers can be accessed via NAMEPtr() and NAMEPtrConst().
 *
 *  \param NAME Member name.
 *
 *  \sa rec::rpc::serialization::Complex
 */
#define DECLARE_POSE2DARRAY_MEMBER( NAME ) \
	private: \
		rec::rpc::serialization::Pose2DArrayPtr _##NAME; \
		void create_##NAME() { _##NAME = rec::rpc::serialization::Pose2DArrayPtr( new rec::rpc::serialization::Pose2DArray ); } \
	public: \
		const rec::rpc::serialization::Pose2DArray& NAME() const { return *_##NAME; } \
		rec::rpc::serialization::Pose2DArray& NAME() { return *_##NAME; } \
		void set##NAME( const rec::rpc::serialization::Pose2DArray& pos ) { *_##NAME = pos; } \
		const rec::rpc::serialization::Pose2DArrayPtr& NAME##PtrConst() const { return _##NAME; } \
		rec::rpc::serialization::Pose2DArrayPtr& NAME##Ptr() { return _##NAME; }

/*!
 *  \brief Function parameter type definition.
 *
 *  \param FUNCTIONNAME Name of the RPC function.
 */
#define DEFINE_POSE2DARRAY_PARAM( FUNCTIONNAME ) DEFINE_SERIALIZABLE( FUNCTIONNAME##Param, rec::rpc::serialization::Pose2DArray );

/*!
 *  \brief Function result type definition.
 *
 *  \param FUNCTIONNAME Name of the RPC function.
 */
#define DEFINE_POSE2DARRAY_RESULT( FUNCTIONNAME ) DEFINE_SERIALIZABLE( FUNCTIONNAME##Result, rec::rpc::serialization::Pose2DArray );

/*!
 *  \brief Topic data type definition.
 *
 *  \param TOPICNAME Name of the topic.
 */
#define DEFINE_POSE2DARRAY_TOPICDATA( TOPICNAME ) DEFINE_SERIALIZABLE( topic##TOPICNAME##Data, rec::rpc::serialization::Pose2DArray );

Q_DECLARE_METATYPE( rec::rpc::serialization::Pose2D )
Q_DECLARE_METATYPE( rec::rpc::serialization::Pose2DPtr )
Q_DECLARE_METATYPE( rec::rpc::serialization::Pose2DArray )
Q_DECLARE_METATYPE( rec::rpc::serialization::Pose2DArrayPtr )

#endif //_REC_RPC_SERIALIZATION_POSE2D_H_
