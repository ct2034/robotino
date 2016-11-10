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

#ifndef _REC_RPC_SERIALIZATION_COMPLEX_H_
#define _REC_RPC_SERIALIZATION_COMPLEX_H_

#include "rec/rpc/serialization/Serializable.h"
#include "rec/rpc/defines.h"

#include <QDataStream>
#include <QList>

namespace rec
{
	namespace rpc
	{
		namespace serialization
		{
			/*!
			 *  \brief Base class for complex serializable data containers.
			 *
			 *  This class can be used as a base class for complex or nested serializable data containers.
			 *  This is useful if a function has more than one parameter or return value or a topic stores more than one single value.
			 *
			 *  Derive from Complex to declare and implement an own complex data container using the preprocessor macros.
			 *
			 *  Every instance of a Complex subclass must have a version string that will also be serialized.
			 *  When deserializing, the version strings are compared, and if they differ, an error will be thrown.
			 *
			 *  It is recommended to use the preprocessor macros to declare a Complex subclass.
			 *
			 *  \sa BEGIN_COMPLEX_DATA_DECLARATION, BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION
			 */
			class REC_RPC_EXPORT Complex : public Serializable
			{
			public:
				/*! Creates a complex instance with a version string. */
				Complex( const QString& versionString );
				virtual ~Complex();

			protected:
				/*!
				 *  Adds a Serializable to the list of children. This must be done in the constructor.
				 *  It is recommended to use the preprocessor macro.
				 *
				 *  \param c The child to be added.
				 *
				 *  \sa ADD_MEMBER
				 */
				void addChild( Serializable* c );

			private:
				void serialize( QDataStream& ) const;
				void deserialize( QDataStream& );

				const QString _versionString;
				QList< Serializable* > _children;
			};
		}
	}
}

/*!
 *  \brief Begin declaration of a custom complex data type.
 *
 *  This macro inserts the code necessary to declare a rec::rpc::serialization::Complex subclass into a header file.
 *  Below this macro, use the appropriate preprocessor macros to declare children.
 *
 *  \param TYPENAME Name of the subclass.
 *
 *  \sa END_COMPLEX_DATA_DECLARATION, BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION
 *  \sa DECLARE_CUSTOM_MEMBER, DECLARE_PRIMITIVE_MEMBER, DECLARE_STRING_MEMBER, DECLARE_BYTEARRAY_MEMBER, DECLARE_IMAGE_MEMBER
 *  \sa DECLARE_Position_MEMBER, DECLARE_Orientation_MEMBER, DECLARE_POSE_MEMBER, DECLARE_POSEARRAY_MEMBER, DECLARE_POSE2D_MEMBER, DECLARE_POSE2DARRAY_MEMBER
 *  \sa USE_COMPLEX_DATA_AS_PARAM, USE_COMPLEX_DATA_AS_RESULT, USE_COMPLEX_DATA_AS_TOPICDATA
 */
#define BEGIN_COMPLEX_DATA_DECLARATION( TYPENAME ) \
	class TYPENAME; \
	typedef QSharedPointer< TYPENAME > TYPENAME##Ptr; \
	typedef QSharedPointer< const TYPENAME > TYPENAME##PtrConst; \
	class TYPENAME : public rec::rpc::serialization::Complex \
	{ \
	public: \
		TYPENAME();

/*!
 *  \brief End declaration of a custom complex data type.
 *
 *  \sa BEGIN_COMPLEX_DATA_DECLARATION
 */
#define END_COMPLEX_DATA_DECLARATION };

/*!
 *  \brief Begin implementation of a custom complex data type's constructor.
 *
 *  This macro inserts the code necessary to implement the constructor of a custom complex data type into a source file.
 *  Below this macro, the preprocessor macro ADD_MEMBER must be used to insert the members to the list of children to be serialized.
 *
 *  \param TYPENAME Name of the data type.
 *  \param VERSION Version string. If data that has been serialized with a type version different from this, it can't be deserialized with this version.
 *
 *  \sa END_COMPLEX_DATA_IMPLEMENTATION, BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION, ADD_MEMBER
 */
#define BEGIN_COMPLEX_DATA_IMPLEMENTATION( TYPENAME, VERSION ) \
	TYPENAME::TYPENAME() \
	: rec::rpc::serialization::Complex( #TYPENAME "_" #VERSION ) \
	{

/*!
 *  \brief End implementation of a custom complex data type's constructor.
 *
 *  \sa BEGIN_COMPLEX_DATA_IMPLEMENTATION
 */
#define END_COMPLEX_DATA_IMPLEMENTATION }

/*!
 *  \brief Begin declaration and implementation of a custom complex data parameter type.
 *
 *  This macro inserts the code necessary to declare a rec::rpc::serialization::Complex subclass with its constructor into a header file.
 *  Use this macro if you want the serializazable declarations only in header files.
 *  Below this macro, the preprocessor macro ADD_MEMBER must be used to insert the members to the list of children to be serialized.
 *  Then END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR must be inserted to begin the declaration of the children.
 *
 *  \param TYPENAME Name of the subclass.
 *  \param VERSION Version string. If data that has been serialized with a type version different from this, it can't be deserialized with this version.
 *
 *  \sa END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION, END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
 *  \sa DECLARE_CUSTOM_MEMBER, DECLARE_PRIMITIVE_MEMBER, DECLARE_STRING_MEMBER, DECLARE_BYTEARRAY_MEMBER, DECLARE_IMAGE_MEMBER
 *  \sa DECLARE_Position_MEMBER, DECLARE_Orientation_MEMBER, DECLARE_POSE_MEMBER, DECLARE_POSEARRAY_MEMBER, DECLARE_POSE2D_MEMBER, DECLARE_POSE2DARRAY_MEMBER
 *  \sa USE_COMPLEX_DATA_AS_PARAM, USE_COMPLEX_DATA_AS_RESULT, USE_COMPLEX_DATA_AS_TOPICDATA
 */
#define BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( TYPENAME, VERSION ) \
	class TYPENAME; \
	typedef QSharedPointer< TYPENAME > TYPENAME##Ptr; \
	typedef QSharedPointer< const TYPENAME > TYPENAME##PtrConst; \
	class TYPENAME : public rec::rpc::serialization::Complex \
	{ \
	public: \
		TYPENAME() \
		: rec::rpc::serialization::Complex( #TYPENAME "_" #VERSION ) \
		{

/*!
 *  \brief End constructor and begin member declaration of a custom complex data type.
 *
 *  This macro closes the constructor and allows you to declare the children.
 *  To declare the children, use the appropriate preprocessor macros. Finally insert END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION.
 *
 *  \sa BEGIN_COMPLEX_DATA_DECLARATION, BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION, END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
 */
#define END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR }

/*!
 *  \brief End implementation and declaration of a custom complex data type.
 *
 *  \sa BEGIN_COMPLEX_DATA_IMPLEMENTATION_AND_DECLARATION
 */
#define END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION };

/*!
 *  \brief Reuse an existing data type definition.
 *
 *  \param EXISTINGTYPENAME Data type that shall be reused.
 *  \param NEWTYPENAME New data type name.
 *
 *  \sa BEGIN_COMPLEX_DATA_DECLARATION, END_COMPLEX_DATA_DECLARATION
 */
#define USE_COMPLEX_DATA( EXISTINGTYPENAME, NEWTYPENAME ) \
	typedef EXISTINGTYPENAME NEWTYPENAME; \
	typedef QSharedPointer< NEWTYPENAME > NEWTYPENAME##Ptr; \
	typedef QSharedPointer< const NEWTYPENAME > NEWTYPENAME##PtrConst;

/*!
 *  \brief Use an existing complex type definition for a function which uses this type as parameter type.
 *
 *  \param TYPENAME Name of the data type that shall be used.
 *  \param FUNCTIONNAME Name of the function which uses the type as parameter type.
 *
 *  \sa BEGIN_COMPLEX_DATA_DECLARATION, END_COMPLEX_DATA_DECLARATION, USE_COMPLEX_DATA
 */
#define USE_COMPLEX_DATA_AS_PARAM( TYPENAME, FUNCTIONNAME ) USE_COMPLEX_DATA( TYPENAME, FUNCTIONNAME##Param )

/*!
 *  \brief Use an existing complex type definition for a function which uses the type as result type.
 *
 *  \param TYPENAME Name of the data type that shall be used.
 *  \param FUNCTIONNAME Name of the function which uses the type as parameter type.
 *
 *  \sa BEGIN_COMPLEX_DATA_DECLARATION, END_COMPLEX_DATA_DECLARATION, USE_COMPLEX_DATA
 */
#define USE_COMPLEX_DATA_AS_RESULT( TYPENAME, FUNCTIONNAME ) USE_COMPLEX_DATA( TYPENAME, FUNCTIONNAME##Result )

/*!
 *  \brief Use an existing complex data type definition for a topic which uses this data type.
 *
 *  \param TYPENAME Name of the data type that shall be used.
 *  \param TOPICNAME Name of the topic which uses the type.
 *
 *  \sa BEGIN_COMPLEX_DATA_DECLARATION, END_COMPLEX_DATA_DECLARATION, USE_COMPLEX_DATA
 */
#define USE_COMPLEX_DATA_AS_TOPICDATA( TYPENAME, TOPICNAME ) USE_COMPLEX_DATA( TYPENAME, topic##TOPICNAME##Data )

/*!
 *  \brief Complex data container member definition for any rec::rpc::serialization::Serializable subclass.
 *
 *  Place this macro into the declaration of a rec::rpc::serialization::Complex subclass to add a custom member.
 *  It can be accessed via NAME() (const and non-const).
 *
 *  \param DATATYPE Type name of the rec::rpc::serialization::Serializable subclass.
 *  \param NAME Member name.
 *
 *  \sa rec::rpc::serialization::Complex
 */
#define DECLARE_CUSTOM_MEMBER( DATATYPE, NAME ) \
	private: \
		QSharedPointer< DATATYPE > _##NAME; \
		void create_##NAME() { _##NAME = QSharedPointer< DATATYPE >( new DATATYPE ); } \
	public: \
		DATATYPE& NAME() { return *_##NAME; } \
		const DATATYPE& NAME() const { return *_##NAME; } \
		void set##NAME( const DATATYPE& pos ) { *_##NAME = pos; }

/*!
 *  Add serializable to the list of children. This must be done in the constructor.
 *
 *  \param NAME Name of the child.
 *
 *  \sa rec::rpc::serialization::Complex::addChild()
 */
#define ADD_MEMBER( NAME ) \
	create_##NAME(); \
	addChild( _##NAME.data() );

#endif //_REC_RPC_SERIALIZATION_COMPLEX_H_
