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

#ifndef _REC_RPC_CLIENTINFO_H_
#define _REC_RPC_CLIENTINFO_H_

#include <QObject>
#include <QHostAddress>
#include <QString>
#include <QDataStream>
#include <QMetaType>
#include <QDebug>

namespace rec
{
	namespace rpc
	{
		/*!
		 *  \brief RPC client info
		 *
		 *  Class that stores information about RPC clients.
		 */
		class ClientInfo : public QObject
		{
		public:
			/*! \brief Default constructor */
			ClientInfo()
				: port( -1 )
			{
			}

			/*!
			 *  \brief Constructor with initializations
			 *
			 *  \param address_ Client's network address
			 *  \param port_ Client's TCP port
			 *  \param name_ Name (can be empty)
			 */
			ClientInfo( const QHostAddress& address_, const int port_, const QString& name_ = QString::null )
				: address( address_ )
				, port( port_)
				, name( name_ )
			{
			}

			/*!
			 *  \brief Copy constructor
			 *
			 *  \param other ClientInfo to copy
			 */
			ClientInfo( const ClientInfo& other )
				: address( other.address )
				, port( other.port )
				, name( other.name )
			{
			}

			/*!
			 *  \brief Assignment operator
			 *
			 *  \param other ClientInfo to copy
			 */
			ClientInfo& operator=( const ClientInfo& other )
			{
				address = other.address;
				port = other.port;
				name = other.name;
				return *this;
			}

			/*!
			 *  \brief Null check
			 *
			 *  \return True if the port number is -1
			 */
			bool isNull() const
			{
				return -1 == port;
			}

			/*!
			 *  \brief Check if client it connected via local socket
			 *
			 *  \return True if the client is local
			 */
			bool isLocal() const
			{
				return ( -1 != port ) && address.isNull();
			}

			/*! \brief Clear all data */
			void clear()
			{
				address = QHostAddress::Null;
				port = -1;
				name.clear();
			}

			/*!
			 *  \brief Equality test
			 *
			 *  \param other Other ClientInfo to compare with
			 *  \return true if *this == other
			 */
			bool operator==( const ClientInfo& other ) const
			{
				if( other.address == address
					&& other.port == port )
				{
					return true;
				}
				return false;
			}

			/*!
			 *  \brief Unequality test
			 *
			 *  \param other Other ClientInfo to compare with
			 *  \return true if *this != other
			 */
			bool operator!=( const ClientInfo& other ) const
			{
				if( other.address != address
					|| other.port != port )
				{
					return true;
				}
				return false;
			}

			QString toString() const
			{
				QString str = name;
				str += " " + address.toString() + ":" + QString::number( port );
				return str;
			}

			/*! \brief Client's network address */
			QHostAddress address;
			/*! \brief Client's TCP port */
			int port;
			/*! \brief Name (can be empty) */
			QString name;
		};

		/*! \brief Set of ClientInfos typedef */
		typedef QSet< ClientInfo > ClientInfoSet;
		/*! \brief Map of ClientInfos typedef */
		typedef QMap< QString, ClientInfo > ClientInfoMap;
	}
}

/* \cond */
QT_BEGIN_NAMESPACE
static QDataStream& operator<<(QDataStream& s, const rec::rpc::ClientInfo& info )
{
	//s << info.address << info.port << info.name;
	s << info.address << info.port;
	return s;
}

static QDataStream& operator>>(QDataStream& s, rec::rpc::ClientInfo& info )
{
	if ( ( s >> info.address ).status() != QDataStream::Ok )
		return s;
	if ( ( s >> info.port ).status() != QDataStream::Ok )
		return s;
	//if ( ( s >> info.name ).status() != QDataStream::Ok )
	//	return s;
	return s;
}

static QDebug operator<<( QDebug dbg, const rec::rpc::ClientInfo& p )
{
	dbg.nospace() << "(" << p.address.toString() << ", " << p.port << ", " << p.name << ", " << ( p.isLocal() ? "local" : "remote" ) << ")";

	return dbg.space();
}

static bool operator<(const rec::rpc::ClientInfo &i1, const rec::rpc::ClientInfo &i2)
{
	if( i1.address != i2.address )
	{
		if ( i1.address.protocol() != i2.address.protocol() )
			return ( i1.address.protocol() < i2.address.protocol() );
		if ( i1.address.protocol() == QAbstractSocket::IPv4Protocol )
		{
			quint32 a1 = i1.address.toIPv4Address();
			quint32 a2 = i2.address.toIPv4Address();
			if ( a1 != a2 )
				return a1 < a2;
		}
		else if ( i1.address.protocol() == QAbstractSocket::IPv6Protocol )
		{
			QIPv6Address a1 = i1.address.toIPv6Address();
			QIPv6Address a2 = i2.address.toIPv6Address();
			for( unsigned int i = 0; i < 16; ++i )
			{
				if ( a1[i] != a2[i] )
					return a1[i] < a2[i];
			}
		}
	}
	
	if( i1.port != i2.port )
	{
		return i1.port < i2.port;
	}
	//else
	//{
	//	return i1.name < i2.name;
	//}
}

inline uint qHash( const rec::rpc::ClientInfo& i )
{
	return qHash( qMakePair( i.address, i.port ) );
}

QT_END_NAMESPACE

Q_DECLARE_METATYPE( rec::rpc::ClientInfo )
Q_DECLARE_METATYPE( rec::rpc::ClientInfoMap )
/* \endcond */

#endif //_REC_RPC_CLIENTINFO_H_
