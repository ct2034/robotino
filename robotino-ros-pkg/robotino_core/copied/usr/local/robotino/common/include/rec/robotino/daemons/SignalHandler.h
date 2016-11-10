#ifndef _REC_ROBOTINO_DAEMONS_SIGNALHANDLER_H_
#define _REC_ROBOTINO_DAEMONS_SIGNALHANDLER_H_

#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#endif

#include <QtCore>

namespace rec
{
	namespace robotino
	{
		namespace daemons
		{
#ifdef WIN32
			class SignalHandler : public QThread
			{
				Q_OBJECT
			public:
				SignalHandler()
				{
					Q_ASSERT( NULL == instance );
					instance = this;

					BOOL ok = SetConsoleCtrlHandler( (PHANDLER_ROUTINE)ConsoleHandler, TRUE );
					Q_ASSERT( (BOOL)TRUE == ok );

					start();
				}

				~SignalHandler()
				{
					exit( 0 );
					wait();

					instance = NULL;
				}

				void setDefaultConnectionsEnabled( bool enable )
				{
					bool connected = true;
					connected &= (bool)connect( this, SIGNAL( sigHup() ), qApp, SLOT( quit() ) );
					connected &= (bool)connect(this, SIGNAL(sigTerm()), qApp, SLOT(quit()));
					connected &= (bool)connect(this, SIGNAL(sigInt()), qApp, SLOT(quit()));
					Q_ASSERT( connected );
				}

			private:
				static BOOL WINAPI ConsoleHandler(DWORD CEvent)
				{
					switch(CEvent)
					{
					case CTRL_C_EVENT:
					case CTRL_BREAK_EVENT:
					case CTRL_CLOSE_EVENT:
					case CTRL_LOGOFF_EVENT:
					case CTRL_SHUTDOWN_EVENT:
						instance->exit( (int)CEvent );
						break;

					}
					return TRUE;
				}

				void run()
				{
					int ret = exec();

					switch( ret )
					{
					case CTRL_C_EVENT:
						Q_EMIT log( "CTRL_C_EVENT", 1 );
						Q_EMIT sigTerm();
						break;
					case CTRL_BREAK_EVENT:
						Q_EMIT log( "CTRL_BREAK_EVENT", 1 );
						Q_EMIT sigTerm();
						break;
					case CTRL_CLOSE_EVENT:
						Q_EMIT log( "CTRL_CLOSE_EVENT", 1 );
						Q_EMIT sigTerm();
						break;
					case CTRL_LOGOFF_EVENT:
						Q_EMIT log( "CTRL_LOGOFF_EVENT", 1 );
						Q_EMIT sigTerm();
						break;
					case CTRL_SHUTDOWN_EVENT:
						Q_EMIT log( "CTRL_SHUTDOWN_EVENT", 1 );
						Q_EMIT sigTerm();
						break;
					}
				}

				static SignalHandler* instance;
#else
			class SignalHandler : public QObject
			{
				Q_OBJECT
			public:
				SignalHandler()
				{
					struct sigaction hup, term, intact;

					hup.sa_handler = hupSignalHandler;
					sigemptyset(&hup.sa_mask);
					hup.sa_flags = 0;
					hup.sa_flags |= SA_RESTART;

					int r = sigaction(SIGHUP, &hup, 0);
					Q_ASSERT( 0 == r );

					term.sa_handler = termSignalHandler;
					sigemptyset(&term.sa_mask);
					term.sa_flags |= SA_RESTART;

					r = sigaction(SIGTERM, &term, 0);
					Q_ASSERT( 0 == r );

					intact.sa_handler = intSignalHandler;
					sigemptyset(&intact.sa_mask);
					intact.sa_flags |= SA_RESTART;

					r = sigaction(SIGINT, &intact, 0);
					Q_ASSERT( 0 == r );

					if (::socketpair(AF_UNIX, SOCK_STREAM, 0, sighupFd))
					{
						qFatal("Couldn't create HUP socketpair");
						Q_ASSERT( false );
					}

					if (::socketpair(AF_UNIX, SOCK_STREAM, 0, sigtermFd))
					{
						qFatal("Couldn't create TERM socketpair");
						Q_ASSERT( false );
					}

					if (::socketpair(AF_UNIX, SOCK_STREAM, 0, sigintFd))
					{
						qFatal("Couldn't create INT socketpair");
						Q_ASSERT( false );
					}

					snHup = new QSocketNotifier(sighupFd[1], QSocketNotifier::Read, this);
					bool connected = connect( snHup, SIGNAL( activated(int) ), this, SLOT(handleSigHup()));
					Q_ASSERT( connected );

					snTerm = new QSocketNotifier(sigtermFd[1], QSocketNotifier::Read, this);
					connected = connect(snTerm, SIGNAL(activated(int)), this, SLOT(handleSigTerm()));
					Q_ASSERT( connected );

					snInt = new QSocketNotifier(sigintFd[1], QSocketNotifier::Read, this);
					connected = connect(snInt, SIGNAL(activated(int)), this, SLOT(handleSigInt()));
					Q_ASSERT( connected );
				}

				void setDefaultConnectionsEnabled( bool enable )
				{
					bool connected = true;
					connected &= (bool)connect( this, SIGNAL( sigHup() ), qApp, SLOT( quit() ) );
					connected &= (bool)connect( this, SIGNAL( sigTerm() ), qApp, SLOT( quit() ) );
					connected &= (bool)connect( this, SIGNAL( sigInt() ), qApp, SLOT( quit() ) );
					Q_ASSERT( connected );
				}

			private:
				// Unix signal handlers.
				static void hupSignalHandler(int unused)
				{
					char a = 1;
					int ret = ::write(sighupFd[0], &a, sizeof(a));
				}

				static void termSignalHandler(int unused)
				{
					char a = 1;
					int ret = ::write(sigtermFd[0], &a, sizeof(a));
				}

				static void intSignalHandler(int unused)
				{
					char a = 1;
					int ret = ::write(sigintFd[0], &a, sizeof(a));
				}

				static int sighupFd[2];
				static int sigtermFd[2];
				static int sigintFd[2];

				QSocketNotifier *snHup;
				QSocketNotifier *snTerm;
				QSocketNotifier *snInt;
#endif

			Q_SIGNALS:
				void sigHup();
				void sigTerm();
				void sigInt();
				void log( const QString&, int );

			private Q_SLOTS:
				void handleSigHup()
				{
#ifndef WIN32
					bool connected = disconnect( snHup, SIGNAL( activated(int) ), this, SLOT(handleSigHup()));
					Q_ASSERT( connected );

					Q_EMIT log( "SIGHUP received", 0 );
					Q_EMIT sigHup();
#endif
				}

				void handleSigTerm()
				{
#ifndef WIN32
					bool connected = disconnect(snTerm, SIGNAL(activated(int)), this, SLOT(handleSigTerm()));
					Q_ASSERT( connected );

					Q_EMIT log( "SIGTERM received", 0 );
					Q_EMIT sigTerm();
#endif
				}

				void handleSigInt()
				{
#ifndef WIN32
					bool connected = disconnect(snInt, SIGNAL(activated(int)), this, SLOT(handleSigInt()));
					Q_ASSERT( connected );

					Q_EMIT log( "SIGINT received", 0 );
					Q_EMIT sigInt();
#endif
				}
			};
		}
	}
}

#endif //_REC_ROBOTINO_DAEMONS_SIGNALHANDLER_H_
