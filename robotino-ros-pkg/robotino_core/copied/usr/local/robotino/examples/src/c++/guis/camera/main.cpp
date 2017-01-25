#include "MainWindow.h"
#include "MyCom.h"


#if defined(Q_OS_IOS) 
extern "C" int qtmn(int argc, char **argv)
#else 
int main(int argc, char **argv)
#endif
{
	QApplication app( argc, argv );
	QCoreApplication::setOrganizationName("REC GmbH");
	QCoreApplication::setOrganizationDomain("servicerobotics.eu");
	QCoreApplication::setApplicationName("camera_gui");
	
	MyCom::init();

	MainWindow mw;
	mw.show();

	MyCom::done();

	return app.exec();
}
