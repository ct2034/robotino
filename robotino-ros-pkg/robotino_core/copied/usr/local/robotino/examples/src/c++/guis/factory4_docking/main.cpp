#include "MainWindow.h"
#include "Log.h"
#include "MyCom.h"

int main(int argc, char **argv)
{
	QApplication app( argc, argv );
	QCoreApplication::setOrganizationName("REC GmbH");
	QCoreApplication::setOrganizationDomain("servicerobotics.eu");
	QCoreApplication::setApplicationName("factory4 laser docking");

	Log::init();
	MyCom::init();

	MainWindow mw;
	mw.show();

	int r = app.exec();

	MyCom::done();
	Log::done();

	return r;
}
