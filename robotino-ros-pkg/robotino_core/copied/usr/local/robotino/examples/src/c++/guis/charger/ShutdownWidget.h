#ifndef _ShutdownWidget_H_
#define _ShutdownWidget_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif

#include <rec/robotino/api2/all.h>

class ShutdownWidget : public QToolBar, rec::robotino::api2::Shutdown
{
	Q_OBJECT
public:
	ShutdownWidget();

private Q_SLOTS:
	void on_shutdown_clicked();
};

#endif
