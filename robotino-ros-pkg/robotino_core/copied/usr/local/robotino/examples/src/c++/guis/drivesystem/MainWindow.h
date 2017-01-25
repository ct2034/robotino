//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#ifndef _MAINWINOW_H_
#define _MAINWINOW_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif


class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
	MainWindow();

private:
};

#endif //_MAINWINOW_H_
