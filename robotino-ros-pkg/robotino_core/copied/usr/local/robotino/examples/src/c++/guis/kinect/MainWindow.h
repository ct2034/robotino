//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#ifndef _MAINWINOW_H_
#define _MAINWINOW_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif


class MyKinect;
class TabView;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
	MainWindow();

private:
	MyKinect* _kinect;
	TabView* _tabView;
};

#endif //_MAINWINOW_H_
