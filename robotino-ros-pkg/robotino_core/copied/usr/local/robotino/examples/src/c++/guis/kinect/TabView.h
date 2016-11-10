#ifndef _TABVIEW_H_
#define _TABVIEW_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif


class MyKinect;

class TabView : public QTabWidget
{
public:
	TabView( QWidget* parent, MyKinect* kinect );
};

#endif //_TABVIEW_H_
