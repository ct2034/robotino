#ifndef _REC_QT_LOG_VIEW_H_
#define _REC_QT_LOG_VIEW_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif


class LogView : public QTextEdit
{
	Q_OBJECT
public:
	LogView( QWidget* parent = NULL );

private Q_SLOTS:
	void on_textChanged();

private:
};

#endif //_REC_QT_LOG_VIEW_H_

