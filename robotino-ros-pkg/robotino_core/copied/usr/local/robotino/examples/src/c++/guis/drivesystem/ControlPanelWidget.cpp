//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#include "ControlPanelWidget.h"
#include "MyCom.h"

ControlPanelWidget::ControlPanelWidget( QWidget* parent )
	: QWidget( parent )
	, _vx( 0 )
	, _vy( 0 )
	, _omega( 0 )
{
	QGridLayout* layout = new QGridLayout( this );
	setLayout( layout );

	_ccl = new QPushButton( this );
	_ccl->setIcon( QPixmap( ":/icons/ccl.png" ) );

	_cl = new QPushButton( this );
	_cl->setIcon( QPixmap( ":/icons/cl.png" ) );

	_n = new QPushButton( this );
	_n->setIcon( QPixmap( ":/icons/n.png" ) );

	_s = new QPushButton( this );
	_s->setIcon( QPixmap( ":/icons/s.png" ) );

	_w = new QPushButton( this );
	_w->setIcon( QPixmap( ":/icons/w.png" ) );

	_o = new QPushButton( this );
	_o->setIcon( QPixmap( ":/icons/o.png" ) );

	_nw = new QPushButton( this );
	_nw->setIcon( QPixmap( ":/icons/nw.png" ) );

	_no = new QPushButton( this );
	_no->setIcon( QPixmap( ":/icons/no.png" ) );

	_sw = new QPushButton( this );
	_sw->setIcon( QPixmap( ":/icons/sw.png" ) );

	_so = new QPushButton( this );
	_so->setIcon( QPixmap( ":/icons/so.png" ) );

	_stop = new QPushButton( this );
	_stop->setIcon( QPixmap( ":/icons/stop.png" ) );

	QList< QPushButton* > buttonList;
	buttonList << _ccl << _cl << _n << _s << _o << _w << _nw << _no << _sw << _so << _stop;
	Q_FOREACH( QPushButton* p, buttonList )
	{
		p->setIconSize( QSize( 40, 40 ) );
		p->setFixedSize( QSize( 44, 44 ) );
		p->setFocusPolicy( Qt::NoFocus );
	}

	_speed = new QSlider( Qt::Horizontal, this );
	_speed->setRange( 1, 100 );
	_speed->setValue( 20 );
	_speed->setFocusPolicy( Qt::NoFocus );

	layout->addWidget( _ccl, 1, 0 );
	layout->addWidget( _cl, 1, 4 );

	layout->addWidget( _n, 0, 2 );
	layout->addWidget( _stop, 1, 2 );
	layout->addWidget( _s, 2, 2 );

	layout->addWidget( _w, 1, 1 );
	layout->addWidget( _o, 1, 3 );

	layout->addWidget( _nw, 0, 1 );
	layout->addWidget( _no, 0, 3 );

	layout->addWidget( _sw, 2, 1 );
	layout->addWidget( _so, 2, 3 );

	layout->addWidget( _speed, 3, 0, 1, 5 );

	layout->setColumnStretch( 5, 1 );

	bool connected = connect( _ccl, SIGNAL( clicked() ), this, SLOT( on_clicked() ) );
	Q_ASSERT( connected );

	connected = connect( _cl, SIGNAL( clicked() ), this, SLOT( on_clicked() ) );
	Q_ASSERT( connected );

	connected = connect( _n, SIGNAL( clicked() ), this, SLOT( on_clicked() ) );
	Q_ASSERT( connected );

	connected = connect( _s, SIGNAL( clicked() ), this, SLOT( on_clicked() ) );
	Q_ASSERT( connected );

	connected = connect( _w, SIGNAL( clicked() ), this, SLOT( on_clicked() ) );
	Q_ASSERT( connected );

	connected = connect( _o, SIGNAL( clicked() ), this, SLOT( on_clicked() ) );
	Q_ASSERT( connected );

	connected = connect( _nw, SIGNAL( clicked() ), this, SLOT( on_clicked() ) );
	Q_ASSERT( connected );

	connected = connect( _no, SIGNAL( clicked() ), this, SLOT( on_clicked() ) );
	Q_ASSERT( connected );

	connected = connect( _sw, SIGNAL( clicked() ), this, SLOT( on_clicked() ) );
	Q_ASSERT( connected );

	connected = connect( _so, SIGNAL( clicked() ), this, SLOT( on_clicked() ) );
	Q_ASSERT( connected );

	connected = connect( _stop, SIGNAL( clicked() ), this, SLOT( on_clicked() ) );
	Q_ASSERT( connected );

	connected = connect( _speed, SIGNAL( valueChanged(int) ), this, SLOT( on_speed_valueChanged(int) ) );
	Q_ASSERT( connected );

	QTimer* timer = new QTimer( this );
	timer->setSingleShot( false );
	timer->setInterval( 100 );

	connected = connect( timer, SIGNAL( timeout() ), SLOT( setVelocity() ) );
	Q_ASSERT( connected );

	timer->start();
}

void ControlPanelWidget::on_clicked()
{
	const QObject* obj = sender();
	if( obj == _ccl )
	{
		++_omega;
	}
	else if( obj == _cl )
	{
		--_omega;
	}
	else if( obj == _n )
	{
		++_vx;
	}
	else if( obj == _s )
	{
		--_vx;
	}
	else if( obj == _w )
	{
		++_vy;
	}
	else if( obj == _o )
	{
		--_vy;
	}
	else if( obj == _nw )
	{
		++_vx;
		++_vy;
	}
	else if( obj == _no )
	{
		++_vx;
		--_vy;
	}
	else if( obj == _sw )
	{
		--_vx;
		++_vy;
	}
	else if( obj == _so )
	{
		--_vx;
		--_vy;
	}
	else if( obj == _stop )
	{
		_vx = 0;
		_vy = 0;
		_omega = 0;
	}

	setVelocity();
}

void ControlPanelWidget::on_speed_valueChanged( int )
{
	setVelocity();
}

void ControlPanelWidget::setVelocity()
{
	float scale = 0.002f * _speed->value();
	_omnidrive.setVelocity( scale * _vx, scale * _vy, 5.0f * scale * _omega );
}
