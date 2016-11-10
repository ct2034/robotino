/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "Tutorial2.h"

/******************************************************************************/
/***  Implementation of class Counter                                       ***/
/******************************************************************************/
Tutorial2::Tutorial2 (plugin::simulation::UnitDelegate& del)
: plugin::simulation::Unit(del, plugin::simulation::Deterministic)
{
}

void Tutorial2::step (void)
{
	float in = readInput( "in" ).toFloat();

	int n = readInput( "n" ).toInt();

	//do your processing here
	float out = static_cast<float>( n ) * in;

	writeOutput( "out", out );
}
