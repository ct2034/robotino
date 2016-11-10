/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "Tutorial1.h"

/******************************************************************************/
/***  Implementation of class Counter                                       ***/
/******************************************************************************/
Tutorial1::Tutorial1 (plugin::simulation::UnitDelegate& del)
: plugin::simulation::Unit(del, plugin::simulation::Deterministic)
{
}

void Tutorial1::step (void)
{
	float in = readInput( "in" ).toFloat();

	//do your processing here
	float out = 2.0f * in;

	writeOutput( "out", out );
}
