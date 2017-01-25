/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#ifndef _Tutorial1_H_
#define _Tutorial1_H_

#include "plugin/simulation/Unit.h"

/******************************************************************************/
/***   Declaration of class Segmenter                                         ***/
/******************************************************************************/
class Tutorial1 : public plugin::simulation::Unit
{
public:
	// Constructors / Destructors //
	Tutorial1 (plugin::simulation::UnitDelegate& del);

	void step (void);

private:
};

#endif // #ifndef _Tutorial1_H_
