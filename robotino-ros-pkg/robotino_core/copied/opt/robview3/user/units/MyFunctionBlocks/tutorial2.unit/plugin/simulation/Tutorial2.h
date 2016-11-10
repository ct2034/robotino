/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#ifndef _Tutorial2_H_
#define _Tutorial2_H_

#include "plugin/simulation/Unit.h"

/******************************************************************************/
/***   Declaration of class Segmenter                                         ***/
/******************************************************************************/
class Tutorial2 : public plugin::simulation::Unit
{
public:
	// Constructors / Destructors //
	Tutorial2 (plugin::simulation::UnitDelegate& del);

	void step (void);

private:
};

#endif // #ifndef _Tutorial1_H_
