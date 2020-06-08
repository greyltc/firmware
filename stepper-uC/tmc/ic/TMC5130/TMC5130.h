/*
 * TMC5130.h
 *
 *  Created on: 03.07.2017
 *      Author: LK
 */

#ifndef TMC_IC_TMC5130_H_
#define TMC_IC_TMC5130_H_

#include "../../helpers/Constants.h"
#include "../../helpers/API_Header.h"
#include "TMC5130_Register.h"
#include "TMC5130_Constants.h"
#include "TMC5130_Fields.h"
#include <avr/pgmspace.h>

// Helper macros
#define TMC5130_FIELD_READ(tdef, address, mask, shift) \
	FIELD_GET(tmc5130_readInt(tdef, address), mask, shift)
#define TMC5130_FIELD_WRITE(tdef, address, mask, shift, value) \
	(tmc5130_writeInt(tdef, address, FIELD_SET(tmc5130_readInt(tdef, address), mask, shift, value)))


#endif /* TMC_IC_TMC5130_H_ */
