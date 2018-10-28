#include <uStepperS.h>

uStepperDriver::uStepperDriver( void ){
}

void uStepperDriver::init( uStepperS * _pointer ){

	this->pointer = _pointer;
	this->chipSelect(true); // Set CS HIGH


	/* Prepare general driver settings */

	/* Set motor current */
	this->writeRegister( IHOLD_IRUN, IHOLD( this->holdCurrent) | IRUN( this->current) | IHOLDDELAY(5));

	this->enableStealth( 100000 );

	/* Set all-round chopper configuration */
	this->writeRegister( CHOPCONF, TOFF(4) | TBL(2) | HSTRT_TFD(4) | HEND(0) );

	/* Set startup ramp mode */
	this->setRampMode( VELOCITY_MODE_POS );

	/* Reset position */
	this->writeRegister(XACTUAL, 0);
	this->writeRegister(XTARGET, 0);

}


void uStepperDriver::enableStealth( uint32_t threshold ){

	/* Set GCONF and enable stealthChop */
	this->writeRegister( GCONF, EN_PWM_MODE(1) | I_SCALE_ANALOG(1) | DIRECTION(0) ); 

	/* Set PWMCONF for StealthChop */
	this->writeRegister( PWMCONF, PWM_GRAD(1) | PWM_AMPL(255) | PWM_FREQ(0) ); 

	/* Specifies the upper velocity for operation in stealthChop voltage PWM mode */
	this->writeRegister( TPWMTHRS, threshold ); 

}


void uStepperDriver::setRampMode( uint8_t mode ){

	switch(mode){
		case POSITIONING_MODE:
			// Positioning mode
			this->writeRegister(VSTART, 0 );
			this->writeRegister(A1, 	1000); 						/* A1 = 1000 */
			this->writeRegister(V1, 	100000); 					/* V1 = 100000 usteps / t  */
			this->writeRegister(AMAX, 	pointer->acceleration); 	/* AMAX */
			this->writeRegister(VMAX, 	pointer->velocity); 		/* VMAX */
			this->writeRegister(D1, 	1400); 						/* D1 = 1400 */
			this->writeRegister(VSTOP, 	100 ); 						/* VSTOP = 10 */

			this->writeRegister(RAMPMODE, POSITIONING_MODE); 		/* RAMPMODE = 0 = Positioning mode */
		
		break;

		case VELOCITY_MODE_POS:
			// Velocity mode (only AMAX and VMAX is used)
			this->writeRegister(A1, 	0); 						
			this->writeRegister(V1, 	0); 
			this->writeRegister(D1, 	0); 
			this->writeRegister(AMAX, 	pointer->acceleration); 	/* AMAX */
			this->writeRegister(VMAX, 	pointer->velocity); 		/* VMAX */

			this->writeRegister(RAMPMODE, VELOCITY_MODE_POS); 		/* RAMPMODE = 1 = Velocity mode */

		break;
	}
}


int32_t uStepperDriver::writeRegister( uint8_t address, uint32_t datagram ){

	// Disabled interrupts until write is complete
	cli();

	// Enable SPI mode 3 to use TMC5130
	this->pointer->setSPIMode(3);

	uint32_t package;

	// Add the value of WRITE_ACCESS to enable register write
	address += WRITE_ACCESS;

	this->chipSelect(false);

	this->status = this->pointer->SPI(address);

	package |= this->pointer->SPI((datagram >> 24) & 0xff);
	package <<= 8;
	package |= this->pointer->SPI((datagram >> 16) & 0xff);
	package <<= 8;
	package |= this->pointer->SPI((datagram >> 8) & 0xff);
	package <<= 8;
	package |= this->pointer->SPI((datagram) & 0xff);

	this->chipSelect(true); // Set CS HIGH

	sei(); 

	return package;
}


int32_t uStepperDriver::readRegister( uint8_t address ){

	// Disabled interrupts until write is complete
	cli();

	// Enable SPI mode 3 to use TMC5130
	this->pointer->setSPIMode(3);

	// Request a reading on address
	this->chipSelect(false);
	this->status = this->pointer->SPI(address);
	this->pointer->SPI(0x00);
	this->pointer->SPI(0x00);
	this->pointer->SPI(0x00);
	this->pointer->SPI(0x00);
	this->chipSelect(true);


	// Read the actual value on second request
	int32_t value = 0;

	this->chipSelect(false);
	this->status = this->pointer->SPI(address);
	value |= this->pointer->SPI(0x00);
	value <<= 8;
	value |= this->pointer->SPI(0x00);
	value <<= 8;
	value |= this->pointer->SPI(0x00);
	value <<= 8;
	value |= this->pointer->SPI(0x00);
	this->chipSelect(true);

	sei(); 

	return value;

}


void uStepperDriver::setSpeed( int32_t velocity ){

	this->writeRegister(VMAX, velocity);

}

int32_t uStepperDriver::getSpeed( void ){

	return this->readRegister(VACTUAL);

}

void uStepperDriver::setPosition( int32_t position ){

	this->setRampMode(POSITIONING_MODE);
	this->writeRegister(XTARGET, position);

}

int32_t uStepperDriver::getPosition( void ){

	return this->readRegister(XACTUAL);

}

void uStepperDriver::chipSelect(bool state){

	if(state == false)
		PORTE &= ~(1 << CS_DRIVER);  // Set CS LOW 
	else
		PORTE |= (1 << CS_DRIVER); // Set CS HIGH

	if( state )
		delayMicroseconds(100);   // per spec, settling time is 100us
}