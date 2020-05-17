/********************************************************************************************
* 	 	File: 		uStepperS.cpp															*
*		Version:    1.0.1                                           						*
*      	Date: 		May 14th, 2019  	                                    				*
*      	Author: 	Thomas Hørring Olsen                                   					*
*                                                   										*	
*********************************************************************************************
*	(C) 2019																				*
*																							*
*	uStepper ApS																			*
*	www.ustepper.com 																		*
*	administration@ustepper.com 															*
*																							*
*	The code contained in this file is released under the following open source license:	*
*																							*
*			Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International			*
* 																							*
* 	The code in this file is provided without warranty of any kind - use at own risk!		*
* 	neither uStepper ApS nor the author, can be held responsible for any damage				*
* 	caused by the use of the code contained in this file ! 									*
*                                                                                           *
********************************************************************************************/
/**
* @file uStepperS.cpp
*
* @brief      Function and class implementation for the uStepper S library
*
*             This file contains class and function implementations for the library.
*
* @author     Thomas Hørring Olsen (thomas@ustepper.com)
*/
#include <uStepperS.h>
uStepperS * pointer;

uStepperS::uStepperS()
{
	pointer = this;

	this->microSteps = 256;
	this->init();	

	this->setMaxAcceleration(2000.0);
	this->setMaxDeceleration(2000.0);
	this->setMaxVelocity(100.0);
}

uStepperS::uStepperS(float acceleration, float velocity)
{
	pointer = this;
	this->microSteps = 256;
	this->init();

	this->setMaxAcceleration(acceleration);
	this->setMaxVelocity(velocity);
}


void uStepperS::init( void ){

	
	this->pidDisabled = 1;
	/* Set CS, MOSI, SCK and DRV_ENN as Output */
	DDRC = (1<<SCK1)|(1<<MOSI_ENC);
	DDRD = (1<<DRV_ENN)|(1<<SD_MODE)|(1<<CS_ENCODER);
	DDRE = (1<<MOSI1)|(1<<CS_DRIVER);

	PORTD &= ~(1 << DRV_ENN);  // Set DRV_ENN LOW  
	PORTD &= ~(1 << SD_MODE);  // Set SD_MODE LOW  

	/* 
	*  ---- Global SPI1 configuration ----
	*  SPE   = 1: SPI enabled
	*  MSTR  = 1: Master
	*  SPR0  = 0 & SPR1 = 0: fOSC/4 = 4Mhz
	*/
	SPCR1 = (1<<SPE1)|(1<<MSTR1);	

	driver.init( this );
	//encoder.init( this );
}

bool uStepperS::getMotorState(uint8_t statusType)
{
	this->driver.readMotorStatus();
	if(this->driver.status & statusType)
	{
		return 0;
	}
	return 1;
}

void uStepperS::setup(	uint8_t mode, 
							uint16_t stepsPerRevolution,
							float pTerm, 
							float iTerm,
							float dTerm,
							uint16_t dropinStepSize,
							bool setHome,
							uint8_t invert,
							uint8_t runCurrent,
							uint8_t holdCurrent)
{

	this->pidDisabled = 1;
	// Should setup mode etc. later
	this->mode = mode;
	this->fullSteps = stepsPerRevolution;
	this->dropinStepSize = 256/dropinStepSize;
	this->angleToStep = (float)this->fullSteps * (float)this->microSteps / 360.0;
	this->rpmToVelocity = (float)(279620.267 * fullSteps * microSteps)/(CLOCKFREQ);
	this->stepsPerSecondToRPM = 60.0/(this->microSteps*this->fullSteps);
	this->RPMToStepsPerSecond = (this->microSteps*this->fullSteps)/60.0;
	this->init();

	this->setCurrent(0.0);
	this->setHoldCurrent(0.0);

	this->stop(HARD);

	while(this->driver.readRegister(VACTUAL) != 0);

	delay(500);

	this->setCurrent(40.0);
	this->setHoldCurrent(25.0);

	encoder.setHome();
}

void uStepperS::moveSteps( int32_t steps )
{
	this->driver.setDeceleration( (uint16_t)( this->maxDeceleration ) );
	this->driver.setAcceleration( (uint16_t)( this->maxAcceleration ) );
	this->driver.setVelocity( (uint32_t)( this->maxVelocity  ) );
	
	// Get current position
	int32_t current = this->driver.getPosition();

	// Set new position
	this->driver.setPosition( current + steps);
}



void uStepperS::moveAngle( float angle )
{
	int32_t steps;

	if(angle < 0.0)
	{
		steps = (int32_t)((angle * angleToStep) - 0.5);
		this->moveSteps( steps ); 
	}
	else
	{
		steps = (int32_t)((angle * angleToStep) + 0.5);
		this->moveSteps( steps );
	}
}


void uStepperS::moveToAngle( float angle )
{
	float diff;
	int32_t steps;

	diff = angle - this->angleMoved();
	steps = (int32_t)( (abs(diff) * angleToStep) + 0.5);

	if(diff < 0.0)
	{
		this->moveSteps( -steps );
	}
	else
	{
		this->moveSteps( steps );
	}
}

bool uStepperS::isStalled( float stallSensitivity )
{
	if(this->stallSensitivity > 1.0)
  	{
  		this->stallSensitivity = 1.0;
  	}
  	else if(this->stallSensitivity < 0.0)
  	{
  		this->stallSensitivity = 0.0;
  	}
  	else{
  		this->stallSensitivity = stallSensitivity;
  	}
  	
	return this->stall;
}

void uStepperS::brakeMotor( bool brake )
{

}

void uStepperS::setRPM( float rpm)
{
	int32_t velocityDir = rpmToVelocity * rpm;

	if(velocityDir > 0){
		driver.setDirection(1);
	}else{
		driver.setDirection(0);
	}

	// The velocity cannot be signed
	uint32_t velocity = abs(velocityDir);

	driver.setVelocity( (uint32_t)velocity );
}


void uStepperS::setSPIMode( uint8_t mode ){

	switch(mode){
		case 2:
			SPCR1 |= (1<<CPOL1);  // Set CPOL HIGH = 1
			SPCR1 &= ~(1<<CPHA1);  // Set CPHA LOW = 0
		break;

		case 3:
			SPCR1 |= (1<<CPOL1);  // Set CPOL HIGH = 1
			SPCR1 |= (1<<CPHA1);  // Set CPHA HIGH = 1
		break;
	}
}

uint8_t uStepperS::SPI(uint8_t data){

	SPDR1 = data;

	// Wait for transmission complete
	while(!( SPSR1 & (1 << SPIF1) ));    

	return SPDR1;

}

void uStepperS::setMaxVelocity( float velocity )
{
	velocity *= (float)this->microSteps;
	velocity = abs(velocity)*VELOCITYCONVERSION;

	this->maxVelocity = velocity;

	// Steps per second, has to be converted to microsteps
	this->driver.setVelocity( (uint32_t)( this->maxVelocity  ) );
}

void uStepperS::setMaxAcceleration( float acceleration )
{
	acceleration *= (float)this->microSteps;
	acceleration = abs(acceleration) * ACCELERATIONCONVERSION;

	this->maxAcceleration = acceleration;

	
	// Steps per second, has to be converted to microsteps
	this->driver.setAcceleration( (uint32_t)(this->maxAcceleration ) );
}

void uStepperS::setMaxDeceleration( float deceleration )
{
	deceleration *= (float)this->microSteps;
	deceleration = abs(deceleration) * ACCELERATIONCONVERSION;
	
	this->maxDeceleration = deceleration;
	
	// Steps per second, has to be converted to microsteps
	this->driver.setDeceleration( (uint32_t)(this->maxDeceleration ) );
}

void uStepperS::setCurrent( double current )
{
	if( current <= 100.0 && current >= 0.0){
		// The current needs to be in the range of 0-31
		this->driver.current = ceil(0.31 * current); 
	}else{
		// If value is out of range, set default
		this->driver.current = 16; 
	}

	driver.updateCurrent();
}

// input here given in percentage
// resulting in RMS current through the motor
// of input value /100 * 325*(2.24/2.5)/((100+20)*sqrt(2)) [A RMS]
// ~= current/100*1.716 [A RMS]
void uStepperS::setHoldCurrent( double current )
{
	// The current needs to be in the range of 0-31
	if( current <= 100.0 && current >= 0.0){
		// The current needs to be in the range of 0-31
		this->driver.holdCurrent = ceil(0.31 * current);
	}else{
		// If value is out of range, set default
		this->driver.holdCurrent = 16; 
	}

	driver.updateCurrent();
}

void uStepperS::runContinous( bool direction )
{
	this->driver.setDeceleration( (uint32_t)( this->maxDeceleration ) );
	this->driver.setAcceleration( (uint32_t)(this->maxAcceleration ) );
	this->driver.setVelocity( (uint32_t)( this->maxVelocity  ) );

	// Make sure we use velocity mode
	this->driver.setRampMode( VELOCITY_MODE_POS );

	// Set the direction
	this->driver.setDirection( direction );
}

float uStepperS::angleMoved ( void )
{
	return this->encoder.getAngleMoved();
}

void uStepperS::stop( bool mode){

	// Check which mode is used

	// if positioning mode  
		// Update XTARGET to current postion
	// else
		// Set VMAX = 0

	// Side 76 TMC5130

	if(mode == HARD)
	{
		this->driver.setDeceleration( 0xFFFE );
		this->driver.setAcceleration( 0xFFFE );
		this->setRPM(0);
		while(this->driver.readRegister(VACTUAL) != 0);
		this->driver.setDeceleration( (uint32_t)( this->maxDeceleration ) );
		this->driver.setAcceleration( (uint32_t)(this->maxAcceleration ) );
	}
	else
	{
		this->setRPM(0);
	}
}

void uStepperS::filterSpeedPos(posFilter_t *filter, int32_t steps)
{
	filter->posEst += filter->velEst * ENCODERINTPERIOD;
	filter->posError = (float)steps - filter->posEst;
	filter->velIntegrator += filter->posError * PULSEFILTERKI;
	filter->velEst = (filter->posError * PULSEFILTERKP) + filter->velIntegrator;
}

void interrupt1(void)
{
	if(PIND & 0x04)
	{
		PORTD |= (1 << 4);
	}
	else
	{
		PORTD &= ~(1 << 4);
	}
}

void interrupt0(void)
{
	if(PIND & 0x04)
	{
		PORTD |= (1 << 4);
	}
	else
	{
		PORTD &= ~(1 << 4);
	}
	if((PINB & (0x08)))			//CCW
	{
		if(!pointer->invertPidDropinDirection)
		{
			pointer->stepCnt-=pointer->dropinStepSize;				//DIR is set to CCW, therefore we subtract 1 step from step count (negative values = number of steps in CCW direction from initial postion)
		}
		else
		{
			pointer->stepCnt+=pointer->dropinStepSize;			//DIR is set to CW, therefore we add 1 step to step count (positive values = number of steps in CW direction from initial postion)	
		}
		
	}
	else						//CW
	{
		if(!pointer->invertPidDropinDirection)
		{
			pointer->stepCnt+=pointer->dropinStepSize;			//DIR is set to CW, therefore we add 1 step to step count (positive values = number of steps in CW direction from initial postion)	
		}
		else
		{
			pointer->stepCnt-=pointer->dropinStepSize;				//DIR is set to CCW, therefore we subtract 1 step from step count (negative values = number of steps in CCW direction from initial postion)
		}
	}
}

void uStepperS::enablePid(void)
{
	cli();
	this->pidDisabled = 0;
	sei();
}

void uStepperS::disablePid(void)
{
	cli();
	this->pidDisabled = 1;
	sei();
}

float uStepperS::getPidError(void)
{
	return this->currentPidError;
}


void uStepperS::setProportional(float P)
{
	this->pTerm = P;
}

void uStepperS::setIntegral(float I)
{
	this->iTerm = I * ENCODERINTPERIOD; 
}

void uStepperS::setDifferential(float D)
{
	this->dTerm = D * ENCODERINTFREQ;
}

void uStepperS::invertDropinDir(bool invert)
{
	this->invertPidDropinDirection = invert;
}

void uStepperS::parseCommand(String *cmd)
{
  uint8_t i = 0;
  String value;

  if(cmd->charAt(2) == ';')
  {
    Serial.println("COMMAND NOT ACCEPTED");
    return;
  }

  value.remove(0);
  /****************** SET P Parameter ***************************
  *                                                            *
  *                                                            *
  **************************************************************/
  if(cmd->substring(0,2) == String("P="))
  {
    for(i = 2;;i++)
    {
      if(cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
      {
        value.concat(cmd->charAt(i));
      }
      else if(cmd->charAt(i) == '.')
      {
        value.concat(cmd->charAt(i));
        i++;
        break;
      }
      else if(cmd->charAt(i) == ';')
      {
        break;
      }
      else
      {
        Serial.println("COMMAND NOT ACCEPTED");
        return;
      }
    }
    
    for(;;i++)
    {
      if(cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
      {
        value.concat(cmd->charAt(i));
      }
      else if(cmd->charAt(i) == ';')
      {
        Serial.print("COMMAND ACCEPTED. P = ");
        Serial.println(value.toFloat(),4);
        this->dropinSettings.P.f = value.toFloat();
    	this->saveDropinSettings();
        this->setProportional(value.toFloat());
        return;
      }
      else
      {
        Serial.println("COMMAND NOT ACCEPTED");
        return;
      }
    }
  }

/****************** SET I Parameter ***************************
  *                                                            *
  *                                                            *
  **************************************************************/
  else if(cmd->substring(0,2) == String("I="))
  {
    for(i = 2;;i++)
    {
      if(cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
      {
        value.concat(cmd->charAt(i));
      }
      else if(cmd->charAt(i) == '.')
      {
        value.concat(cmd->charAt(i));
        i++;
        break;
      }
      else if(cmd->charAt(i) == ';')
      {
        break;
      }
      else
      {
        Serial.println("COMMAND NOT ACCEPTED");
        return;
      }
    }
    
    for(;;i++)
    {
      if(cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
      {
        value.concat(cmd->charAt(i));
      }
      else if(cmd->charAt(i) == ';')
      {
        Serial.print("COMMAND ACCEPTED. I = ");
        Serial.println(value.toFloat(),4);
        this->dropinSettings.I.f = value.toFloat();
    	this->saveDropinSettings();
        this->setIntegral(value.toFloat());
        return;
      }
      else
      {
        Serial.println("COMMAND NOT ACCEPTED");
        return;
      }
    }
  }

/****************** SET D Parameter ***************************
  *                                                            *
  *                                                            *
  **************************************************************/
  else if(cmd->substring(0,2) == String("D="))
  {
    for(i = 2;;i++)
    {
      if(cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
      {
        value.concat(cmd->charAt(i));
      }
      else if(cmd->charAt(i) == '.')
      {
        value.concat(cmd->charAt(i));
        i++;
        break;
      }
      else if(cmd->charAt(i) == ';')
      {
        break;
      }
      else
      {
        Serial.println("COMMAND NOT ACCEPTED");
        return;
      }
    }
    
    for(;;i++)
    {
      if(cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
      {
        value.concat(cmd->charAt(i));
      }
      else if(cmd->charAt(i) == ';')
      {
        Serial.print("COMMAND ACCEPTED. D = ");
        Serial.println(value.toFloat(),4);
        this->dropinSettings.D.f = value.toFloat();
    	this->saveDropinSettings();
        this->setDifferential(value.toFloat());
        return;
      }
      else
      {
        Serial.println("COMMAND NOT ACCEPTED");
        return;
      }
    }
  }

/****************** invert Direction ***************************
  *                                                            *
  *                                                            *
  **************************************************************/
  else if(cmd->substring(0,6) == String("invert"))
  {
      if(cmd->charAt(6) != ';')
      {
        Serial.println("COMMAND NOT ACCEPTED");
        return;
      }
      if(this->invertPidDropinDirection)
      {
      	Serial.println(F("Direction normal!"));
      	this->dropinSettings.invert = 0;
    	this->saveDropinSettings();
        this->invertDropinDir(0);
        return;
      }
      else
      {
      	Serial.println(F("Direction inverted!"));
      	this->dropinSettings.invert = 1;
    	this->saveDropinSettings();
        this->invertDropinDir(1);
        return;
      }
  }

  /****************** get Current Pid Error ********************
  *                                                            *
  *                                                            *
  **************************************************************/
  else if(cmd->substring(0,5) == String("error"))
  {
      if(cmd->charAt(5) != ';')
      {
        Serial.println("COMMAND NOT ACCEPTED");
        return;
      }
      Serial.print(F("Current Error: "));
      Serial.print(this->getPidError());
      Serial.println(F(" Steps"));
  }

  /****************** Get run/hold current settings ************
  *                                                            *
  *                                                            *
  **************************************************************/
  else if(cmd->substring(0,7) == String("current"))
  {
      if(cmd->charAt(7) != ';')
      {
        Serial.println("COMMAND NOT ACCEPTED");
        return;
      }
      Serial.print(F("Run Current: "));
      Serial.print(ceil(((float)this->driver.current)/0.31));
      Serial.println(F(" %"));
      Serial.print(F("Hold Current: "));
      Serial.print(ceil(((float)this->driver.holdCurrent)/0.31));
      Serial.println(F(" %"));
  }
  
  /****************** Get PID Parameters ***********************
  *                                                            *
  *                                                            *
  **************************************************************/
  else if(cmd->substring(0,10) == String("parameters"))
  {
      if(cmd->charAt(10) != ';')
      {
        Serial.println("COMMAND NOT ACCEPTED");
        return;
      }
      Serial.print(F("P: "));
      Serial.print(this->dropinSettings.P.f,4);
      Serial.print(F(", "));
      Serial.print(F("I: "));
      Serial.print(this->dropinSettings.I.f,4);
      Serial.print(F(", "));
      Serial.print(F("D: "));
      Serial.println(this->dropinSettings.D.f,4);
  }

  /****************** Help menu ********************************
  *                                                            *
  *                                                            *
  **************************************************************/
  else if(cmd->substring(0,4) == String("help"))
  {
      if(cmd->charAt(4) != ';')
      {
        Serial.println("COMMAND NOT ACCEPTED");
        return;
      }
      this->dropinPrintHelp();
  }

/****************** SET run current ***************************
  *                                                            *
  *                                                            *
  **************************************************************/
  else if(cmd->substring(0,11) == String("runCurrent="))
  {
    for(i = 11;;i++)
    {
      if(cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
      {
        value.concat(cmd->charAt(i));
      }
      else if(cmd->charAt(i) == '.')
      {
        value.concat(cmd->charAt(i));
        i++;
        break;
      }
      else if(cmd->charAt(i) == ';')
      {
        break;
      }
      else
      {
        Serial.println("COMMAND NOT ACCEPTED");
        return;
      }
    }
    
    for(;;i++)
    {
      if(cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
      {
        value.concat(cmd->charAt(i));
      }
      else if(cmd->charAt(i) == ';')
      {
      	if(value.toFloat() >= 0.0 && value.toFloat() <= 100.0)
      	{
      		i = (uint8_t)value.toFloat();
    		break;	
      	}
      	else
      	{
      		Serial.println("COMMAND NOT ACCEPTED");
        	return;
      	}
      }
      else
      {
        Serial.println("COMMAND NOT ACCEPTED");
        return;
      }
    }

    Serial.print("COMMAND ACCEPTED. runCurrent = ");
    Serial.print(i);
    Serial.println(F(" %"));
    this->dropinSettings.runCurrent = i;
    this->saveDropinSettings();
    this->setCurrent(i);
  }

  /****************** SET run current ***************************
  *                                                            *
  *                                                            *
  **************************************************************/
  else if(cmd->substring(0,12) == String("holdCurrent="))
  {
    for(i = 12;;i++)
    {
      if(cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
      {
        value.concat(cmd->charAt(i));
      }
      else if(cmd->charAt(i) == '.')
      {
        value.concat(cmd->charAt(i));
        i++;
        break;
      }
      else if(cmd->charAt(i) == ';')
      {
        break;
      }
      else
      {
        Serial.println("COMMAND NOT ACCEPTED");
        return;
      }
    }
    
    for(;;i++)
    {
      if(cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
      {
        value.concat(cmd->charAt(i));
      }
      else if(cmd->charAt(i) == ';')
      {
      	if(value.toFloat() >= 0.0 && value.toFloat() <= 100.0)
      	{
      		i = (uint8_t)value.toFloat();
    		break;	
      	}
      	else
      	{
      		Serial.println("COMMAND NOT ACCEPTED");
        	return;
      	}
      }
      else
      {
        Serial.println("COMMAND NOT ACCEPTED");
        return;
      }
    }

    Serial.print("COMMAND ACCEPTED. holdCurrent = ");
    Serial.print(i);
    Serial.println(F(" %"));
    this->dropinSettings.holdCurrent = i;
    this->saveDropinSettings();
    this->setHoldCurrent(i);
  }

  /****************** DEFAULT (Reject!) ************************
  *                                                            *
  *                                                            *
  **************************************************************/
  else
  {
    Serial.println("COMMAND NOT ACCEPTED");
    return;
  }
  
}

void uStepperS::dropinCli()
{
	static String stringInput;
	static uint32_t t = millis();

	while(1)
	{
		while(!Serial.available())
		{
			delay(1);
			if((millis() - t) >= 500)
			{
				stringInput.remove(0);
				t = millis();
			}
		}
		t = millis();
		stringInput += (char)Serial.read();
		if(stringInput.lastIndexOf(';') > -1)
		{
		  this->parseCommand(&stringInput);
		  stringInput.remove(0);
		}
	}
}

void uStepperS::dropinPrintHelp()
{
	Serial.println(F("uStepper S Dropin !"));
	Serial.println(F(""));
	Serial.println(F("Usage:"));
	Serial.println(F("Show this command list: 'help;'"));
	Serial.println(F("Get PID Parameters: 'parameters;'"));
	Serial.println(F("Set Proportional constant: 'P=10.002;'"));
	Serial.println(F("Set Integral constant: 'I=10.002;'"));
	Serial.println(F("Set Differential constant: 'D=10.002;'"));
	Serial.println(F("Invert Direction: 'invert;'"));
	Serial.println(F("Get Current PID Error: 'error;'"));
	Serial.println(F("Get Run/Hold Current Settings: 'current;'"));
	Serial.println(F("Set Run Current (percent): 'runCurrent=50.0;'"));
	Serial.println(F("Set Hold Current (percent): 'holdCurrent=50.0;'"));
	Serial.println(F(""));
	Serial.println(F(""));
}