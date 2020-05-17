//#define DEBUG
//#define NO_LED

#ifndef NO_LED
#define LED_PIN 3
#endif

#include <Arduino.h>
#include <uStepperS.h>
#include <Wire.h>

#define I2C_SLAVE_ADDRESS 0x04

uStepperS stepper;
const unsigned int aliveCycleT = 100; // [ms] main loop delay time
int32_t req_pos = 0; // requested position in microsteps
int32_t stage_length = 0; // this is zero when homing has not been done
bool blocked = false; // set this when we don't want to accept new movement commands
bool pending_cmd = false; // set this when the main loop should execute a command

// response buffer for i2c responses to requests from the master
#define RESP_BUF_LEN 20
char resp_buf[RESP_BUF_LEN] = { 'f' };

// command buffer
#define CMD_BUF_LEN 20
char cmd_buf[CMD_BUF_LEN] = { 0x00 };

#define MAIN_LOOP_DELAY 500

void receiveEvent(int); // when master sends bytes
void requestEvent(void); // when master asks for bytes
int wait_for_move(uint32_t);
int wait_for_move(uint32_t timeout_ms = 1000ul);
void home(void);
bool is_stalled(uint32_t);

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
#endif
  stepper.setup(); //init the uStepper S
  stepper.setMaxAcceleration(2000);
  stepper.setMaxDeceleration(2000);
  stepper.setMaxVelocity(1200);
  
  // set current limits
  stepper.setCurrent(45.0);
  stepper.setHoldCurrent(10.0);
  
  // reset encoder position to zero
  stepper.driver.writeRegister(X_ENC, 0x00);

#ifndef NO_LED
  // setupLED pin
  digitalWrite(LED_PIN, LOW); // light off
  pinMode(LED_PIN, OUTPUT); // light enabled
#endif // NO_LED
  
  // setup I2C
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
#ifdef DEBUG
  Serial.println("Setup complete.");
#endif // DEBUG
  delay(MAIN_LOOP_DELAY);
#ifdef DEBUG
  req_pos = 60000; // set bounce magnitude for testing
#endif // DEBUG
}

void loop() {
#ifndef NO_LED
  //toggle the alive pin
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
#endif // NO_LED
  delay(aliveCycleT);

  // handle a command
  if (pending_cmd){ // there's a command to be processed
    pending_cmd = false;
    switch(cmd_buf[0]){
      case 'h':
        home();
        break;
#ifdef DEBUG
      default:
        Serial.println("Got unknown command.");
#endif // DEBUG
    }
  }

#ifdef DEBUG
  int move_result;
  // do a bounce routine
  if (stage_length > 0) {
    Serial.println(stepper.driver.readRegister(X_ENC));
    go_to(stage_length/2 + stage_length/8);
    move_result = wait_for_move();
    if (move_result != 1){
      Serial.print("Warning: Movement terminated with code: ");
      Serial.println(move_result);
    }
    Serial.println(stepper.driver.readRegister(X_ENC));
    go_to(stage_length/2 - stage_length/8);
    move_result = wait_for_move();
    if (move_result != 1){
      Serial.print("Warning: Movement terminated with code: ");
      Serial.println(move_result);
    }
  }
#endif // DEBUG
}

// waits for a move to complete and returns a reason for why it completed
// 0 completed neither because of a stall nor because the goal position was reached
// 1 completed because target position reached
// 2 completed because there was a stall
// 3 exited because the movement seemingly never started (after waiting for it to start for timeout_ms)
int wait_for_move(uint32_t timeout_ms){ //timeout_ms defaults to 1000ul
  uint32_t ramp_stat;
  //bool saw_zerowait = false;
  bool saw_stall = false;
  bool saw_position_reached = false;
  bool saw_nonzero_velocity = false;
  bool timedout = false;
  unsigned long start_ms = millis();
  unsigned long current_ms;

  do {
    ramp_stat = stepper.driver.readRegister(RAMP_STAT);
    //drv_stat = stepper.driver.readRegister(DRV_STATUS);
    if ( is_stalled(ramp_stat) ){ //stall bit
      if (saw_stall == false){ // we stalled
        stepper.driver.writeRegister(VMAX_REG, 0); // for vel or pos mode
        stepper.driver.writeRegister(VSTART_REG, 0); // only needed for pos mode
        stepper.driver.writeRegister(RAMPMODE, HOLD_MODE);
      }
      saw_stall = true;
    }

    //if ( (ramp_stat & ((uint32_t)1)<<11) == ((uint32_t)1<<11) ){ // zerowait bit
    //  saw_zerowait = true;
    //}

    if ( (ramp_stat & ((uint32_t)1)<<9) == ((uint32_t)1<<9) ){  // position reached bit
      saw_position_reached = true;
    }
    
    if ( (ramp_stat & ((uint32_t)1)<<10) != ((uint32_t)1<<10) )  { // non-zero velocity
        saw_nonzero_velocity = true;
        //saw_zerowait = false; // clear this if we've seen it
        //saw_position_reached = false;
        //saw_stall = false;
      } else { // the velocity is now zero, what should we do?
      // if we've never seen motion in the past, check the timer
      if (saw_nonzero_velocity == false){
        current_ms = millis();
        if ((current_ms - start_ms) >= timeout_ms){
          timedout = true;
        break;
        }
      } else { // we have seen motion in the past, we're done
        break;
      }
    }
  } while ( true );

  // wait for velocity to be zero, (hopefully blasted through)
  while( !((stepper.driver.readRegister(RAMP_STAT) & ((uint32_t)1)<<10) == ((uint32_t)1<<10)) ); //vzero bit
  // wait for the conclusion of the zerowait period
  while( (stepper.driver.readRegister(RAMP_STAT) & ((uint32_t)1)<<11) == ((uint32_t)1<<11) ); //zerowait bit
  // wait for the standstill bit to be set
  while( !((stepper.driver.readRegister(DRV_STATUS) & ((uint32_t)1)<<31) == ((uint32_t)1<<31)) ); //standstill bit

  if (saw_stall){

    //stepper.driver.writeRegister( SW_MODE, SG_STOP(0));

    // update target to be actual
    stepper.driver.writeRegister(XTARGET, stepper.driver.readRegister(XACTUAL));
    stepper.driver.writeRegister(VMAX_REG, stepper.driver.VMAX);
    stepper.driver.writeRegister(RAMPMODE, POSITIONING_MODE);

    // reset coolconf
    //stepper.driver.writeRegister( COOLCONF, SGT(0x00)| SEMIN(7));
    

    //Serial.println("AA");
    while( stepper.driver.readRegister(GSTAT) != 0 ); // global status flags
    //Serial.println("BB");
    while( (stepper.driver.readRegister(RAMP_STAT) & ((uint32_t)1)<<6) == ((uint32_t)1<<6) ); //event_stop_sg bit
    //Serial.println("CC");
    while( (stepper.driver.readRegister(RAMP_STAT) & ((uint32_t)1)<<13) == ((uint32_t)1<<13) ); //status_sg bit

    //Serial.println("DD");
    //Serial.println(stepper.driver.readRegister(DRV_STATUS) & 0x3FF);
    //while( (stepper.driver.readRegister(DRV_STATUS) & ((uint32_t)1)<<24) == ((uint32_t)1<<24) ); //StallGuard bit (I can't seem clear this)
    //Serial.println("EE");
    //stepper.driver.writeRegister(VMAX_REG, vmax_old);

    // re-ebable motion
    //stepper.driver.writeRegister(VSTART_REG, vstart_old);
    //stepper.driver.writeRegister(VMAX_REG, stepper.driver.VMAX);

    // switch to positioning mode no matter what. we don't want vel mode if we just stalled 
    //stepper.driver.writeRegister(RAMPMODE, POSITIONING_MODE);

    
    //stepper.driver.writeRegister(VSTART_REG, vstart_old);
    //stepper.driver.writeRegister(VMAX_REG, stepper.driver.VMAX);
    //stepper.driver.writeRegister( SW_MODE, SG_STOP(1));

    return(2);
  }

  if(saw_position_reached){
    return(1);
  }

  if(timedout){
    return(3);
  }

  return(0);
}

bool is_stalled(uint32_t ramp_stat){
  bool stalled;
  //if ( ((ramp_stat & ((uint32_t)1)<<13) == ((uint32_t)1<<13)) || ((ramp_stat & ((uint32_t)1)<<6) == ((uint32_t)1<<6)) || ((drv_stat & ((uint32_t)1)<<24) == ((uint32_t)1<<24)) ){ //stall bit
  if ( ((ramp_stat & ((uint32_t)1)<<13) == ((uint32_t)1<<13)) || ((ramp_stat & ((uint32_t)1)<<6) == ((uint32_t)1<<6)) ){ //stall bit
    stalled = true;
  } else {
    stalled = false;
  }
  return (stalled);
}

//check for a stall and handle it
bool check_stall(){
  bool stalled = true;
  uint32_t ramp_stat = stepper.driver.readRegister(RAMP_STAT);
  if (is_stalled(ramp_stat)){
    stepper.driver.writeRegister( VMAX_REG, 0 );
    stage_length = 0;
    stalled = true;
  } else {
    stalled = false;
  }
  return(stalled);
}

// sends the stage somewhere
bool go_to (int32_t pos) {
  // TODO: need to more aggressively limit travel to not go close to edges
  if ( (pos > 0) && (pos <= stage_length) && !blocked && !check_stall()) {
    stepper.driver.writeRegister(XTARGET, pos);
    return true;
  } else {
    return false;
  }
}

// homing/stage length measurement task
void home(void){
  int32_t tmp_stage_len = 0;
  blocked = true; // block movement during homing
  stage_length = -1;

#ifdef DEBUG
  Serial.println("Homing procedure initiated!");
  Serial.println(wait_for_move(100ul));
#else
  wait_for_move(100ul);
#endif // DEBUG
  // jog negatively
  stepper.driver.writeRegister( VMAX_REG, stepper.driver.VMAX );
  stepper.driver.writeRegister(RAMPMODE, VELOCITY_MODE_NEG);
  //stepper.driver.writeRegister( SW_MODE, SG_STOP(0));
  //stepper.driver.writeRegister(XTARGET, (int32_t)stepper.driver.readRegister(XACTUAL) - 0x00FFFFFF);

#ifdef DEBUG
  Serial.println("Jogging to zero");
  Serial.println(wait_for_move());
#else // DEBUG
  wait_for_move();
#endif // DEBUG
  delay(500); // wait after hitting zero stop
  // define zero (home)
  stepper.driver.writeRegister( XACTUAL, 0 );  
  stepper.driver.writeRegister( X_ENC, 0 );
  //stepper.driver.writeRegister(XTARGET, 0x00FFFFFF); // jog pos
  stepper.driver.writeRegister( RAMPMODE, VELOCITY_MODE_POS );
  //stepper.driver.writeRegister( SW_MODE, SG_STOP(0));
#ifdef DEBUG
  Serial.println("Jogging to max");
  Serial.println(wait_for_move());
  Serial.println("Max reached");
#else
  wait_for_move();
#endif // DEBUG
  delay(500); // wait after hitting max stop
  
  tmp_stage_len = stepper.driver.readRegister(XACTUAL);
  //tmp_stage_len = stepper.driver.readRegister(X_ENC);
#ifdef DEBUG
  Serial.print("The stage is ");
  Serial.print(tmp_stage_len);
  Serial.println(" steps long.");
#endif // DEBUG

  // now move off the end point for one second without stallguard
  //stepper.driver.writeRegister( SW_MODE, SG_STOP(0) );
  //stepper.driver.writeRegister(VMAX_REG, stepper.driver.VMAX/4);
  stepper.driver.writeRegister( RAMPMODE, VELOCITY_MODE_NEG );
  delay(1000); // move back for 1 second
  stepper.driver.writeRegister( VMAX_REG, 0 );
#ifdef DEBUG
  Serial.println(wait_for_move());
#else
  wait_for_move();
#endif //DEBUG
  //stepper.driver.writeRegister( SW_MODE, SG_STOP(1) );
  stepper.driver.writeRegister( XTARGET, stepper.driver.readRegister(XACTUAL) );
  stepper.driver.writeRegister( VMAX_REG, stepper.driver.VMAX );
  stepper.driver.writeRegister( RAMPMODE, POSITIONING_MODE );

  blocked = false;
  stage_length = tmp_stage_len;
}


// function that executes whenever data is received from master
// this function is registered as an event, see setup()
// an ISR
void receiveEvent(int howMany){
  //Serial.println('rcv');
  int i = 0; // bytes received, bad things will happend if this gets higher than cmd_buf_len
  while(Wire.available() > 0 && i < CMD_BUF_LEN){ // loop through the bytes until there are no more
    cmd_buf[i] = Wire.read(); // put the char into the command buffer
    i++;
  }
}


// fires when the master asks for bytes
// an ISR
void requestEvent(void){
  //Serial.println('req');
  uint32_t tmp = 0;
  switch(cmd_buf[0]){
    case 'h':  //home
     if (blocked){
       Wire.write('f'); // press f for fail
     } else {
       Wire.write('p'); // press p for pass
       pending_cmd = true;
     }
     break;
    case 'g':  //goto
      memcpy(&req_pos, &cmd_buf[1], 4); // copy in target
      if (go_to(req_pos)){
        Wire.write('p'); //press P for pass
      } else {
        Wire.write('f'); //press F for fail
      }
      break;
    case 'l': // get stage length
      resp_buf[0] = 'p'; // press p for pass
      resp_buf[1] = (stage_length >> 24) & 0xFF;
      resp_buf[2] = (stage_length >> 16) & 0xFF;
      resp_buf[3] = (stage_length >> 8) & 0xFF;
      resp_buf[4] = stage_length & 0xFF;
      Wire.write((uint8_t*)resp_buf, 5);
      break;
    case 'r': // read back position request
      if (blocked){
        Wire.write('f'); // press f for fail
      } else {
        resp_buf[0] = 'p'; // press p for pass
        tmp = stepper.driver.readRegister(XACTUAL); // TODO: one day think about sending the encoder position
        resp_buf[1] = (tmp >> 24) & 0xFF;
        resp_buf[2] = (tmp >> 16) & 0xFF;
        resp_buf[3] = (tmp >> 8) & 0xFF;
        resp_buf[4] = tmp & 0xFF;
        Wire.write((uint8_t*)resp_buf, 5);
      }
      break;
    default:
      Wire.write('f'); //press F for fail
  }
}
