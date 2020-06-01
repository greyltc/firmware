//#define DEBUG
//#define NO_LED

#ifndef NO_LED
#define LED_PIN 3
#endif

#include <Arduino.h>
#include <uStepperS.h>
#include <Wire.h>

#define I2C_SLAVE_ADDRESS 0x04

int wait_for_move(unsigned long timeout_ms = 1000);

uStepperS stepper;
const unsigned int aliveCycleT = 100; // [ms] main loop delay time
volatile int32_t stage_length = 0; // this is zero when homing has not been done, -1 while homing
volatile bool blocked = false; // set this when we don't want to accept new movement commands
volatile bool send_later = false; // the master has asked for bytes but we have none ready

// response buffer for i2c responses to requests from the master
#define RESP_BUF_LEN 20
static uint8_t out_buf[RESP_BUF_LEN] = { 'f' };
volatile int bytes_ready = 0;

// command buffer
#define CMD_BUF_LEN 20
volatile uint8_t cmd_byte = 0x00;
static uint8_t in_buf[CMD_BUF_LEN] = { 0x00 };

// time to wait before starting the main loop
#define MAIN_LOOP_DELAY 500

void receiveEvent(int);
bool requestEvent(void);
int handle_cmd(bool);
bool go_to (int32_t);
int32_t home(void);
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
  Wire.setWireTimeoutUs(25000ul, true);
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
#ifdef DEBUG
  Serial.println("Setup complete.");
#endif // DEBUG
  // wait a bit before starting the main loop
  delay(MAIN_LOOP_DELAY);
#ifdef DEBUG
  req_pos = 60000; // set bounce magnitude for testing
#endif // DEBUG
  digitalWrite(LED_PIN, LOW);
}

#ifndef NO_LED
uint32_t led_loops = 5000ul;
#endif // NO_LED

// how many loops we've done
uint32_t num_loops = 0ul;

int rdy_to_send; // to store how many bytes are ready to send up to the master

void loop() {
  num_loops++;
#ifndef NO_LED
  //toggle the alive pin
  if (num_loops%led_loops == 0){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    //PIND = PIND | 0x01<<3; // fast LED on
  }
#endif // NO_LED

  // handle a command
  if (cmd_byte != 0x00){
    // handle the command in slo mode (commands can take a long time to complete)
    rdy_to_send = handle_cmd(true);
    if (rdy_to_send > 0){
      
      while (!send_later){ // wait for the master to ask us for bytes
        continue;
      }
      send_later = false;
      
      Wire.write((uint8_t*)out_buf, rdy_to_send);
      Wire.flush();
    }
  }

#ifdef DEBUG // bouncy debug
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

// processes commands from cmd_buf and cmd_byte
// slo mode indicates that commands which requre "slow" processing should be handled
// where "slow" is any processing that requires delays, external access or anything beyond simple variable lookups
// slo_mode is true when called from the main loop and false when called from an ISR
// places response bytes (if any) into out_buf and returns number of bytes it put there
int handle_cmd(bool slo_mode){
  int rdy_to_send = 0; // number of bytes ready to send
  char this_cmd = cmd_byte;
  int32_t pos = 0; // for storing position data
  switch (this_cmd){
    case 'g': // goto
      if(blocked){
        out_buf[0] = 'f';
        cmd_byte = 0x00; // clear the cmd _byte. it has been handled
        rdy_to_send = 1;
      } else { //not blocked
        if(slo_mode){ // processing in main loop, not ISR
          memcpy(&pos, in_buf, sizeof(pos)); // copy in target pos
          blocked = true; // block duiring go_to
          if (go_to(pos)){
            out_buf[0] = 'p';
            cmd_byte = 0x00; // clear the cmd _byte. it has been handled
            rdy_to_send = 1;
          } else {
            out_buf[0] = 'f';
            cmd_byte = 0x00; // clear the cmd _byte. it has been handled
            rdy_to_send = 1;
          }
          blocked = false;
        }
      }
      break;

    case 'h': // home command
      if(blocked){
        out_buf[0] = 'f';
        cmd_byte = 0x00; // clear the cmd _byte. it has been handled
        rdy_to_send = 1;
      } else { //not blocked
        if(slo_mode){  // processing in main loop, not ISR
          cmd_byte = 0x00; // clear the cmd _byte. it has been handled
          blocked = true; // block duiring home
          stage_length = home();
          blocked = false;
        } else { // processing in ISR, not main loop.
          // accept the homing command, and respond immediately to the master
          // but don't actually execute the homing procedure here
          // instead we leave that to be done in the main loop later
          stage_length = -1;
          out_buf[0] = 'p';
          rdy_to_send = 1;
        }
      }
      break;

    case 'l': // get stage length command
      out_buf[0] = 'p'; // press p for pass
      out_buf[1] = (stage_length >> 24) & 0xFF;
      out_buf[2] = (stage_length >> 16) & 0xFF;
      out_buf[3] = (stage_length >> 8) & 0xFF;
      out_buf[4] = stage_length & 0xFF;
      cmd_byte = 0x00; // clear the cmd _byte. it has been handled
      //_delay_us(10000); // for testing to check if we miss the master's request for bytes with this
      rdy_to_send = 5;
      break;
    
    case 'r': // read back position command
      if(blocked){
        out_buf[0] = 'f';
        cmd_byte = 0x00; // clear the cmd _byte. it has been handled
        rdy_to_send = 1;
      } else { //not blocked
        if(slo_mode){  // processing in main loop, not ISR
          out_buf[0] = 'p'; // press p for pass
          blocked = true; // block duiring postition register read
          pos = stepper.driver.readRegister(XACTUAL); // TODO: one day think about sending the encoder position
          blocked = false;
          out_buf[1] = (pos >> 24) & 0xFF;
          out_buf[2] = (pos >> 16) & 0xFF;
          out_buf[3] = (pos >> 8) & 0xFF;
          out_buf[4] = pos & 0xFF;
          cmd_byte = 0x00; // clear the cmd _byte. it has been handled
          rdy_to_send = 5;
        }
      }
      break;

    default:
      out_buf[0] = 'f'; // press f for fail
      cmd_byte = 0x00; // clear the cmd_byte. it has been handled
      rdy_to_send = 1;
  } // end switch
  return(rdy_to_send);
}

// waits for a move to complete and returns a reason for why it completed
// 0 completed neither because of a stall nor because the goal position was reached
// 1 completed because target position reached
// 2 completed because there was a stall
// 3 exited because the movement seemingly never started (after waiting for it to start for timeout_ms)
int wait_for_move(unsigned long timeout_ms){ //timeout_ms defaults to 1000
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
  if ( (pos > 0) && (pos <= stage_length) && !check_stall()) {
    stepper.driver.writeRegister(XTARGET, pos);
    return true;
  } else {
    return false;
  }
}

// homing/stage length measurement task
int32_t home(void){
  int32_t measured_len = 0;

#ifdef DEBUG
  Serial.println("Homing procedure initiated!");
  Serial.println(wait_for_move(100));
#else
  wait_for_move(100);
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
  
  measured_len = stepper.driver.readRegister(XACTUAL);
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

  return (measured_len);
}

// fires off whenever the master sends us bytes
// is called from an ISR in the Wire library
void receiveEvent(int how_many){
  int i;
  if (how_many > 0){
    cmd_byte = Wire.read();
  }
  for (i=1; (i<how_many) && (i<CMD_BUF_LEN); i++){
    in_buf[i-1] = Wire.read();
  }

  // attempt to handle the command right in the ISR
  bytes_ready = handle_cmd(false);
}

// fires off whenever when the master asks for bytes
// is called from an ISR in the Wire library
bool requestEvent(void){
  send_later = false;
  if (bytes_ready > 0){ // if we have something to send, send it right away
    //send_later = false;
    Wire.write((uint8_t*)out_buf, bytes_ready);
    bytes_ready = 0;
  } else { // no bytes ready
    // we've been unable to respond to the master from the ISR
    // we'll have to do it later, in the main loop
    send_later = true; // this will trigger clock stretching
    // which will be ended by the call to Wire.flush() from the main loop
  }
  return(send_later);
}
