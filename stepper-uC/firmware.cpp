#define VERSION_MAJOR 0
#define VERSION_MINOR 3
#define VERSION_PATCH 2
#define BUILD e7fe2a6

//#define DEBUG
//#define NO_LED

// maximum velocity in microsteps/s
#define VMAX 200000

// axis:address --> 1:0x50, 2:0x51, 3:0x52
#define I2C_SLAVE_ADDRESS 0x52

#include <Arduino.h>
#include <Wire.h>
#include <util/delay.h>
#include <avr/io.h>
extern "C" {
  #include "tmc/ic/TMC5130/TMC5130.h"
}


// create version string
#define STRINGIFY0(s) # s
#define STRINGIFY(s) STRINGIFY0(s)
#define FIRMWARE_VER STRINGIFY(VERSION_MAJOR) "." STRINGIFY(VERSION_MINOR) "." STRINGIFY(VERSION_PATCH) "+" STRINGIFY(BUILD)

#ifndef NO_LED
#define LED_PIN 3
#endif

#define BIT_SET(VALUE, BIT_POSITION) ((VALUE) |= (1<<(BIT_POSITION)))
#define BIT_CLEAR(VALUE, BIT_POSITION) ((VALUE) &= ~(1<<(BIT_POSITION)))

#define PIN_MODE(PIN_NAME, IO_MODE) (PIN_MODE_ ## IO_MODE(PIN_NAME ## _DDR, PIN_NAME ## _BIT))
#define PIN_MODE_OUTPUT(DDR, BIT_POSITION) BIT_SET(DDR, BIT_POSITION)
#define PIN_MODE_INPUT(DDR, BIT_POSITION) BIT_CLEAR(DDR, BIT_POSITION)

// pin defs
#define MISO_DDR DDRC
#define MISO_PORT PORTC
#define MISO_PIN PINC
#define MISO_BIT 0

#define MOSI_DDR DDRE
#define MOSI_PORT PORTE
#define MOSI_PIN PINE
#define MOSI_BIT 3

#define SCK_DDR DDRC
#define SCK_PORT PORTC
#define SCK_PIN PINC
#define SCK_BIT 1

#define SS_DDR DDRE
#define SS_PORT PORTE
#define SS_PIN PINE
#define SS_BIT 2

#define SD_MODE_DDR DDRD
#define SD_MODE_PORT PORTD
#define SD_MODE_PIN PIND
#define SD_MODE_BIT 5

#define ENN_DDR DDRD
#define ENN_PORT PORTD
#define ENN_PIN PIND
#define ENN_BIT 4

#define RAMP_POS	0
#define RAMP_FWD	1
#define RAMP_REV	2
#define RAMP_HOLD	3

int wait_for_move(unsigned long timeout_ms = 1000);
void receiveEvent(int);
bool requestEvent(void);
int handle_cmd(bool);
bool go_to (int32_t);
int32_t home(void);
bool is_stalled(uint32_t);
void spi_setup(void);
void tmc5130_readWriteArray(uint8_t , uint8_t *, size_t );

void tmc5130_writeDatagram(uint8_t, uint8_t , uint8_t , uint8_t , uint8_t , uint8_t );
int32_t tmc5130_readInt(uint8_t, uint8_t );
void tmc5130_writeInt(uint8_t, uint8_t , int32_t );

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

// allows for different channels
uint8_t TMC5130 = 0;

void setup() {

  // disable drive
  BIT_SET(ENN_PORT, ENN_BIT);
  PIN_MODE(ENN, OUTPUT);

  // use internal ramp generator
  BIT_CLEAR(SD_MODE_PORT, SD_MODE_BIT);
  PIN_MODE(SD_MODE, OUTPUT);

  spi_setup();

#ifdef DEBUG
  Serial.begin(9600);
#endif

  tmc5130_writeInt(TMC5130, TMC5130_XACTUAL, 0);
  tmc5130_writeInt(TMC5130, TMC5130_XTARGET, 0);

  tmc5130_writeInt(TMC5130, TMC5130_CHOPCONF, 0);
  tmc5130_writeInt(TMC5130, TMC5130_GCONF, 0);
  tmc5130_writeInt(TMC5130, TMC5130_PWMCONF, 0x00050480);
  tmc5130_writeInt(TMC5130, TMC5130_TPWMTHRS, 0);
  tmc5130_writeInt(TMC5130, TMC5130_RAMPMODE, 0);

  tmc5130_writeInt(TMC5130, TMC5130_VSTART, 0);
  tmc5130_writeInt(TMC5130, TMC5130_A1, 1000);
  tmc5130_writeInt(TMC5130, TMC5130_V1, 50000);
  tmc5130_writeInt(TMC5130, TMC5130_AMAX, 2000);
  tmc5130_writeInt(TMC5130, TMC5130_VMAX, VMAX);
  tmc5130_writeInt(TMC5130, TMC5130_DMAX, 2000);
  tmc5130_writeInt(TMC5130, TMC5130_D1, 2400);
  tmc5130_writeInt(TMC5130, TMC5130_VSTOP, 10);

  tmc5130_writeInt(TMC5130, TMC5130_DRVSTATUS, 0);
  tmc5130_writeInt(TMC5130, TMC5130_RAMPSTAT, 0);
  tmc5130_writeInt(TMC5130, TMC5130_SWMODE, 0);

  tmc5130_writeInt(TMC5130, TMC5130_COOLCONF, 0);
  tmc5130_writeInt(TMC5130, TMC5130_TSTEP, 2000);
  tmc5130_writeInt(TMC5130, TMC5130_ENCMODE, 0);
  tmc5130_writeInt(TMC5130, TMC5130_ENC_CONST, (int32_t)-26214399);

  TMC5130_FIELD_WRITE(TMC5130, TMC5130_GCONF, TMC5130_EN_PWM_MODE_MASK, TMC5130_EN_PWM_MODE_SHIFT, 0);
  TMC5130_FIELD_WRITE(TMC5130, TMC5130_GCONF, TMC5130_I_SCALE_ANALOG_MASK, TMC5130_I_SCALE_ANALOG_SHIFT, 1);
  TMC5130_FIELD_WRITE(TMC5130, TMC5130_GCONF, TMC5130_ENC_COMMUTATION_MASK, TMC5130_ENC_COMMUTATION_SHIFT, 0);

  uint32_t pwm_reg_val = 0;
  pwm_reg_val = FIELD_SET(pwm_reg_val, TMC5130_PWM_AMPL_MASK, TMC5130_PWM_AMPL_SHIFT, 128);
  pwm_reg_val = FIELD_SET(pwm_reg_val, TMC5130_PWM_GRAD_MASK, TMC5130_PWM_GRAD_SHIFT, 1);
  pwm_reg_val = FIELD_SET(pwm_reg_val, TMC5130_PWM_FREQ_MASK, TMC5130_PWM_FREQ_SHIFT, 0);
  pwm_reg_val = FIELD_SET(pwm_reg_val, TMC5130_PWM_AUTOSCALE_MASK, TMC5130_PWM_AUTOSCALE_SHIFT, 1);
  pwm_reg_val = FIELD_SET(pwm_reg_val, TMC5130_FREEWHEEL_MASK, TMC5130_FREEWHEEL_SHIFT, 1);
  tmc5130_writeInt(TMC5130, TMC5130_PWMCONF, pwm_reg_val);

  tmc5130_writeInt(TMC5130, TMC5130_TPWMTHRS, 5000);

  TMC5130_FIELD_WRITE(TMC5130, TMC5130_CHOPCONF, TMC5130_TOFF_MASK, TMC5130_TOFF_SHIFT, 2);
  TMC5130_FIELD_WRITE(TMC5130, TMC5130_CHOPCONF, TMC5130_TFD_ALL_MASK, TMC5130_TFD_ALL_SHIFT, 4);
  TMC5130_FIELD_WRITE(TMC5130, TMC5130_CHOPCONF, TMC5130_OFFSET_MASK, TMC5130_OFFSET_SHIFT, 0);
  TMC5130_FIELD_WRITE(TMC5130, TMC5130_CHOPCONF, TMC5130_RNDTF_MASK, TMC5130_RNDTF_SHIFT, 0);
  TMC5130_FIELD_WRITE(TMC5130, TMC5130_CHOPCONF, TMC5130_CHM_MASK, TMC5130_CHM_SHIFT, 1);
  TMC5130_FIELD_WRITE(TMC5130, TMC5130_CHOPCONF, TMC5130_TBL_MASK, TMC5130_TBL_SHIFT, 2);
  TMC5130_FIELD_WRITE(TMC5130, TMC5130_CHOPCONF, TMC5130_SYNC_MASK, TMC5130_SYNC_SHIFT, 0);
  TMC5130_FIELD_WRITE(TMC5130, TMC5130_CHOPCONF, TMC5130_INTPOL_MASK, TMC5130_INTPOL_SHIFT, 0);
  TMC5130_FIELD_WRITE(TMC5130, TMC5130_CHOPCONF, TMC5130_DEDGE_MASK, TMC5130_DEDGE_SHIFT, 0);

  // stallguard
  TMC5130_FIELD_WRITE(TMC5130, TMC5130_SWMODE, TMC5130_SW_SG_STOP_MASK, TMC5130_SW_SG_STOP_SHIFT, 1);

  // set up transition to something
  tmc5130_writeInt(TMC5130, TMC5130_TCOOLTHRS, 1000);

  // setup stall threshold
  // -64 --> least difficult to stall
  //  63 --> most difficult to stall
  uint32_t coolconf_reg_val = 0;
  //coolconf_reg_val = FIELD_SET(coolconf_reg_val, TMC5130_SGT_MASK, TMC5130_SGT_SHIFT, 0x3F); // 63  most difficult to stall
	//coolconf_reg_val = FIELD_SET(coolconf_reg_val, TMC5130_SGT_MASK, TMC5130_SGT_SHIFT, 0x1E);; // 30
  //coolconf_reg_val = FIELD_SET(coolconf_reg_val, TMC5130_SGT_MASK, TMC5130_SGT_SHIFT, 0x0F);; // 15
	coolconf_reg_val = FIELD_SET(coolconf_reg_val, TMC5130_SGT_MASK, TMC5130_SGT_SHIFT, 0x00);
	//coolconf_reg_val = FIELD_SET(coolconf_reg_val, TMC5130_SGT_MASK, TMC5130_SGT_SHIFT, 0x60); // -32
	//coolconf_reg_val = FIELD_SET(coolconf_reg_val, TMC5130_SGT_MASK, TMC5130_SGT_SHIFT, 0x30); // -48
	//coolconf_reg_val = FIELD_SET(coolconf_reg_val, TMC5130_SGT_MASK, TMC5130_SGT_SHIFT, 0x40); // -64 least difficult to stall
  tmc5130_writeInt(TMC5130, TMC5130_COOLCONF, coolconf_reg_val);

  tmc5130_writeInt(TMC5130, TMC5130_XACTUAL, 0);
  tmc5130_writeInt(TMC5130, TMC5130_XTARGET, tmc5130_readInt(TMC5130, TMC5130_XACTUAL));

  uint32_t current_reg_val = 0;

  // set run current [1,31]
  current_reg_val = FIELD_SET(current_reg_val, TMC5130_IRUN_MASK, TMC5130_IRUN_SHIFT, 14);

  // set hold/idle current [1,31]
  current_reg_val = FIELD_SET(current_reg_val, TMC5130_IHOLD_MASK, TMC5130_IHOLD_SHIFT, 4);

  // program currents
  tmc5130_writeInt(TMC5130, TMC5130_IHOLD_IRUN, current_reg_val);
  
  // reset encoder position to zero
  tmc5130_writeInt(TMC5130, TMC5130_XENC, 0);
  
  // power up drive by releasing hardware line
  BIT_CLEAR(ENN_PORT, ENN_BIT);

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
  //req_pos = 60000; // set bounce magnitude for testing
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
    Serial.println(tmc5130_readInt(TMC5130, TMC5130_XENC));
    go_to(stage_length/2 + stage_length/8);
    move_result = wait_for_move();
    if (move_result != 1){
      Serial.print("Warning: Movement terminated with code: ");
      Serial.println(move_result);
    }
    Serial.println(tmc5130_readInt(TMC5130, TMC5130_XENC));
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

    case 'c': //comms check command
      out_buf[0] = 'p'; // press p for pass
      out_buf[1] = 0xCA;
      out_buf[2] = 0XFE;
      out_buf[3] = 0XBA;
      out_buf[4] = 0XBE;
      cmd_byte = 0x00; // clear the cmd _byte. it has been handled
      //_delay_us(10000); // for testing to check if we miss the master's request for bytes with this
      rdy_to_send = 5;
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
    
    case 'e': // enable drive
      BIT_CLEAR(ENN_PORT, ENN_BIT);
      out_buf[0] = 'p'; // press p for pass
      cmd_byte = 0x00; // clear the cmd _byte. it has been handled
      rdy_to_send = 1;
      break;

    case 'd': // disable drive
      BIT_SET(ENN_PORT, ENN_BIT);
      out_buf[0] = 'p'; // press p for pass
      cmd_byte = 0x00; // clear the cmd _byte. it has been handled
      rdy_to_send = 1;
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
          pos = tmc5130_readInt(TMC5130, TMC5130_XACTUAL);
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

    ramp_stat = tmc5130_readInt(TMC5130, TMC5130_RAMPSTAT);
    if ( is_stalled(ramp_stat) ){ //stall bit
      if (saw_stall == false){ // we stalled
        tmc5130_writeInt(TMC5130, TMC5130_VMAX, 0); // for vel or pos mode
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
  ; // for vel or pos mode
  while( !((tmc5130_readInt(TMC5130, TMC5130_RAMPSTAT) & ((uint32_t)1)<<10) == ((uint32_t)1<<10)) ); //vzero bit
  // wait for the conclusion of the zerowait period
  while( (tmc5130_readInt(TMC5130, TMC5130_RAMPSTAT) & ((uint32_t)1)<<11) == ((uint32_t)1<<11) ); //zerowait bit
  // wait for the standstill bit to be set
  while( !((tmc5130_readInt(TMC5130, TMC5130_DRVSTATUS) & ((uint32_t)1)<<31) == ((uint32_t)1<<31)) ); //standstill bit

  if (saw_stall){

    // update target to be actual
    tmc5130_writeInt(TMC5130, TMC5130_XTARGET, tmc5130_readInt(TMC5130, TMC5130_XACTUAL));
    tmc5130_writeInt(TMC5130, TMC5130_VMAX, VMAX);
    tmc5130_writeInt(TMC5130, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);// positioning mode

    // reset coolconf
    

    //Serial.println("AA");
    
    while( tmc5130_readInt(TMC5130, TMC5130_GSTAT) != 0 ); // global status flags
    //Serial.println("BB");
    while( (tmc5130_readInt(TMC5130, TMC5130_RAMPSTAT) & ((uint32_t)1)<<6) == ((uint32_t)1<<6) ); //event_stop_sg bit
    //Serial.println("CC");
    while( (tmc5130_readInt(TMC5130, TMC5130_RAMPSTAT) & ((uint32_t)1)<<13) == ((uint32_t)1<<13) ); //status_sg bit

    //Serial.println("DD");

    // re-ebable motion

    // switch to positioning mode no matter what. we don't want vel mode if we just stalled 

  
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
  if (is_stalled(tmc5130_readInt(TMC5130, TMC5130_RAMPSTAT))){
    tmc5130_writeInt(TMC5130, TMC5130_VMAX, 0);
    stage_length = 0;
    stalled = true;
  } else {
    stalled = false;
  }
  return(stalled);
}

// sends the stage somewhere
bool go_to (int32_t pos) {
  //uint32_t dummy;
  // TODO: need to more aggressively limit travel to not go close to edges
  if ( (pos > 0) && (pos <= stage_length) && !check_stall()) {
    tmc5130_writeInt(TMC5130, TMC5130_XTARGET, pos);
    //tmc5130a_shift(true, XTARGET, pos, dummy);
    return true;
  } else {
    return false;
  }
}

// homing/stage length measurement task
int32_t home(void){
  int32_t measured_len = 0;
  //uint32_t dummy;

#ifdef DEBUG
  Serial.println("Homing procedure initiated!");
#endif // DEBUG
  wait_for_move(100);
  // jog negatively
  tmc5130_writeInt(TMC5130, TMC5130_VMAX, VMAX);
  tmc5130_writeInt(TMC5130, TMC5130_RAMPMODE, TMC5130_MODE_VELNEG);



#ifdef DEBUG
  Serial.println("Jogging to zero");  
#endif // DEBUG
  wait_for_move();
  // define zero (home)
  tmc5130_writeInt(TMC5130, TMC5130_XACTUAL, 0);
  tmc5130_writeInt(TMC5130, TMC5130_XENC, 0);


  tmc5130_writeInt(TMC5130, TMC5130_RAMPMODE, TMC5130_MODE_VELPOS);

#ifdef DEBUG
  Serial.println("Jogging to max");
#endif // DEBUG
  wait_for_move();

  measured_len = tmc5130_readInt(TMC5130, TMC5130_XACTUAL);

#ifdef DEBUG
  Serial.println("Max reached");
  Serial.print("The stage is ");
  Serial.print(measured_len);
  Serial.println(" steps long.");
#endif // DEBUG

  tmc5130_writeInt(TMC5130, TMC5130_XTARGET, measured_len*0.9);
  tmc5130_writeInt(TMC5130, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);

  
  wait_for_move();

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

// tmc5130a 40 bit datagram shift in & out
uint8_t tmc5130a_shift(bool write, uint8_t address, uint32_t to_send, uint32_t &got_back){
  uint8_t status = 0x00;
  uint32_t ret = 0;
  uint8_t data[5] = { 0, 0, 0, 0, 0 };

  if (write){
    // writes get the MSB set
    data[0] = 0b10000000 | address;
  } else { // read
    // reads get the MSB cleared (you screwed up your address if this does anything)
    data[0] = 0b01111111 & address;
  }
  data[1] = 0xFF & (to_send >> 24);
  data[2] = 0xFF & (to_send >> 16);
  data[3] = 0xFF & (to_send >> 8);
  data[4] = 0xFF &  to_send;

  tmc5130_readWriteArray(0, data, 5);

  status = data[0];
  ret |= (uint32_t)data[1]<<24;
  ret |= (uint32_t)data[2]<<16;
  ret |= (uint32_t)data[3]<<8;
  ret |= (uint32_t)data[4];

  got_back = ret;
  return(status);
}

// Send [length] bytes stored in the [data] array over SPI and overwrite [data]
// with the replies. data[0] is the first byte sent and received.
void tmc5130_readWriteArray(uint8_t channel, uint8_t *data, size_t length){
  unsigned int i;

  BIT_CLEAR(SS_PORT, SS_BIT); // select salve  
  for(i=0; i<length; i++){
    SPDR1 = data[i];
    while(!(SPSR1 & (1<<SPIF1)));
    data[i] = SPDR1;
  }
  BIT_SET(SS_PORT, SS_BIT);  // deselect salve
}

//setup SPI
void spi_setup(void){
  PIN_MODE(MOSI, OUTPUT);
  PIN_MODE(SCK, OUTPUT);
  PIN_MODE(SS, OUTPUT);

  PIN_MODE(MISO, INPUT);

  // Enable SPI, Master mode, phase/polarity mode 3 & keep default clock rate (fosc/4)
  SPCR1 = (1<<SPE1)|(1<<MSTR1)|(1<<CPOL1)|(1<<CPHA1);
}

// int32_t tmc5130_readIntL(TMC5130TypeDef *tmc5130, uint8_t address)
// {
// 	uint32_t ret = 0;

// 	uint8_t data[5] = { 0, 0, 0, 0, 0 };

// 	data[0] = address;
// 	tmc5130_readWriteArray(tmc5130->config->channel, &data[0], 5);

// 	data[0] = address;
// 	tmc5130_readWriteArray(tmc5130->config->channel, &data[0], 5);

// 	ret |= (uint32_t)data[1]<<24;
//   ret |= (uint32_t)data[2]<<16;
//   ret |= (uint32_t)data[3]<<8;
//   ret |= (uint32_t)data[4];

// 	return (ret);

// }

// int32_t tmc5130_readIntL2(TMC5130TypeDef *tmc5130, uint8_t address)
// {
// 	address = TMC_ADDRESS(address);
//   uint32_t ret = 0;

// 	// register not readable -> shadow register copy
// 	if(!TMC_IS_READABLE(tmc5130->registerAccess[address]))
// 		return tmc5130->config->shadowRegister[address];

// 	uint8_t data[5] = { 0, 0, 0, 0, 0 };

// 	data[0] = address;
// 	tmc5130_readWriteArray(tmc5130->config->channel, &data[0], 5);

// 	data[0] = address;
// 	tmc5130_readWriteArray(tmc5130->config->channel, &data[0], 5);

// 	return (((int32_t)data[1] << 24) | ((int32_t)data[2] << 16) | ((int32_t)data[3] << 8) | ((int32_t)data[4]));
//   //ret |= (uint32_t)data[1]<<24;
//   //ret |= (uint32_t)data[2]<<16;
//   //ret |= (uint32_t)data[3]<<8;
//   //ret |= (uint32_t)data[4];

// 	//return (ret);
// }

// === modded from tmc code  ===


// Writes (x1 << 24) | (x2 << 16) | (x3 << 8) | x4 to the given address
void tmc5130_writeDatagram(uint8_t tmc5130, uint8_t address, uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4)
{
  uint8_t addr = address | TMC5130_WRITE_BIT;
	uint8_t data[5] = {addr , x1, x2, x3, x4 };
	tmc5130_readWriteArray(tmc5130, &data[0], 5);
}

void tmc5130_writeInt(uint8_t tmc5130, uint8_t address, int32_t value)
{
	tmc5130_writeDatagram(tmc5130, address, BYTE(value, 3), BYTE(value, 2), BYTE(value, 1), BYTE(value, 0));
}

int32_t tmc5130_readInt(uint8_t tmc5130, uint8_t address)
{
	address = TMC_ADDRESS(address);

	uint8_t data[5] = { 0, 0, 0, 0, 0 };

	data[0] = address;
	tmc5130_readWriteArray(tmc5130, &data[0], 5);

	data[0] = address;
	tmc5130_readWriteArray(tmc5130, &data[0], 5);

	return (((int32_t)data[1] << 24) | ((int32_t)data[2] << 16) | ((int32_t)data[3] << 8) | ((int32_t)data[4]));
}