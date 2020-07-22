#define VERSION_MAJOR 1
#define VERSION_MINOR 1
#define VERSION_PATCH 1
#define BUILD d44d9f3

//#define DEBUG
//#define NO_LED

// maximum velocity in microsteps/s
#define VMAX 200000

//[O-32] maximum current used while moving
#define RUN_CURRENT 14

//[O-32] maximum current used while resisting movement
#define HOLD_CURRENT 4

//maximum number of milliseonds any homing move could possibly take
#define MOVE_TIMEOUT 120000

// I2C timeouts
#define I2C_TIMEOUT_US 25000ul // number in micros, 25ms
//#define I2C_TIMEOUT_US 100000ul // number in micros, 100ms
#define SEND_LATER_TIMEOUT 250000ul // number in micros, give up on send later after waiting this long
//#define SEND_LATER_TIMEOUT 500000ul // number in micros, give up on send later after waiting this long

// for debugging
//#define MOVE_TIMEOUT 5000

// setup stall threshold
// 0 --> least difficult to stall
// 64 --> default value
// 127 --> most difficult to stall
#define STALL 64

// axis:address --> 1:0x50, 2:0x51, 3:0x52
#define I2C_SLAVE_ADDRESS 0x51

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
#define BIT_GET(VALUE, BIT_POSITION) (((VALUE) & ( 1 << (BIT_POSITION) )) >> (BIT_POSITION))

#define PIN_MODE(PIN_NAME, IO_MODE) (PIN_MODE_ ## IO_MODE(PIN_NAME ## _DDR, PIN_NAME ## _BIT))
#define PIN_MODE_OUTPUT(DDR, BIT_POSITION) BIT_SET(DDR, BIT_POSITION)
#define PIN_MODE_INPUT(DDR, BIT_POSITION) BIT_CLEAR(DDR, BIT_POSITION)

// no operation
#define NOP __asm__ __volatile__ ("nop\n\t")

// debug trick
#ifdef DEBUG
#  define D(x) (x)
#else
#  define D(x) do{}while(0)
#endif // DEBUG

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


int wait_for_move(int32_t timeout_ms = MOVE_TIMEOUT);
void receiveEvent(int);
bool requestEvent(void);
int handle_cmd(bool);
bool go_to (int32_t);
int32_t home(void);
void jog(int);
void spi_setup(void);
void tmc5130_readWriteArray(uint8_t , uint8_t *, size_t );

void tmc5130_writeDatagram(uint8_t, uint8_t , uint8_t , uint8_t , uint8_t , uint8_t );
int32_t tmc5130_readInt(uint8_t, uint8_t );
void tmc5130_writeInt(uint8_t, uint8_t , int32_t );
uint8_t rescale_sgt(uint8_t);
uint8_t seven_bit_twos_c(int);

void clear_stall(void);
void freewheel(bool);
void estop(bool);

volatile int32_t stage_length = 0; // this is zero when homing has not been done, -1 while homing
volatile bool blocked = false; // set this when we don't want to accept new movement commands
volatile bool send_later = false; // the master has asked for bytes but we have none ready
volatile uint32_t send_later_t0; // timer to keep track of how long it's been since send_later was set (in micros)
volatile uint8_t status; // status byte from the driver

// response buffer for i2c responses to requests from the master
#define RESP_BUF_LEN 20
static uint8_t out_buf[RESP_BUF_LEN] = { 'f' };
volatile int bytes_ready = 0;
//uint32_t send_later_timeout = 250000ul; // 250 ms

// command buffer
#define CMD_BUF_LEN 20
volatile uint8_t cmd_byte = 0x00;
static uint8_t in_buf[CMD_BUF_LEN] = { 0x00 };

// time to wait after powering up the drive
#define POWER_UP_DELAY 500

// allows for different channels
uint8_t TMC5130 = 0;

// we need to use shadow registers for some of the device registers
// because some of them are read only and they have mupliple fields
// we'll keep track of them ourselves here
uint32_t COOLCONF_shadow_reg = 0;
uint32_t PWMCONF_shadow_reg = 0x00050480;
uint32_t IHOLD_IRUN_shadow_reg = 0;

void setup() {

  // disable drive
  BIT_SET(ENN_PORT, ENN_BIT);
  PIN_MODE(ENN, OUTPUT);

  // use internal ramp generator
  BIT_CLEAR(SD_MODE_PORT, SD_MODE_BIT);
  PIN_MODE(SD_MODE, OUTPUT);

  spi_setup();

  D(Serial.begin(115200));

  tmc5130_writeInt(TMC5130, TMC5130_XACTUAL, 0);
  tmc5130_writeInt(TMC5130, TMC5130_XTARGET, 0);

  tmc5130_writeInt(TMC5130, TMC5130_CHOPCONF, 0);
  tmc5130_writeInt(TMC5130, TMC5130_GCONF, 0);
  tmc5130_writeInt(TMC5130, TMC5130_PWMCONF, 0x00050480);
  tmc5130_writeInt(TMC5130, TMC5130_TPWMTHRS, 0);
  tmc5130_writeInt(TMC5130, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);

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

  TMC5130_FIELD_WRITE(TMC5130, TMC5130_GCONF, TMC5130_EN_PWM_MODE_MASK, TMC5130_EN_PWM_MODE_SHIFT, 1);
  TMC5130_FIELD_WRITE(TMC5130, TMC5130_GCONF, TMC5130_I_SCALE_ANALOG_MASK, TMC5130_I_SCALE_ANALOG_SHIFT, 1);
  TMC5130_FIELD_WRITE(TMC5130, TMC5130_GCONF, TMC5130_ENC_COMMUTATION_MASK, TMC5130_ENC_COMMUTATION_SHIFT, 0);

  PWMCONF_shadow_reg = FIELD_SET(PWMCONF_shadow_reg, TMC5130_PWM_AMPL_MASK, TMC5130_PWM_AMPL_SHIFT, 128);
  //PWMCONF_shadow_reg = FIELD_SET(PWMCONF_shadow_reg, TMC5130_PWM_GRAD_MASK, TMC5130_PWM_GRAD_SHIFT, 1);
  //PWMCONF_shadow_reg = FIELD_SET(PWMCONF_shadow_reg, TMC5130_PWM_FREQ_MASK, TMC5130_PWM_FREQ_SHIFT, 0);
  PWMCONF_shadow_reg = FIELD_SET(PWMCONF_shadow_reg, TMC5130_PWM_AUTOSCALE_MASK, TMC5130_PWM_AUTOSCALE_SHIFT, 1);
  PWMCONF_shadow_reg = FIELD_SET(PWMCONF_shadow_reg, TMC5130_FREEWHEEL_MASK, TMC5130_FREEWHEEL_SHIFT, 1);
  tmc5130_writeInt(TMC5130, TMC5130_PWMCONF, PWMCONF_shadow_reg); // write the shadow register

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
  // higher numbers make stall detection less reliable
  // lower numbers mean stalling will only be turned on with faster movement
  tmc5130_writeInt(TMC5130, TMC5130_TCOOLTHRS, 300); // was 1000

	COOLCONF_shadow_reg = FIELD_SET(COOLCONF_shadow_reg, TMC5130_SGT_MASK, TMC5130_SGT_SHIFT, rescale_sgt(STALL));
  COOLCONF_shadow_reg = FIELD_SET(COOLCONF_shadow_reg, TMC5130_SEMIN_MASK, TMC5130_SEMIN_SHIFT, 2);
  tmc5130_writeInt(TMC5130, TMC5130_COOLCONF, COOLCONF_shadow_reg);

  // set current position to zero, then set target to where we are now
  tmc5130_writeInt(TMC5130, TMC5130_XACTUAL, 0);
  tmc5130_writeInt(TMC5130, TMC5130_XTARGET, tmc5130_readInt(TMC5130, TMC5130_XACTUAL));

  // current config
  IHOLD_IRUN_shadow_reg = FIELD_SET(IHOLD_IRUN_shadow_reg, TMC5130_IRUN_MASK, TMC5130_IRUN_SHIFT, RUN_CURRENT); // set run current [0,32]
  IHOLD_IRUN_shadow_reg = FIELD_SET(IHOLD_IRUN_shadow_reg, TMC5130_IHOLD_MASK, TMC5130_IHOLD_SHIFT, HOLD_CURRENT); // set hold/idle current [0,32]
  // set rampdown time from run to hold current [0,15], 4 might give 1 second
  IHOLD_IRUN_shadow_reg = FIELD_SET(IHOLD_IRUN_shadow_reg, TMC5130_IHOLDDELAY_MASK, TMC5130_IHOLDDELAY_SHIFT, 4);
  tmc5130_writeInt(TMC5130, TMC5130_IHOLD_IRUN, IHOLD_IRUN_shadow_reg); // write shadow reg
  
  // reset encoder position to zero
  tmc5130_writeInt(TMC5130, TMC5130_XENC, 0);

  // power up drive by releasing hardware line
  BIT_CLEAR(ENN_PORT, ENN_BIT);

  delay(POWER_UP_DELAY); // give the driver a few ms to reach equlibrium

  // read these registers to clear any error conditions from the boot up/power up / power cycle
  tmc5130_readInt(TMC5130, TMC5130_GSTAT);
  tmc5130_readInt(TMC5130, TMC5130_RAMPSTAT);
  tmc5130_readInt(TMC5130, TMC5130_ENC_STATUS);

  // boot into freewheel mode
  //estop(true); 
  freewheel(true);

#ifndef NO_LED
  // setupLED pin
  digitalWrite(LED_PIN, LOW); // light off
  pinMode(LED_PIN, OUTPUT); // light enabled
#endif // NO_LED
  
  // setup I2C
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.setWireTimeoutUs(I2C_TIMEOUT_US, true);
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event

  D(Serial.println("Setup complete."));

  digitalWrite(LED_PIN, LOW);
}

#ifndef NO_LED
uint32_t led_loops = 5000ul;
#endif // NO_LED

// how many loops we've done
uint32_t num_loops = 0ul;

int rdy_to_send; // to store how many bytes are ready to send up to the master

int32_t time0;
bool timesup = false;
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
    timesup = false;
    if (rdy_to_send > 0){
      time0 = millis();
      while (!send_later){ // wait for the master to ask us for bytes
        NOP;
        if((millis() - time0) > 500){
          timesup = true;
          break;
        }
      }
      send_later = false;
      if (!timesup){
        Wire.write((uint8_t*)out_buf, rdy_to_send);
        Wire.flush();
      }
    }
  }

  // give up on clock stretch
  if ((send_later) && (SEND_LATER_TIMEOUT < (micros() - send_later_t0))){
    send_later = false;
    Wire.write(0x00);
    Wire.flush();
  }

// #ifdef DEBUG // bouncy motion testing/debugging
//   int move_result;
//   // do a bounce routine
//   if (stage_length > 0) {
//     Serial.println(tmc5130_readInt(TMC5130, TMC5130_XENC));
//     go_to(stage_length/2 + stage_length/8);
//     move_result = wait_for_move();
//     if (move_result != 0){
//       Serial.print("Warning: Movement terminated with code: ");
//       Serial.println(move_result);
//     }
//     Serial.println(tmc5130_readInt(TMC5130, TMC5130_XENC));
//     go_to(stage_length/2 - stage_length/8);
//     move_result = wait_for_move();
//     if (move_result != 0){
//       Serial.print("Warning: Movement terminated with code: ");
//       Serial.println(move_result);
//     }
//   }
// #endif // DEBUG
}

// clears stall (and other error status).
// sets target position to current position
// sets positionoing mode
void clear_stall(void){
  tmc5130_writeInt(TMC5130, TMC5130_XTARGET, tmc5130_readInt(TMC5130, TMC5130_XACTUAL));
  tmc5130_writeInt(TMC5130, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);

  // read all the R+C registers
  tmc5130_readInt(TMC5130, TMC5130_GSTAT);
  tmc5130_readInt(TMC5130, TMC5130_RAMPSTAT);
  tmc5130_readInt(TMC5130, TMC5130_ENC_STATUS);
}

void freewheel(bool fw){
  if (fw){
    stage_length = 0; // we are now unhomed
    IHOLD_IRUN_shadow_reg = FIELD_SET(IHOLD_IRUN_shadow_reg, TMC5130_IHOLD_MASK, TMC5130_IHOLD_SHIFT, 0); // hold current to zero
    tmc5130_writeInt(TMC5130, TMC5130_IHOLD_IRUN, IHOLD_IRUN_shadow_reg); // write shadow reg
    tmc5130_writeInt(TMC5130, TMC5130_VMAX, 0); // disable movement
  } else {
    IHOLD_IRUN_shadow_reg = FIELD_SET(IHOLD_IRUN_shadow_reg, TMC5130_IHOLD_MASK, TMC5130_IHOLD_SHIFT, HOLD_CURRENT); // hold current to default
    tmc5130_writeInt(TMC5130, TMC5130_IHOLD_IRUN, IHOLD_IRUN_shadow_reg); // write shadow reg
    tmc5130_writeInt(TMC5130, TMC5130_VMAX, VMAX); // enable movement
  }
}

// issuing this command during a home or a jog should cause
// wait_for_move to take the full timeout to recover
// estop(false) can not be called from the ISR (only true can)
void estop(bool stop){
  if (stop){
    BIT_SET(ENN_PORT, ENN_BIT); // power off driver
    stage_length = 0; // we are now unhomed
  } else {
    BIT_CLEAR(ENN_PORT, ENN_BIT); // power on driver
    delay(POWER_UP_DELAY);
    clear_stall();
  }
}

//tages a signed int and returns one byte representing the
// 7 bit two's compliment value of the number.
// maximum = 63, minimum = -64. anything above or below those will get pegged to the limit
uint8_t seven_bit_twos_c(int i){
  uint8_t out = 0x00;
  if (i>63){
    i = 63;
  }
  if (i<-64){
    i = -64;
  }
  if (i<0){
    i *= -1;
    out = (uint8_t)i;
    out = ~out;
    out += 1;
    out &= 0b01111111;
  } else {
    out = i;
  }
  return (out);
}

// rescales stallguard threshold config value to be on the interval [0,127]
// takes an takes an input, clips it to 127 and generates an output for the config register
uint8_t rescale_sgt(uint8_t in){
  int i;
  if (in > 127){
    i = 127;
  } else {
    i = in;
  }
  return(seven_bit_twos_c(i-64));
}


// processes commands from cmd_buf and cmd_byte
// slo mode indicates that commands which requre "slow" processing should be handled
// where "slow" is any processing that requires delays, external access or anything beyond simple variable lookups
// slo_mode is true when called from the main loop and false when called from an ISR
// places response bytes (if any) into out_buf and returns number of bytes it put there

// command|meaning|number of bytes to expect in response
// g|go to|1
// c|comms check|5
// h|home|1
// d|power off drive|1
// f|enter freewheel mode|1
// l|get stage length|5
// s|get status byte|2
// r|read back current position|5
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
        stage_length = -1;
        if(slo_mode){  // processing in main loop, not ISR
          cmd_byte = 0x00; // clear the cmd _byte. it has been handled
          blocked = true; // block duiring home
          stage_length = home();
          blocked = false;
        } else { // processing in ISR, not main loop.
          // accept the homing command, and respond immediately to the master
          // but don't actually execute the homing procedure here
          // instead we leave that to be done in the main loop later
          out_buf[0] = 'p';
          rdy_to_send = 1;
        }
      }
      break;

    case 'a': // jog direction a command
    case 'b': // jog direction b command
      if(blocked){
        out_buf[0] = 'f';
        cmd_byte = 0x00; // clear the cmd _byte. it has been handled
        rdy_to_send = 1;
      } else { //not blocked
        stage_length = -1;
        if(slo_mode){  // processing in main loop, not ISR
          cmd_byte = 0x00; // clear the cmd _byte. it has been handled
          blocked = true; // block duiring jog
          if (this_cmd == 'a'){
            jog(0); // jog in direction 0 (same as initial homing direction)
          } else {
            jog(1); // jog in direction 1 (opposite to that of initial homing direction)
          }
          stage_length = 0;
          blocked = false;
        } else { // processing in ISR, not main loop.
          // accept the jogging command, and respond immediately to the master
          // but don't actually execute the jogging procedure here
          // instead we leave that to be done in the main loop later
          out_buf[0] = 'p';
          rdy_to_send = 1;
        }
      }
      break;

    case 'f': // enter freewheel mode (drive can kinda freely spin)
      if(blocked){
        out_buf[0] = 'f';
        cmd_byte = 0x00; // clear the cmd _byte. it has been handled
        rdy_to_send = 1;
      } else { //not blocked
        if(slo_mode){  // processing in main loop, not ISR
          freewheel(true);
          out_buf[0] = 'p'; // press p for pass
          cmd_byte = 0x00; // clear the cmd _byte. it has been handled
          rdy_to_send = 1;
        }
      }
      break;

    case 'd': // disable drive
      estop(true);
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

    case 's': // status command
      if(blocked){
        out_buf[0] = 'p';
        out_buf[1] = status; //just send the last status we're aware of. if we're blocked it's probably being updated frequently
        //D(Serial.println(status, BIN));
        //D(Serial.println(tmc5130_readInt(TMC5130, TMC5130_DRVSTATUS), BIN));
        cmd_byte = 0x00; // clear the cmd _byte. it has been handled
        rdy_to_send = 2;
      } else { //not blocked
        if(slo_mode){  // processing in main loop, not ISR
          out_buf[0] = 'p'; // press p for pass
          blocked = true; // block duiring postition register read
          tmc5130_readInt(TMC5130, TMC5130_DRVSTATUS); // force update of the status byte
          blocked = false;
          out_buf[1] = status;
          cmd_byte = 0x00; // clear the cmd _byte. it has been handled
          rdy_to_send = 2;
        }
      }
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
      out_buf[0] = 'u'; // press u for unknown command
      cmd_byte = 0x00; // clear the cmd_byte. it has been handled
      rdy_to_send = 1;
  } // end switch
  return(rdy_to_send);
}

// waits for a move to complete and returns a reason for why it completed
// -1 programming error. movement completed for unhandled reason
// 0 completed because target position reached
// 1 completed because there was a stall
// 2 exited because of a timeout (was moving for too long)
// 3 exited because we noticed the motor driver power has been removed
int wait_for_move(int32_t timeout_ms){ //timeout_ms defaults to MOVE_TIMEOUT
  int32_t move_start_timeout = 500; // let's wait up to this long for the motion to begin
  int32_t t0 = millis();
  int32_t elapsed = 0;
  int ret_code = -1;
  uint8_t this_status = status; //store this so that it doesn't get changed on us unexpectedly

  // loop to wait for motion to begin
  do{
    tmc5130_readInt(TMC5130, TMC5130_DRVSTATUS); // force update of the status byte
    this_status = status;
    if (this_status & BIT3){ // bit 3 is for standstill
      NOP; // we're standing still
    } else {
      break; // we're not standing still
    }
    elapsed = millis()-t0;
  } while( elapsed < move_start_timeout);

  // loop to wait for motion to end
  do{
    tmc5130_readInt(TMC5130, TMC5130_DRVSTATUS); // force update of the status byte
    this_status = status;
    if (this_status & BIT3){ // bit 3 is for standstill
      break; // we're standing still
    }
    if (BIT_GET(ENN_PORT, ENN_BIT)){
      break;
    }
    elapsed = millis()-t0;
  } while( elapsed < timeout_ms);

  if (BIT_GET(ENN_PORT, ENN_BIT)){
    ret_code = 3;
  } else if (elapsed >= timeout_ms){
    ret_code = 2;
  } else if (this_status & BIT5) {
    ret_code = 0;
  } else if (this_status & BIT2) { // somehow this bit seems to always be set...
    ret_code = 1;
  }
  return(ret_code);
}

// sends the stage somewhere
// returns false if the stage is stalled or if the movement is out of bounds
bool go_to (int32_t target) {
  bool return_code = false;
  // TODO: need to more aggressively limit travel to not go close to edges
  if ( (target > 0) && (target <= stage_length)) {
    tmc5130_writeInt(TMC5130, TMC5130_XTARGET, target);

    // this is the stall check
    // the action of reading this field will clear a few bits in the register that maybe we don't care about
    //if (status & BIT2){
    //if (TMC5130_FIELD_READ(TMC5130, TMC5130_RAMPSTAT, TMC5130_STATUS_SG_MASK, TMC5130_STATUS_SG_SHIFT)){
    if (TMC5130_FIELD_READ(TMC5130, TMC5130_RAMPSTAT, TMC5130_EVENT_STOP_SG_MASK, TMC5130_EVENT_STOP_SG_SHIFT)){
      freewheel(true); // we're done
    } else {
      return_code = true;
    }
  }
  return (return_code);
}

// this is basically half the homing procedure only with configuratble direction
// noteably it can not perform a proper back off from the end once it hits it because
// i can't know the stage length. this function should be used in only very special cases
// because it leaves teh sled in a position where it could possibly be driven in the wrong
// direction without the ability to stall correctly
void jog(int dir){
  D(Serial.println("Jogging started..."));

  estop(false); // in case we were in emergency stop, re-enable
  freewheel(false); // in case freewheel was on, disable it
  wait_for_move(); // just in case some other movement was going on, let's wait for that to finish
  clear_stall();  // and in case that movement caused a stall, let's clear it

  if (dir == 0){ // this is the same direction that the motor starts moving for the home command
    // move in the negative direction
    tmc5130_writeInt(TMC5130, TMC5130_RAMPMODE, TMC5130_MODE_VELNEG);
  } else if (dir == 1){
    // move in the positive direction
    tmc5130_writeInt(TMC5130, TMC5130_RAMPMODE, TMC5130_MODE_VELPOS);
  } else {
    return;
  }

  // don't actually care why the move ended, really
  wait_for_move();

  // i'm not even going to clear the stall here because the next action has to be a home

  D(Serial.println("Jogging completed."));

  return;
}

// homing/stage length measurement task
int32_t home(void){
  int32_t measured_len = 0;
  D(Serial.println("Homing started..."));
  int move_result;

  estop(false); // in case we were in emergency stop, re-enable
  freewheel(false); // in case freewheel was on, disable it
  wait_for_move(); // just in case some other movement was going on, let's wait for that to finish
  clear_stall();  // and in case that movement caused a stall, let's clear it
  
  // move in the negative direction
  tmc5130_writeInt(TMC5130, TMC5130_RAMPMODE, TMC5130_MODE_VELNEG);
  move_result = wait_for_move();
  if (move_result == 2){ // wait for the stall
    return (0); // homing has failed because movement timed out before hitting an end
  }

  // define zero (home) by writing it into the driver's registers
  tmc5130_writeInt(TMC5130, TMC5130_XACTUAL, 0);
  tmc5130_writeInt(TMC5130, TMC5130_XENC, 0);
  clear_stall();

  // move in the positive direction
  tmc5130_writeInt(TMC5130, TMC5130_RAMPMODE, TMC5130_MODE_VELPOS);
  move_result = wait_for_move();
  if (move_result == 2){ // wait for the stall
    return (0); // homing has failed because movement timed out before hitting an end
  }

  // record the stage length
  measured_len = tmc5130_readInt(TMC5130, TMC5130_XACTUAL);
  clear_stall();

  // back off from the end of the stage by 10% of the stage's length
  tmc5130_writeInt(TMC5130, TMC5130_XTARGET, measured_len*0.9);
  
  move_result = wait_for_move();
  if (move_result != 0){ // wait for the target position to be reached
    return (0); // homing has failed because we were unable to complete the backoff procedure
  }
  clear_stall();

  D(Serial.println("Homing completed."));
  return (measured_len);
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
  status = data[0]; // update status byte
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

// ===  these next three functions are modded from tmc provided code  ===
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
    send_later_t0 = micros(); // record the time
    // which will be ended by the call to Wire.flush() from the main loop
  }
  return(send_later);
}
