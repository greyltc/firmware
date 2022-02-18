#define VERSION_MAJOR 1
#define VERSION_MINOR 6
#define VERSION_PATCH 1
#define BUILD edc2568

// when DEBUG is defined, a serial comms interface will be brought up over USB to print out some debug info
//#define DEBUG

//#define NO_LED

// maximum velocity in microsteps/s
#define VMAX 200000

//[O-32] maximum current used while moving
#define RUN_CURRENT 14

//[O-32] maximum current used while resisting movement
#define HOLD_CURRENT 4

//maximum number of milliseonds any homing move could possibly take
#define MOVE_TIMEOUT 120000ul

// I2C timeouts
//#define I2C_TIMEOUT_US 25000ul // number in micros, 25ms
//#define I2C_TIMEOUT_US 100000ul // number in micros, 100ms
//#define SEND_LATER_TIMEOUT 250000ul // number in micros, give up on send later after waiting this long
//#define SEND_LATER_TIMEOUT 500000ul // number in micros, give up on send later after waiting this long

// for debugging
//#define MOVE_TIMEOUT 5000

// setup stall threshold
// 0 --> least difficult to stall
// 64 --> default value
// 127 --> most difficult to stall
#define STALL 64

// axis:address --> 1:0x50, 2:0x51, 3:0x52
#define I2C_SLAVE_ADDRESS 0x50

#include <Arduino.h>
#include <Wire.h>
#include <util/crc16.h>  // for _crc_xmodem_update
//#include <util/delay.h>
#include <avr/io.h>
#include <avr/wdt.h> /*Watchdog timer handling*/
extern "C" {
  #include "tmc/ic/TMC5130/TMC5130.h"
}

// create version string
#define STRINGIFY0(s) # s
#define STRINGIFY(s) STRINGIFY0(s)
#define FIRMWARE_VER "20220218.0.13"

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

// The following bits are OPTIBOOT stuff to allow us to know what MCUSR
// had before the bootloader wiped it out
/*
   First, we need a variable to hold the reset cause that can be written before
   early sketch initialization (that might change r2), and won't be reset by the
   various initialization code.
   avr-gcc provides for this via the ".noinit" section.
*/
uint8_t resetFlag __attribute__ ((section(".noinit")));

/*
   Next, we need to put some code to save reset cause from the bootload (in r2)
   to the variable.  Again, avr-gcc provides special code sections for this.
   If compiled with link time optimization (-flto), as done by the Arduno
   IDE version 1.6 and higher, we need the "used" attribute to prevent this
   from being omitted.
*/
void resetFlagsInit(void) __attribute__ ((naked))
__attribute__ ((used))
__attribute__ ((section (".init0")));
void resetFlagsInit(void)
{
  /*
     save the reset flags passed from the bootloader
     This is a "simple" matter of storing (STS) r2 in the special variable
     that we have created.  We use assembler to access the right variable.
  */
  __asm__ __volatile__ ("sts %0, r2\n" : "=m" (resetFlag) :);
}

void do_every_short_while(void);
int wait_for_move(int32_t timeout_ms = MOVE_TIMEOUT);
void receiveEvent(int);
void requestEvent(void);
void go_to (int32_t);
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
void reset_drive(void);
bool validate_message(int);
int do_isr_cmd(uint8_t);
int do_main_cmd(uint8_t);
int prep_response(int);
void reset(void);
void reset_reason(void);

volatile int32_t stage_length = 0; // this is zero when homing has not been done, -1 while homing
volatile uint32_t current_position = 0;  // keeps track of where we are now
volatile bool busy = false; // set this when we don't want to accept new movement commands
volatile uint8_t status; // status byte from the driver

// response buffer for i2c responses to requests from the master
#define RESP_BUF_LEN 24
uint8_t out_buf[RESP_BUF_LEN] = { 'f' };
volatile int bytes_in_out_buf = 0;

// command buffer
#define CMD_BUF_LEN 24
uint8_t in_buf[CMD_BUF_LEN] = { 0x00 };
uint8_t pending_in_buf[CMD_BUF_LEN] = { 0x00 };  // to store not yet processed ones
volatile uint8_t sequence_num = 0;  // the last sequence number we got from the master

// time to wait after powering up the drive
#define POWER_UP_DELAY 500

// allows for different channels
uint8_t TMC5130 = 0;

// we need to use shadow registers for some of the device registers
// because some of them are read only and they have mupliple fields
// we'll keep track of them ourselves here
uint32_t COOLCONF_shadow_reg;
uint32_t PWMCONF_shadow_reg;
uint32_t IHOLD_IRUN_shadow_reg;

#ifndef NO_LED
const unsigned int LED_PIN = 3; // arduino pin for alive LED
#endif

void setup() {
#ifndef NO_LED
  // setupLED pin
  digitalWrite(LED_PIN, LOW); // light off
  pinMode(LED_PIN, OUTPUT); // light enabled
#endif // NO_LED

  D(Serial.begin(115200));
  D(Serial.println("Setup started..."));

  reset_reason();

  // set watchdog timer for 8 seconds
  wdt_enable(WDTO_8S);

  // disable drive
  BIT_SET(ENN_PORT, ENN_BIT);
  PIN_MODE(ENN, OUTPUT);

  // use internal ramp generator
  BIT_CLEAR(SD_MODE_PORT, SD_MODE_BIT);
  PIN_MODE(SD_MODE, OUTPUT);

  spi_setup();
  reset_drive();
  
  // setup I2C
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.setWireTimeout(25000ul, true);
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event

  D(Serial.println("Setup complete."));
}


// short while duration
//5000 ~= 1 second (this might change if the main loop changes speed)
const uint32_t short_while_loops = 5000ul;

// how many loops we've done
uint32_t loop_counter = 0ul;

// for timeouts in wait for move
int32_t time0;

// main program loop
void loop() {
  loop_counter++;

  if (loop_counter%short_while_loops == 0){
    do_every_short_while();
  }

  // handle a command that can't be done in the ISR
  if (busy){
    bytes_in_out_buf = prep_response(do_main_cmd(pending_in_buf[0]));  // handle the command
    busy = false;
  } else {
    // if we're not handling a command, update the thing's position (and ststus byte)
    current_position = tmc5130_readInt(TMC5130, TMC5130_XACTUAL);
  }
}

// appends the sequence number
// then the a CRC to the n_bytes in out_buf and returns the total ready to send
int prep_response(int n_bytes){
  int i = 0;
  uint16_t crc = 0xffff;  // crc starting value
  if ((n_bytes > 0) && ((n_bytes+3) <= RESP_BUF_LEN)){
    out_buf[n_bytes++] = sequence_num;
    for (i=0; i<n_bytes; i++){
      crc = _crc_xmodem_update(crc, out_buf[i]);
    }
    out_buf[i++] = (crc >> 8) & 0xFF;
    out_buf[i++] = crc & 0xFF;
  }
  return(i);
}

void reset_drive(void){
  COOLCONF_shadow_reg = 0;
  PWMCONF_shadow_reg = 0x00050480;
  IHOLD_IRUN_shadow_reg = 0;

  BIT_SET(ENN_PORT, ENN_BIT);  // power down the drive
  delay(POWER_UP_DELAY); // give the driver a few ms to power off

  tmc5130_writeInt(TMC5130, TMC5130_XACTUAL, 0);
  tmc5130_writeInt(TMC5130, TMC5130_XTARGET, 0);

  tmc5130_writeInt(TMC5130, TMC5130_CHOPCONF, 0);
  tmc5130_writeInt(TMC5130, TMC5130_GCONF, 0);
  tmc5130_writeInt(TMC5130, TMC5130_PWMCONF, 0x00050480);
  tmc5130_writeInt(TMC5130, TMC5130_TPWMTHRS, 0);
  tmc5130_writeInt(TMC5130, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);

  tmc5130_writeInt(TMC5130, TMC5130_VSTART, 0);
  tmc5130_writeInt(TMC5130, TMC5130_A1, 1000);
  tmc5130_writeInt(TMC5130, TMC5130_V1, 0);  // was 50000. 0 disables A1 and V1 phases
  tmc5130_writeInt(TMC5130, TMC5130_AMAX, 1500);
  tmc5130_writeInt(TMC5130, TMC5130_VMAX, VMAX);
  tmc5130_writeInt(TMC5130, TMC5130_DMAX, 1500);
  tmc5130_writeInt(TMC5130, TMC5130_D1, 1000);
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

  // stallguard sg_stop on
  TMC5130_FIELD_WRITE(TMC5130, TMC5130_SWMODE, TMC5130_SW_SG_STOP_MASK, TMC5130_SW_SG_STOP_SHIFT, 1);

  // set up transition to something
  // higher numbers make stall detection less reliable
  // lower numbers mean stalling will only be turned on with faster movement
  tmc5130_writeInt(TMC5130, TMC5130_TCOOLTHRS, 700); // was 1000


  // speed threshold below which stealth is off
  tmc5130_writeInt(TMC5130, TMC5130_THIGH, 500);

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

// takes a signed int and returns one byte representing the
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

// takes command byte and does the thing if it should be done in the ISR
// returns number of bytes that should be transmitted out of out_buf
// sets in_buf[0] to 0x00 if it handled the command, otherwise leaves it
int do_isr_cmd(uint8_t cmd){
  int ret_bytes = 0;
  switch (cmd){
    case 't':  // reset the uc
      reset();
      break;
    case 'v':
      out_buf[0] = 'p';
      ret_bytes = 1;
      ret_bytes += sprintf(&out_buf[1], FIRMWARE_VER);
      ret_bytes++; // want null terminator
      in_buf[0] = 0x00;
      break;
    case 'r': // get stage pos
      out_buf[0] = 'p';
      memcpy(&out_buf[1], &current_position, 4);
      ret_bytes = 5;
      in_buf[0] = 0x00;
      break;
    case 'l': // get stage length
      out_buf[0] = 'p';
      memcpy(&out_buf[1], &stage_length, 4);
      ret_bytes = 5;
      in_buf[0] = 0x00;
      break;
    case 'e':  // get reset reason byte
      out_buf[0] = 'p';
      out_buf[1] = resetFlag;
      ret_bytes = 2;
      in_buf[0] = 0x00;
      break;
    case 's':  // get driver status byte
      out_buf[0] = 'p';
      out_buf[1] = status; //just send the last status we're aware of. if we're blocked it's probably being updated frequently
      ret_bytes = 2;
      in_buf[0] = 0x00;
      break;
    case 'd':  // power off drive
      estop(true);
      out_buf[0] = 'p';
      in_buf[0] = 0x00;
      break;
    case 'c':  // comms check
      out_buf[0] = 'p';
      ret_bytes = 1;
      in_buf[0] = 0x00;
      break;
    case 'h':  //home request (to be done later)
    case 'w':  //write drive register request (to be done later)
    case 'i':  //read drive register request (to be done later)
    case 'f':  //freewheel request (to be done later)
    case 'g':  //move request (to be done later)
    case 'a':  //jog request (to be done later)
    case 'b':  //jog request (to be done later)
      if (busy){
        out_buf[0] = 'f';  // send 'f' to reject a command because we're busy now
        in_buf[0] = 0x00;
      } else {
        if ((cmd == 'g') && (stage_length <= 0)){
          out_buf[0] = 'f';  // send 'f' to reject a goto command since we're unhomed
          in_buf[0] = 0x00;
        } else {
          out_buf[0] = 'p';  // send 'p' to ack that the command will be done (later)
        }
      }
      ret_bytes = 1;
      break;
    default:
      out_buf[0] = 'u'; // send 'u' becuase this is an unknown command
      ret_bytes = 1;
      in_buf[0] = 0x00;
  }
  return(ret_bytes);
}

int do_main_cmd(uint8_t cmd){
  int ret_bytes = 0;
  int32_t value;
  uint8_t address;
  switch (cmd){
    case 'h':
      stage_length = -1;
      stage_length = home();
      break;
    case 'f':
      freewheel(true);
      break;
    case 'a':
      jog(0);
      break;
    case 'b':
      jog(1);
      break;
    case 'i':  // read a driver register
      address = (uint8_t) pending_in_buf[1];
      value = tmc5130_readInt(TMC5130, address);
      memcpy(out_buf, &value, 4);
      ret_bytes = 4;
      D(Serial.print(F("Register 0x")));
      D(Serial.print(address, HEX));
      D(Serial.print(F(" is: 0x")));
      D(Serial.print(value, HEX));
      D(Serial.print(F(" (that's ")));
      D(Serial.print(value));
      D(Serial.println(F(" dec)")));
      break;
    case 'w':  // program a driver register
      address = (uint8_t) pending_in_buf[1];
      memcpy(&value, &pending_in_buf[2], 4);
      tmc5130_writeInt(TMC5130, address, value);
      D(Serial.print(F("Register 0x")));
      D(Serial.print(address, HEX));
      D(Serial.print(F(" becomes: 0x")));
      D(Serial.print(value, HEX));
      D(Serial.print(F(" (that's ")));
      D(Serial.print(value));
      D(Serial.println(F(" dec)")));
      break;
    case 'g':
      memcpy(&value, &pending_in_buf[1], 4);
      //go_to(*(int32_t*) &pending_in_buf[1]);
      go_to(value);
      break;
  }
  return(ret_bytes);
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
    current_position = tmc5130_readInt(TMC5130, TMC5130_XACTUAL);  // force update of the status byte
    this_status = status;
    if (this_status & BIT3){ // bit 3 is for standstill
      NOP; // we're standing still
    } else {
      break; // we're not standing still
    }
    elapsed = millis()-t0;
    wdt_reset(); // pet the dog
  } while( elapsed < move_start_timeout);

  // loop to wait for motion to end
  do{
    current_position = tmc5130_readInt(TMC5130, TMC5130_XACTUAL);  // force update of the status byte
    this_status = status;
    if (this_status & BIT3){ // bit 3 is for standstill
      break; // we're standing still
    }
    if (BIT_GET(ENN_PORT, ENN_BIT)){  // check if motor power has been removed
      break;
    }
    elapsed = millis()-t0;
    wdt_reset(); // pet the dog
  } while( elapsed < timeout_ms);

  if (BIT_GET(ENN_PORT, ENN_BIT)){  // check if motor power has been removed
    ret_code = 3;
  } else if (elapsed >= timeout_ms){
    ret_code = 2;
  } else if (this_status & BIT5) {
    ret_code = 0;
  } else if (this_status & BIT2) {  // somehow this bit seems to always be set...
    ret_code = 1;
  }
  return(ret_code);
}

// sends the stage somewhere
// returns false if the stage is stalled or if the movement is out of bounds
void go_to (int32_t target) {

  // this is the stall check
  //if (status & BIT2){
  //if (TMC5130_FIELD_READ(TMC5130, TMC5130_RAMPSTAT, TMC5130_STATUS_SG_MASK, TMC5130_STATUS_SG_SHIFT)){
  if (TMC5130_FIELD_READ(TMC5130, TMC5130_RAMPSTAT, TMC5130_EVENT_STOP_SG_MASK, TMC5130_EVENT_STOP_SG_SHIFT)){
    freewheel(true); // we're done
  } else if ( (target > 0) && (target <= stage_length) ) {
    // TODO: need to more aggressively limit travel to not go close to edges
    tmc5130_writeInt(TMC5130, TMC5130_XTARGET, target);
  }

  


    
    // the action of reading this field will clear a few bits in the register that maybe we don't care about
    
    //if (TMC5130_FIELD_READ(TMC5130, TMC5130_RAMPSTAT, TMC5130_STATUS_SG_MASK, TMC5130_STATUS_SG_SHIFT)){
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
    while(!(SPSR1 & (1<<SPIF1)));  // shift occurs here
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

  // load in the address
	data[0] = address;
	tmc5130_readWriteArray(tmc5130, &data[0], 5);

  // do the actual read
	data[0] = address;
	tmc5130_readWriteArray(tmc5130, &data[0], 5);

	return (((int32_t)data[1] << 24) | ((int32_t)data[2] << 16) | ((int32_t)data[3] << 8) | ((int32_t)data[4]));
}

// fires off whenever the master sends us bytes
// is called from an ISR in the Wire library
void receiveEvent(int how_many){
  int i;
  if (how_many > 0){
    for (i=0; (i<how_many) && (i<CMD_BUF_LEN); i++){
      in_buf[i] = Wire.read();
    }
    if (validate_message(i)) {
      //  BIT_SET(PORTD, LED_PIN);
      sequence_num = in_buf[i-3];
      // most stuff can be handled in the ISR
      bytes_in_out_buf = prep_response(do_isr_cmd(in_buf[0]));
      if (in_buf[0] != 0x00){  // will need to handle in main
        busy = true;  // set the busy flag because we'll need to do something in the main loop
        // copy over the buffer for cosumption in main
        memcpy(pending_in_buf, in_buf, CMD_BUF_LEN);
      }  // main loop processing check
    }  // valid message check
  }  // non-zero rx check
}

// fires off whenever when the master asks for bytes
// is called from an ISR in the Wire library
void requestEvent(void){
  if ((bytes_in_out_buf > 0)){
    Wire.write((uint8_t*)out_buf, bytes_in_out_buf);
    bytes_in_out_buf = 0;
  }
}

// returns true if a CRC checks out
bool validate_message(int n_bytes){
  bool ret = false;
  int i;
  uint16_t crc = 0xffff;  // crc starting value

  for (i=0; i<n_bytes; i++){
    crc = _crc_xmodem_update(crc, in_buf[i]);
  }
  if (crc == 0){
    ret = true;
  }
  return(ret);
}

// gets run once per short while
void do_every_short_while(void){
  wdt_reset();  // pet the dog
#ifndef NO_LED
  //toggle the alive LED pin
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
#endif // NO_LED
#ifdef DEBUG
  if (Wire.getWireTimeoutFlag()){
    Serial.println(F("We had a wire library timeout!"));
    Wire.clearWireTimeoutFlag();
  }
#endif // DEBUG
}

// asks the dog to reset the uc (takes 15ms)
void reset(void) {
  wdt_enable(WDTO_15MS);
  while(true);
}

// reset reason detection
void reset_reason(void){
  // three quick blinks if bit by dog
  if (resetFlag & (0x01<<WDRF)){
    D(Serial.println(F("Got reset by the WDT.")));
#ifndef NO_LED
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
#endif // DEBUG
    NOP;
  }
}
