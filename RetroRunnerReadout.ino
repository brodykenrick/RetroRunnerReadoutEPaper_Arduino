

//*****************************************************************************
//**************************** <INCLUDES> *************************************
#include "RetroRunnerReadout.h"


/* Retro Runner Readout
http://retrorunnerreadout.blogspot.com
Copyright 2013 Brody Kenrick.

Interfacing of Garmin ANT+ device (via a cheap Nordic nRF24AP UART module) to an Arduino and an 8 character 7-segment display module (using MAX7219).

The code runs through a loop of strings. Each of these are displayed on the 7-segment display. Some of these
strings are dynamically replaced with other strings. The dynamic strings consist of different groups. Some are
names of friends/supporters. Others are motivational/funny quotes. A further group are information about the run
taking place -- these include distance expected today, distance so far today, distance left, strides taken, current
heart rate etc.


Hardware
An Arduino Pro Mini 3v3
This nRF24AP2 module : http://www.goodluckbuy.com/nrf24ap2-networking-module-zigbee-module-with-ant-transceiver-.html

Either Display7S:
This 7-segment display : http://dx.com/p/8-segment-led-display-board-module-for-arduino-147814
Batteries, a 5V regulator and a level converter (http://www.freetronics.com/products/logic-level-converter-module)

Or DisplayEP:
EA 2.7" Screen
Batteries -- no extra regulator or level converters :)

The connector on nRF24AP2 board is (looking from the front, pin 1 is marked []):
[]GND(=VSS) | VDD(=3.3 volts)
UART_TX   | UART_RX
!(SUSP)   | SLEEP
RTS       | !(RESET)

Wiring to the Arduino Pro Mini 3v3 can be seen in 'antplus' below.

Display7S
The display board is labelled ant the connections are as follows:
MAX7219/7221 -> Arduino:Pin
  DIN       -> MOSI:11    (Arduino output)
  CLK       -> SCK:13     (Arduino output)
  LOAD/#CS  -> SS:10      (Arduino output)

Wiring to the Arduino Pro Mini 3v3 can be seen in 'mydisplay' below.

DisplayEP
TBD
*/

#include <Arduino.h>

#define NDEBUG
#define __ASSERT_USE_STDERR
#include <assert.h>

//#define USE_DISPLAY_7SEGMENT
#define USE_DISPLAY_EPAPER
#define USE_LOGO
#define USE_ANT

//Only one of these....
//#define USE_ANT_HRM
#define USE_ANT_SDM


//#define EPAPER_HARDCODED_TEMP_CELSIUS (21)


//#define USE_NARCOLEPTIC_DELAY //<! Use Narcoleptic to save battery with power down sleep (vs. busy delay())
//NOTE: This won't work with ANT+ as it uses serial. Software serial requires a fully operating system. Hardware serial will wake the system if in IDLE.
//TODO: Need to rework and use IDLE mode and HardwareSerial for the ANTPlus instead...
//#define USE_NARCOLEPTIC_DISABLE //<! Use Narcoleptic to save some battery with disabling certain items but not use narco delay....

//#define ANTPLUS_ON_HW_UART //!< H/w UART (i.e. Serial) instead of software serial. NOTE: There seems to be issues in not getting as many broadcast packets when using hardware serial (likely related to SPI and interrupt servicing taking too long).........


#if (defined(USE_NARCOLEPTIC_DELAY) || defined(USE_NARCOLEPTIC_DISABLE))
//#include <Narcoleptic.h>
#error "If you want this uncomment it: Commented this include as defines don't stop the Arduino IDE"
#endif

#if !defined(ANTPLUS_ON_HW_UART)
#include <SoftwareSerial.h>
#endif

#if defined(USE_DISPLAY_7SEGMENT)
//#include <LedControl.h>
#error "If you want this uncomment it: Commented this include as defines don't stop the Arduino IDE"
#endif //defined(USE_DISPLAY_7SEGMENT)

#if defined(USE_DISPLAY_EPAPER)
#include <SPI.h>
#include <EPD.h>

#if !defined(EPAPER_HARDCODED_TEMP_CELSIUS)
//Temperature sensor
#include <Wire.h>
#include <LM75A.h>
//#error "If you want this uncomment it: Commented this include as defines don't stop the Arduino IDE"
#endif //defined(EPAPER_HARDCODED_TEMP_CELSIUS)

#include <Adafruit_GFX.h>



// Change this for different display size
// supported sizes: 144 200 270
//#define SCREEN_SIZE 0
#define SCREEN_SIZE 270 //BK


#define CANTOO_LOGO_SUBSAMPLED
#define LOGO_INSERT_MESSAGE_COUNT (8) //!< Each X messages we insert the logo.

#if defined(CANTOO_LOGO_SUBSAMPLED)
#define IMAGE_CANTOO_LOGO  CanToo_bw_sample_h2_v2
#define CANTOO_LOGO_SUBSAMPLED_PARAM (true)
#else
#define IMAGE_CANTOO_LOGO  CanToo_bw
#define CANTOO_LOGO_SUBSAMPLED_PARAM (false)
#endif //defined(CANTOO_LOGO_SUBSAMPLED)

#if (SCREEN_SIZE == 144)
#define EPD_SIZE EPD_1_44
#define FILE_SUFFIX _1_44.xbm
#define NAME_SUFFIX _1_44_bits
#define HEIGHT_SUFFIX _1_44_height
#define WIDTH_SUFFIX  _1_44_width
#error "Not supported yet. Change/extend...."
#define EPD_WIDTH (-1)
#define EPD_HEIGHT (-1)

#elif (SCREEN_SIZE == 200)

#define EPD_SIZE EPD_2_0
#define FILE_SUFFIX _2_0.xbm
#define NAME_SUFFIX _2_0_bits
#define HEIGHT_SUFFIX _2_0_height
#define WIDTH_SUFFIX  _2_0_width
#error "Not supported yet. Change/extend...."
#define EPD_WIDTH (-1)
#define EPD_HEIGHT (-1)

#elif (SCREEN_SIZE == 270)

#define EPD_SIZE EPD_2_7
#define FILE_SUFFIX _2_7.xbm
#define NAME_SUFFIX _2_7_bits
#define HEIGHT_SUFFIX _2_7_height
#define WIDTH_SUFFIX  _2_7_width
#define EPD_WIDTH (264)
#define EPD_HEIGHT (176)

#else
#error "Unknown EPB size: Change the #define SCREEN_SIZE to a supported value"
#endif



//Note: This include is affected by EMBEDDED_ARTISTS define
#include <EPD_GFX.h>

//! Height in pixels of each segment in the EPD buffer. Proportional to the memory used and inversely proportional to processing to display. Also has visual impact and adds some delay in rendering.
//!If your Arduino hangs at startup reduce this (BUT must be a factor of the screen size -- you can start with 1).
//#define HEIGHT_OF_SEGMENT (11)
//#define HEIGHT_OF_SEGMENT (4)
//#define HEIGHT_OF_SEGMENT (8)
//#define HEIGHT_OF_SEGMENT (16)
#define HEIGHT_OF_SEGMENT (22)

// pre-processor convert to string
#define MAKE_STRING1(X) #X
#define MAKE_STRING(X) MAKE_STRING1(X)

// other pre-processor magic
// token joining and computing the string for #include
#define ID(X) X
#define MAKE_NAME1(X,Y) ID(X##Y)
#define MAKE_NAME(X,Y) MAKE_NAME1(X,Y)
#define MAKE_JOIN(X,Y) MAKE_STRING(MAKE_NAME(X,Y))

// calculate the include name and variable names
#define IMAGE_CANTOO_LOGO_FILE MAKE_JOIN(IMAGE_CANTOO_LOGO,FILE_SUFFIX)
#define IMAGE_CANTOO_LOGO_BITS MAKE_NAME(IMAGE_CANTOO_LOGO,NAME_SUFFIX)
#define IMAGE_CANTOO_LOGO_HEIGHT MAKE_NAME(IMAGE_CANTOO_LOGO,HEIGHT_SUFFIX)
#define IMAGE_CANTOO_LOGO_WIDTH MAKE_NAME(IMAGE_CANTOO_LOGO,WIDTH_SUFFIX)

// Add Images library to compiler path
#include <Images.h>  // this is just an empty file

//PROGMEM const
//Tweaked CanToo logos to be progmem (wasn't good messing around with all the settings on the include)
#include IMAGE_CANTOO_LOGO_FILE

#endif //defined(USE_DISPLAY_EPAPER)

#if defined(USE_ANT)
#include <ANTPlus.h>
#endif //defined(USE_ANT)

#define USE_SERIAL_CONSOLE //!<Use the hardware serial as the console. This needs to be off if using hardware serial for driving the ANT+ module.
#define CONSOLE_BAUD_RATE (115200)

//#define DEBUG_MODE //<! A debugging mode that does some things including making the strings displayed sequential (instead of random).

//#define DEBUG_OVERLAY //<! For e-paper overlay a few text/stats on each text screen



#if defined(USE_DISPLAY_7SEGMENT)
#define DISPLAY_DURATION_MS (1200)  //!< This is the display time for 8 characters. Scrolling takes longer (not quite linear increase).
#define DISPLAY_DURATION_STARTUP_MS (DISPLAY_DURATION_MS)
#define DELAY_BETWEEN_DISPLAYS_MS (1750) //<! Duration to have screen shut down between displaying messages
#define DISPLAY_INTENSITY (15) //<! 0..15
#endif //defined(USE_DISPLAY_7SEGMENT)


#if defined(USE_DISPLAY_EPAPER)

#if defined(DEBUG_MODE)
#define DISPLAY_DURATION_MS (3 * 1000)  //!< This is the time we pause with content on the screen.
#else
#define DISPLAY_DURATION_MS (10 * 1000)  //!< This is the time we pause with content on the screen.
#endif //defined(DEBUG_MODE)

#define DISPLAY_DURATION_STARTUP_MS (DISPLAY_DURATION_MS/3)  //!< This is the time we pause with content on the screen.

#else
//FIXME

#define DISPLAY_DURATION_MS (3 * 1000)  //!< This is the time we pause with content on the screen.
#define DISPLAY_DURATION_STARTUP_MS (DISPLAY_DURATION_MS/3)  //!< This is the time we pause with content on the screen.
#define DELAY_BETWEEN_DISPLAYS_MS (0) //<! Duration to have screen shut down between displaying messages

#endif //defined(USE_DISPLAY_EPAPER)


#if defined(NDEBUG) || defined(ANTPLUS_ON_HW_UART)
#undef CONSOLE_BAUD_RATE
#undef USE_SERIAL_CONSOLE
#endif

//Logging macros
//********************************************************************

#define SERIAL_DEBUG
//#define SERIAL_DEVDEBUG
//#define SERIAL_DEVDEBUG_FLUSH//!<Flush after each write
#define SERIAL_INFO
#define SERIAL_WARNING

#if !defined(USE_SERIAL_CONSOLE)
//Disable logging under these circumstances
#undef SERIAL_DEBUG
#undef SERIAL_DEVDEBUG
#undef SERIAL_INFO
#undef SERIAL_WARNING
#endif
//F() stores static strings that come into existence here in flash (makes things a bit more stable)
//TODO: Update with using a logging (like log4j) system - with level (tricky using F() though), time, file etc.
#ifdef SERIAL_DEBUG

#define SERIAL_DEBUG_PRINT(x)  	        (Serial.print(x))
#define SERIAL_DEBUG_PRINTLN(x)	        (Serial.println(x))
#define SERIAL_DEBUG_PRINT_F(x)  	(Serial.print(F(x)))
#define SERIAL_DEBUG_PRINTLN_F(x)	(Serial.println(F(x)))

#define SERIAL_DEBUG_PRINT2(x,y)  	(Serial.print(x,y))
#define SERIAL_DEBUG_PRINTLN2(x,y)	(Serial.println(x,y))

#else
#define SERIAL_DEBUG_PRINT(x)
#define SERIAL_DEBUG_PRINTLN(x)
#define SERIAL_DEBUG_PRINT_F(x)
#define SERIAL_DEBUG_PRINTLN_F(x)

#define SERIAL_DEBUG_PRINT2(x,y)
#define SERIAL_DEBUG_PRINTLN2(x,y)
#endif

#ifdef SERIAL_DEVDEBUG

#ifdef SERIAL_DEVDEBUG_FLUSH
//NOTE: This does not have parentheses on purpose.
#define SERIAL_DEVDEBUG_FLUSH_CMD Serial.flush();
#else
#define SERIAL_DEVDEBUG_FLUSH_CMD
#endif


#define SERIAL_DEVDEBUG_PRINT(x)  	        do { Serial.print(x);      SERIAL_DEVDEBUG_FLUSH_CMD }while(0)
#define SERIAL_DEVDEBUG_PRINTLN(x)	        do { Serial.println(x);    SERIAL_DEVDEBUG_FLUSH_CMD }while(0)
#define SERIAL_DEVDEBUG_PRINT_F(x)  	        do { Serial.print(F(x));   SERIAL_DEVDEBUG_FLUSH_CMD }while(0)
#define SERIAL_DEVDEBUG_PRINTLN_F(x)	        do { Serial.println(F(x)); SERIAL_DEVDEBUG_FLUSH_CMD }while(0)

#define SERIAL_DEVDEBUG_PRINT2(x,y)  	        do { Serial.print(x,y);    SERIAL_DEVDEBUG_FLUSH_CMD }while(0)
#define SERIAL_DEVDEBUG_PRINTLN2(x,y)	        do { Serial.println(x,y);  SERIAL_DEVDEBUG_FLUSH_CMD }while(0)

#else
#define SERIAL_DEVDEBUG_PRINT(x)
#define SERIAL_DEVDEBUG_PRINTLN(x)
#define SERIAL_DEVDEBUG_PRINT_F(x)
#define SERIAL_DEVDEBUG_PRINTLN_F(x)

#define SERIAL_DEVDEBUG_PRINT2(x,y)
#define SERIAL_DEVDEBUG_PRINTLN2(x,y)
#endif

#ifdef SERIAL_INFO
#define SERIAL_INFO_PRINT(x)  	        (Serial.print(x))
#define SERIAL_INFO_PRINTLN(x)	        (Serial.println(x))
#define SERIAL_INFO_PRINT_F(x)  	(Serial.print(F(x)))
#define SERIAL_INFO_PRINTLN_F(x)	(Serial.println(F(x)))
#else
#define SERIAL_INFO_PRINT(x)
#define SERIAL_INFO_PRINTLN(x)
#define SERIAL_INFO_PRINT_F(x)
#define SERIAL_INFO_PRINTLN_F(x)
#endif

#ifdef SERIAL_WARNING
#define SERIAL_WARN_PRINT(x)  	        (Serial.print(x))
#define SERIAL_WARN_PRINTLN(x)	        (Serial.println(x))
#define SERIAL_WARN_PRINT_F(x)  	(Serial.print(F(x)))
#define SERIAL_WARN_PRINTLN_F(x)	(Serial.println(F(x)))
#else
#define SERIAL_WARN_PRINT(x)
#define SERIAL_WARN_PRINTLN(x)
#define SERIAL_WARN_PRINT_F(x)
#define SERIAL_WARN_PRINTLN_F(x)
#endif
//********************************************************************



//Special strings that will be replaced when encountered.
//#define PRE_NAME_REPLACE        ("PNAME_REP")
#define NAME_REPLACE            ("NAME_REP")
#define MOTIVATE_REPLACE        ("MOTV_REP")
#define DISTANCE_TODAY_REPLACE  ("TODAY_REP")

#if defined(USE_ANT)
#define DISTANCE_LEFT_REPLACE   ("LEFT_REP")
#define DISTANCE_DONE_REPLACE   ("DONE_REP")
#endif //defined(USE_ANT)


#if defined(USE_ANT)

#if defined(USE_ANT_HRM)
#define BPM_REPLACE             ("BPM_REP")
#endif //defined(USE_ANT_HRM)

#if defined(USE_ANT_SDM)
#define STRIDE_COUNT_REPLACE    ("SC_REP")
#if 0 //Speed is not exciting -- removing
#define SPEED_REPLACE           ("SPD_REP")
#endif
#endif //defined(USE_ANT_SDM)

#if defined(DEBUG_MODE)
#if defined(USE_DISPLAY_7SEGMENT)
#define DEBUG_RX_REPLACE        ("DBRX_REP")
#define DEBUG_TX_REPLACE        ("DBTX_REP")
#define DEBUG_RXDP_REPLACE      ("DBRXDP_REP")
#endif //defined(USE_DISPLAY_7SEGMENT)
#endif //defined(DEBUG_MODE)

#endif //defined(USE_ANT)

#define DEBUG_STATS_REPLACE     ("DBSTATS_REP")

//#define MAX_CHARS_TO_DISPLAY (56)
#define MAX_CHARS_TO_DISPLAY (70)
#define MAX_CHARS_TO_DISPLAY_STR (MAX_CHARS_TO_DISPLAY+1) //'\0' terminated

#define ANTPLUS_BAUD_RATE (9600) //!< The moduloe I am using is hardcoded to this baud rate.


#if defined(USE_ANT)

//The ANT+ network keys are not allowed to be published so they are stripped from here.
//They are available in the ANT+ docs at thisisant.com
#define ANT_SENSOR_NETWORK_KEY {0xb9, 0xa5, 0x21, 0xfb, 0xbd, 0x72, 0xc3, 0x45}
#define ANT_GPS_NETWORK_KEY    {0xa8, 0xa4, 0x23, 0xb9, 0xf5, 0x5e, 0x63, 0xc1}

#if !defined( ANT_SENSOR_NETWORK_KEY ) || !defined(ANT_GPS_NETWORK_KEY)
#error "The Network Keys are missing. Better go find them by signing up at thisisant.com"
#endif

#endif //defined(USE_ANT)

// ****************************************************************************
// ******************************  GLOBALS  ***********************************
// ****************************************************************************

#if defined(USE_ANT)

//Arduino Pro Mini pins to the nrf24AP2 modules pinouts
#define RTS_PIN      (2) //!< RTS on the nRF24AP2 module
#define RTS_PIN_INT  (0) //!< The interrupt equivalent of the RTS_PIN


#if !defined(ANTPLUS_ON_HW_UART)
#define TX_PIN       (8) //Using software serial for the UART
#define RX_PIN       (9) //Ditto
/*static*/ SoftwareSerial ant_serial(TX_PIN, RX_PIN); // RXArd, TXArd -- Arduino is opposite to nRF24AP2 module
#else
//Using Hardware Serial (0,1) instead
#endif

#endif //defined(USE_ANT)


// EPaper Arduino IO layout
static const byte Pin_PANEL_ON  = A2;//2;
static const byte Pin_BORDER    = A3;//3;
static const byte Pin_DISCHARGE = 4;
static const byte Pin_PWM       = 5;
static const byte Pin_RESET     = 6;
static const byte Pin_BUSY      = 7;
static const byte Pin_EPD_CS    = 10;//8; //Move out of way of 8,9 for softserial?
//static const byte Pin_RED_LED   = 13; //Disconnect this??? -- Can't needs to be plugged in - SPI used
//Also A5, A4 - SDA/SCL




#if defined(USE_ANT)
//ANTPlus -- NOTE: Changes from default pins
//A0   //4/*SLEEP*/
//A1   //5/*RESET*/
/*static*/ ANTPlus        antplus   = ANTPlus(RTS_PIN, 3/*SUSPEND*/, A0/*SLEEP*/, A1/*RESET*/ );
#endif //!defined(USE_ANT)
#if defined(USE_DISPLAY_7SEGMENT)
/*static*/ LedControl     mydisplay = LedControl(11/*DIN:MOSI*/, 13/*CLK:SCK*/, 10/*CS:SS*/, 1/*Device count*/);
#endif //defined(USE_DISPLAY_7SEGMENT)


#if defined(USE_DISPLAY_EPAPER)
EPD_Class EPD(EPD_SIZE, Pin_PANEL_ON, Pin_BORDER, Pin_DISCHARGE, Pin_PWM, Pin_RESET, Pin_BUSY, Pin_EPD_CS);
#if !defined(EPAPER_HARDCODED_TEMP_CELSIUS)
LM75A_Class LM75A;
#endif //defined(EPAPER_HARDCODED_TEMP_CELSIUS)
// Graphic handler
//TODO: Move this into setup/loop so that we can create a visible error if we run out of memory
EPD_GFX G_EPD(EPD, EPD_WIDTH, EPD_HEIGHT,
#if defined(EPAPER_HARDCODED_TEMP_CELSIUS)
EPAPER_HARDCODED_TEMP_CELSIUS,
#else
LM75A,
#endif //defined(EPAPER_HARDCODED_TEMP_CELSIUS)
HEIGHT_OF_SEGMENT);
#endif //defined(USE_DISPLAY_EPAPER)

/*static*/ const long unsigned int metres_today  = 42195;

#if defined(USE_ANT)
//ANT Channels for various device types
#if defined(USE_ANT_HRM)
/*static*/ ANT_Channel hrm_channel =
{
  ANT_CHANNEL_NUMBER_INVALID,
  PUBLIC_NETWORK,
  DEVCE_TIMEOUT,
  DEVCE_TYPE_HRM,
  DEVCE_SENSOR_FREQ,
  DEVCE_HRM_LOWEST_RATE,
  ANT_SENSOR_NETWORK_KEY,
  ANT_CHANNEL_ESTABLISH_PROGRESSING,
  FALSE,
  0, //state_counter
};
#endif //defined(USE_ANT_HRM)

#if 0
/*static*/ ANT_Channel fr410_channel =
{
  ANT_CHANNEL_NUMBER_INVALID,
  PUBLIC_NETWORK,
  DEVCE_TIMEOUT,
  DEVCE_TYPE_GPS,
  DEVCE_GPS_FREQ,
  DEVCE_GPS_RATE,
  ANT_GPS_NETWORK_KEY,
  ANT_CHANNEL_ESTABLISH_PROGRESSING,
  FALSE,
  0, //state_counter
};
#endif

#if 0
/*static*/ ANT_Channel cadence_channel =
{
  ANT_CHANNEL_NUMBER_INVALID,
  PUBLIC_NETWORK,
  DEVCE_TIMEOUT,
  DEVCE_TYPE_CADENCE,
  DEVCE_SENSOR_FREQ,
  DEVCE_CADENCE_RATE,
  ANT_SENSOR_NETWORK_KEY,
  ANT_CHANNEL_ESTABLISH_PROGRESSING,
  FALSE,
  0, //state_counter
};
#endif

#if defined(USE_ANT_SDM)
//Garmin Footpod
/*static*/ ANT_Channel sdm_channel =
{
  ANT_CHANNEL_NUMBER_INVALID,
  PUBLIC_NETWORK,
  DEVCE_TIMEOUT,
  DEVCE_TYPE_SDM,
  DEVCE_SENSOR_FREQ,
  DEVCE_SDM_LOWEST_RATE,
  ANT_SENSOR_NETWORK_KEY,
  ANT_CHANNEL_ESTABLISH_PROGRESSING,
  FALSE,
  0, //state_counter
};
#endif


/*static*/ ANT_Channel * channels_to_setup[ANT_DEVICE_NUMBER_CHANNELS] =
{
#if defined(USE_ANT_SDM)
  &sdm_channel,
#endif //defined(USE_ANT_SDM)
#if defined(USE_ANT_HRM)
  &hrm_channel,
#endif //defined(USE_ANT_HRM)
};

volatile byte rts_ant_received = 0; //!< ANT RTS interrupt flag see isr_rts_ant()

#if defined(USE_ANT_HRM)
/*static*/ byte last_computed_heart_rate      = -1;
#endif //defined(USE_ANT_HRM)

#if defined(USE_ANT_SDM)
/*static*/ long unsigned int metres_left      = metres_today;
#if 0 //Speed is not exciting -- removing
/*static*/ byte last_inst_speed_int_mps       = -1;
/*static*/ byte last_inst_speed_frac_mps_d256 = -1;
#endif

/*static*/ unsigned long int cumulative_distance       = 0;
/*static*/ byte               prev_msg_distance        = -1;

/*static*/ unsigned long int cumulative_stride_count   = 0;
/*static*/ byte               prev_msg_stride_count    = -1;
#endif //defined(USE_ANT_SDM)

#endif //defined(USE_ANT)

//Used in setup and then in loop -- save stack on this big variable by using globals
/*static*/ char    adjusted_text[MAX_CHARS_TO_DISPLAY_STR]; //Are we writing past the end of this????
static uint8_t max_replace_text_len = 0;

//#define BUFFER_PROTECTION
#if defined(BUFFER_PROTECTION)
/*static*/ char    adjusted_text_buffer_protection[40]; //Are we writing past the end of this????
#endif //defined(BUFFER_PROTECTION)

#if defined(USE_DISPLAY_7SEGMENT)
/*static*/ boolean adjusted_text_decimals[MAX_CHARS_TO_DISPLAY];
#endif //defined(USE_DISPLAY_7SEGMENT)

// ****************************************************************************
// *******************************  TEXT  *************************************
// ****************************************************************************

//Counters for allowing looping/changing the various "replace"able special text.
#if defined(DEBUG_MODE)
/*static*/ unsigned int loop_pre_names = 0;
/*static*/ unsigned int loop_names     = 0;
/*static*/ unsigned int loop_motives   = 0;
#endif //defined(DEBUG_MODE)

#if defined(USE_DISPLAY_7SEGMENT)
const char startup_text_00[] PROGMEM = "- _ - ~ - _ - ~ - _ - ~ -"; //Looks cool on the screen
#endif //defined(USE_DISPLAY_7SEGMENT)
#if defined(USE_DISPLAY_EPAPER)
const char startup_text_00[] PROGMEM = "E-Paper\nRetro\nRunner";
#else
const char startup_text_00[] PROGMEM = "FIXME";
#endif //defined(USE_DISPLAY_EPAPER)

#if !defined(DEBUG_MODE)
const char startup_text_01[] PROGMEM = "Oh Hi!\nMandy";
//const char startup_text_02[] PROGMEM = "Mandy ";
const char startup_text_03[] PROGMEM = "Ready";
const char startup_text_04[] PROGMEM = "Set";
#endif //defined(DEBUG_MODE)
const char startup_text_05[] PROGMEM = "Go!";



#if defined(DEBUG_MODE)
//TODO: Rename this
const char loop_text_DEBUG_STATS[] PROGMEM = DEBUG_STATS_REPLACE;
#endif //defined(DEBUG_MODE)


//! Texts sent to the display only at startup [in setup()]
PROGMEM const char * const startup_texts[] =
{
  startup_text_00,
#if !defined(DEBUG_MODE)
  startup_text_01,
//  startup_text_02,
  startup_text_03,
  startup_text_04,
#else
  loop_text_DEBUG_STATS,
#endif //defined(DEBUG_MODE)
  startup_text_05,
};
#define STARTUP_TEXTS_COUNT ( sizeof(startup_texts)/sizeof(const char *) )

// ****************************************************************************

//const char loop_text_00[] PROGMEM         = "CanToo";
const char loop_text_00a[] PROGMEM        = "Cure\nCancer";
const char loop_text_TODAY[] PROGMEM         = DISTANCE_TODAY_REPLACE;
//const char loop_text_PRE_NAME[] PROGMEM   = PRE_NAME_REPLACE;
const char loop_text_NAME[] PROGMEM       = NAME_REPLACE;

const char loop_text_MOTIV8[] PROGMEM     = MOTIVATE_REPLACE;

#if defined(USE_ANT)


#if defined(USE_ANT_HRM)
const char loop_text_BPM[] PROGMEM        = BPM_REPLACE;
#endif //defined(USE_ANT_HRM)

#if defined(USE_ANT_SDM)
const char loop_text_DONE[] PROGMEM       = DISTANCE_DONE_REPLACE;
const char loop_text_LEFT[] PROGMEM       = DISTANCE_LEFT_REPLACE;
const char loop_text_STRIDES[] PROGMEM    = STRIDE_COUNT_REPLACE;

#if 0 //Speed is not exciting -- removing
const char loop_text_SPEED[] PROGMEM      = SPEED_REPLACE;
#endif

#endif //defined(USE_ANT_SDM)

#endif //defined(USE_ANT)

#if defined(DEBUG_MODE)

#if defined(USE_ANT)
#if defined(USE_DISPLAY_7SEGMENT)
const char loop_text_DEBUG_RX[] PROGMEM    = DEBUG_RX_REPLACE;
const char loop_text_DEBUG_TX[] PROGMEM    = DEBUG_TX_REPLACE;
const char loop_text_DEBUG_RXDP[] PROGMEM  = DEBUG_RXDP_REPLACE;
#endif //defined(USE_DISPLAY_7SEGMENT)
#endif //defined(USE_ANT)

//const char loop_text_DEBUG_STATS[] PROGMEM = DEBUG_STATS_REPLACE;

#endif //defined(DEBUG_MODE)


PROGMEM const char * const loop_texts[] =
{
#if defined(DEBUG_MODE)
//  loop_text_PRE_NAME,
  loop_text_NAME,
  loop_text_MOTIV8,

#if defined(USE_ANT)

#if defined(USE_ANT_HRM)
  loop_text_BPM,
#endif //defined(USE_ANT_HRM)

#if defined(USE_ANT_SDM)
  loop_text_STRIDES,
#if 0 //Speed is not exciting -- removing
  loop_text_SPEED,
#endif
  loop_text_DONE,
  loop_text_LEFT,
#endif //defined(USE_ANT_SDM)

#endif //defined(USE_ANT)

#if defined(USE_ANT)
#if defined(USE_DISPLAY_7SEGMENT)
  loop_text_DEBUG_RX,
  loop_text_DEBUG_TX,
  loop_text_DEBUG_RXDP,
#endif //defined(USE_DISPLAY_7SEGMENT)
#endif //defined(USE_ANT)

  loop_text_DEBUG_STATS,

#else
 //This is the real running setup

//  loop_text_00,
  loop_text_00a,
  //loop_text_TODAY,
  loop_text_NAME,
  loop_text_MOTIV8,

#if defined(USE_ANT)

#if defined(USE_ANT_HRM)
  loop_text_NAME,
  loop_text_MOTIV8,
  loop_text_BPM,
#endif //defined(USE_ANT_HRM)

  loop_text_STRIDES,
#if 0 //Speed is not exciting -- removing
  loop_text_SPEED,
#endif

  loop_text_NAME,
  loop_text_MOTIV8,

  loop_text_DONE,

  loop_text_NAME,
  loop_text_MOTIV8,
  loop_text_LEFT,
#endif //defined(USE_ANT)

  loop_text_NAME,
  loop_text_MOTIV8,



#endif //!defined(DEBUG_MODE)
};
#define LOOP_TEXTS_COUNT ( sizeof(loop_texts)/sizeof(loop_texts[0]) )

// ****************************************************************************

const char names_text_00[] PROGMEM = "Mandy";
const char names_text_01[] PROGMEM = "Flip";
const char names_text_02[] PROGMEM = "Fran";
const char names_text_03[] PROGMEM = "Andy";
const char names_text_04[] PROGMEM = "Steve";
const char names_text_05[] PROGMEM = "Micheal";
const char names_text_06[] PROGMEM = "Hayley";
const char names_text_07[] PROGMEM = "Ange";
const char names_text_08[] PROGMEM = "Emma";
const char names_text_09[] PROGMEM = "Annie";
const char names_text_10[] PROGMEM = "Sarah";
const char names_text_11[] PROGMEM = "Kirsty";
const char names_text_12[] PROGMEM = "Christy";
const char names_text_13[] PROGMEM = "Anne";
const char names_text_14[] PROGMEM = "Alice";

PROGMEM const char * const names_texts[] =
{
  names_text_00, names_text_01, names_text_02, names_text_03, names_text_04,
  names_text_05, names_text_06, names_text_07, names_text_08, names_text_09,
  names_text_10, names_text_11, names_text_12, names_text_13, names_text_14,
};
#define NAMES_TEXTS_COUNT ( sizeof(names_texts)/sizeof(names_texts[0]) )

// ****************************************************************************

const char pre_names_text_00[] PROGMEM = "Do It";
const char pre_names_text_01[] PROGMEM = "Run";
const char pre_names_text_02[] PROGMEM = "Push";
const char pre_names_text_03[] PROGMEM = "Go";
const char pre_names_text_04[] PROGMEM = "Harder";
const char pre_names_text_05[] PROGMEM = "Onya";


PROGMEM const char * const pre_names_texts[] =
{
  pre_names_text_00,
  pre_names_text_01,
  pre_names_text_02,
  pre_names_text_03,
  pre_names_text_04,
  pre_names_text_05,
};
#define PRE_NAMES_TEXTS_COUNT ( sizeof(pre_names_texts)/sizeof(pre_names_texts[0]) )

// ****************************************************************************

//                                       "0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890"
//                                       "0         1         2          3         4         5         6        7         8         9        10        11        12"
const char motivates_text_00[] PROGMEM = "Channel\nyour\ninner\nKenyan";
const char motivates_text_01[] PROGMEM = "Run Like\nYou\nStole\nSomething";
const char motivates_text_02[] PROGMEM = "Keep\nIt Up";
const char motivates_text_03[] PROGMEM = "Chafe now\nBrag Later!";
const char motivates_text_04[] PROGMEM = "Run\nHard";
const char motivates_text_05[] PROGMEM = "Core\nON!";
const char motivates_text_06[] PROGMEM = "You have\nto\nWant it!";
const char motivates_text_07[] PROGMEM = "Catch Me\nIf you Can";
const char motivates_text_08[] PROGMEM = "Mind\nyour\nstep...";
const char motivates_text_09[] PROGMEM = "Almost\nthere!";
const char motivates_text_10[] PROGMEM = "Keep\nEarning\nIce Cream";
const char motivates_text_11[] PROGMEM = "Long\nHard\nFast?";
const char motivates_text_12[] PROGMEM = "Almost\nBeer time";
const char motivates_text_13[] PROGMEM = "You have\nstamina!\nCall Me";
//const char motivates_text_14[] PROGMEM = "I just Met you; this is Crazy -- Here's My bib nuMber...";
const char motivates_text_15[] PROGMEM = "Stop\nreading\nKeep\nrunning";
const char motivates_text_16[] PROGMEM = "Toenails\nare\nOverrated";
const char motivates_text_17[] PROGMEM = "Lookout!\nBehind you.\nRUN\nRUUUUUUN!";
const char motivates_text_18[] PROGMEM = "Having\nfun\nyet?";
const char motivates_text_19[] PROGMEM = "You thought\nthey said\nRUM too?";
const char motivates_text_20[] PROGMEM = "Your pace\nor mine?";
const char motivates_text_21[] PROGMEM = "Relentless\nForward\nMotion";
//const char motivates_text_22[] PROGMEM = "Do not fear Moving sloWly forWard...  Fear standing still!";
//const char motivates_text_23[] PROGMEM = "If found on\nground\ndrag to\nfinish line";
const char motivates_text_24[] PROGMEM = "I found\nMy Happy\nPace";
const char motivates_text_25[] PROGMEM = "I wonder\nhow this\nthing works?";
const char motivates_text_26[] PROGMEM = "Pain now...\nBeer Later";
//const char motivates_text_27[] PROGMEM = "Do my eMitted photons push her faster?";
//const char motivates_text_28[] PROGMEM = "These Messages Not brought to you by the letter 'kay'";
const char motivates_text_29[] PROGMEM = "I hope you\nhaven't seen\nALL these\nMessages....";
const char motivates_text_30[] PROGMEM = "Running's a\nmental sport.\nWe're insane!!";
const char motivates_text_31[] PROGMEM = "Friends Don't\nLet Friends\nRun Marathons";
//const char motivates_text_32[] PROGMEM = "Hey!\nI just passed you!";
//const char motivates_text_33[] PROGMEM = "I run like a girl - try to keep up";
//const char motivates_text_34[] PROGMEM = "Running Won't kill you... you'll pass out first.";
const char motivates_text_35[] PROGMEM = "In it\nfor the\nlong run!";
const char motivates_text_36[] PROGMEM = "Hurdle\ncoming\nup";
//const char motivates_text_37[] PROGMEM = "Does this thing have space Invaders?";
//const char motivates_text_38[] PROGMEM = "Race entry and running shoes 200.00. Finishing one more marathon than my boyfriend....priceless.";
const char motivates_text_39[] PROGMEM = "Fast girls\nhave\ngood times";
const char motivates_text_40[] PROGMEM = "I've got\nthe runs";
const char motivates_text_41[] PROGMEM = "Good times\nahead";
const char motivates_text_42[] PROGMEM = "Running makes\nyou HOT";
const char motivates_text_43[] PROGMEM = "Ouch!!";
const char motivates_text_44[] PROGMEM = "Is that\nall you've\ngot Mitch";
const char motivates_text_45[] PROGMEM = "Your\nanniversary\nMitch & Ange";
//                                       "0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890"
//                                       "0         1         2          3         4         5         6        7         8         9        10        11        12"

PROGMEM const char * const motivates_texts[] =
{
  motivates_text_00, motivates_text_01, motivates_text_02, motivates_text_03, motivates_text_04,
  motivates_text_05, motivates_text_06, motivates_text_07, motivates_text_08, motivates_text_09,
  motivates_text_10, motivates_text_11, motivates_text_12, motivates_text_13,/*motivates_text_14,*/
  motivates_text_15, motivates_text_16, motivates_text_17, motivates_text_18, motivates_text_19,
  motivates_text_20, motivates_text_21,/*motivates_text_22,*/ /*motivates_text_23,*/ motivates_text_24,
  motivates_text_25, motivates_text_26,/*motivates_text_27,*//*motivates_text_28,*/ motivates_text_29,
  motivates_text_30, motivates_text_31, /*motivates_text_32,*//*motivates_text_33,*//*motivates_text_34,*/
  motivates_text_35, motivates_text_36, /*motivates_text_37,*//* motivates_text_38,*/ motivates_text_39,
  motivates_text_40, motivates_text_41, motivates_text_42, motivates_text_43,
  motivates_text_44, motivates_text_45,
//  motivates_text_46,
//  motivates_text_47,
//  motivates_text_48,
//  motivates_text_49,
};
#define MOTIVATES_TEXTS_COUNT ( sizeof(motivates_texts)/sizeof(motivates_texts[0]) )



// **************************************************************************************************
// *********************************  ISRs  *********************************************************
// **************************************************************************************************

#if defined(USE_ANT)
//! Interrupt service routine to get RTS from ANT messages
void isr_rts_ant()
{
  rts_ant_received = 1;
}
#endif //defined(USE_ANT)

// **************************************************************************************************
// **********************************  Helper *******************************************************
// **************************************************************************************************
#define FREE_MEM_STORED_PTRS() (stackptr - heapptr)
#define FREE_MEM_STORED_MAX_PTRS() (stackptr_lowest - heapptr_highest)
#define FREE_MEM_CALC() ( check_memory() )

uint8_t * heapptr  = NULL;
uint8_t * heapptr_highest  = NULL;
uint8_t * stackptr = NULL;
uint8_t * stackptr_lowest = (uint8_t *)(-1);
// Free RAM check for debugging. SRAM for ATmega328p = 2048Kb.
//NOTE: Should turn off interrupts if this code can be interrupted
int check_memory()
{
  stackptr = (uint8_t *)malloc(4);          // use stackptr temporarily
  heapptr = stackptr;                       // save value of heap pointer
  free(stackptr);                           // free up the memory again (sets stackptr to 0)
  stackptr =  (uint8_t *)(SP);              // save value of stack pointer

  stackptr_lowest = min(stackptr_lowest, stackptr);
  heapptr_highest = max(heapptr_highest, heapptr);

  return FREE_MEM_STORED_PTRS();
}


unsigned long my_millis_function()
{
#if defined(USE_NARCOLEPTIC_DELAY)
  return( millis() + Narcoleptic.millis() );
#else
  return millis();
#endif
}

//Function allowing the LED library to do a callback to our delay functions
//It wrappers the narco vs busy delay() logic
void my_delay_function(unsigned long duration_ms)
{
#if 1
  SERIAL_DEVDEBUG_PRINT( my_millis_function() );
  SERIAL_DEVDEBUG_PRINT_F( " @ delay " );
  SERIAL_DEVDEBUG_PRINT( duration_ms );
  SERIAL_DEVDEBUG_PRINTLN_F( " ms" );
#endif
#if 0
#if defined(USE_DISPLAY_7SEGMENT)
  //Print out some stats in the quiet period of the LED 7-segment
  if(duration_ms >= DELAY_BETWEEN_DISPLAYS_MS)
  {
      my_delay_function( 150 );

      mydisplay.shutdown(0, false);  // Turns on display
      mydisplay.clearDisplay(0);
      mydisplay.setChar(0, 0, antplus.rx_packet_count%0xF, true);
      mydisplay.setChar(0, 1, antplus.rx_packet_count%0xF0>>4, false);
      mydisplay.setChar(0, 2, antplus.tx_packet_count%0xF, true);
      mydisplay.setChar(0, 3, antplus.tx_packet_count%0xF0>>4, false);

      mydisplay.setChar(0, 7, antplus.hw_reset_count, true);

      my_delay_function( duration_ms -150 -150 );

      mydisplay.clearDisplay(0);

      my_delay_function( 150 );
  }
#endif //defined(USE_DISPLAY_7SEGMENT)
#endif

#if defined(USE_NARCOLEPTIC_DELAY)
#if defined(USE_SERIAL_CONSOLE)
  Serial.flush(); //Need to let the serial buffer finish transmissions
#endif //defined(USE_SERIAL_CONSOLE)
  Narcoleptic.delay( duration_ms );
#else
  delay( duration_ms );
#endif

}


void my_delay_function(const long duration_ms, void fn_to_call(void), const long period_to_call_at_ms )
{
  long duration_left_ms = duration_ms;
  assert( fn_to_call );

  //Call at the start
  //SERIAL_DEBUG_PRINTLN_F( "mdf:pre:fn_to_call()" );
  fn_to_call();
  do
  {
      my_delay_function( min(period_to_call_at_ms, duration_left_ms) );
      duration_left_ms -= period_to_call_at_ms;
      //SERIAL_DEBUG_PRINTLN_F( "mdf:mid:fn_to_call()" );
      fn_to_call();
  }while( duration_left_ms > 0 );

  //Always finishes the above loop with a loop call
}

//counter is just a counter value -- it does not need to be inside the pointer as we will mod it.
//Todo: String safety -- this no longer is guaranteed to be at the start of the string
void get_text_from_pm_char_ptr_array(char dst_text[], const int dst_text_size,
                                     /*PROGMEM*/ const char * const pm_char_ptr_array[], //PM Array of Ptrs to PM ptrs
                                     const int pm_char_ptr_array_size, //Num entries
                                     const int counter)
{
  const char * const* str_in_pm = &pm_char_ptr_array[ counter % pm_char_ptr_array_size ];

  //TODO: Review this -- this should not even be needed (should be pre-cleared)
  for(int i = 0; i<dst_text_size; i++)
  {
    dst_text[i] = '\0';
  }
  strncpy_P(dst_text, (char*)pgm_read_word( str_in_pm ), dst_text_size);

//  SERIAL_DEBUG_PRINT_F( "Text=" );
//  SERIAL_DEBUG_PRINT_F( " @ 0x" );
//  SERIAL_DEBUG_PRINT2( (int)str_in_pm, HEX );
//  SERIAL_DEBUG_PRINT_F( "=" );
//  SERIAL_DEBUG_PRINTLN( dst_text );
}

// **************************************************************************************************
// **********************************  Display  *****************************************************
// **************************************************************************************************

//Prints a distance with zeroes after the decimal place "42.001"
//Replaces string from start
//TODO: Add some string safety ( replace_text_size is 56 though -- so pretty safe... )
//TODO: Make it so it can work more easily mid-string
#if 0
void put_km_distance_in_string(unsigned long distance, char * const replace_text)
{
  //add kilometer part
  itoa(distance / 1000, replace_text, 10);
  //decimal
  strcat (replace_text,".");
  //Any "leading zeros" in places after the decimal
  if((distance % 1000) <= 99)
  {
    strcat (replace_text,"0");
  }
  if((distance % 1000) <= 9)
  {
    strcat (replace_text,"0");
  }
  //Rest of metre figure
  itoa(distance % 1000, replace_text + strlen(replace_text), 10);
}
#else
void put_m_distance_in_string(unsigned long distance, char * const replace_text)
{
  ltoa(distance, replace_text, 10);
}
#endif


//Fills out replace_text with appropriate values
//Will either return the incoming text (in replace_text) if no replacement happens
//Or do the replacement and return the changed text (in replace_text)
//TODO: Add some string safety ( replace_text_size is 56 though -- so pretty safe... )
void replace_special_strings(const char * const text, char * const replace_text, int replace_text_size)
{
  //SERIAL_DEBUG_PRINTLN_F( "replace_special_strings(){" );


  if(!strcmp(text, "NEVER_MATCH"))
  {
    //TODO: Tidy. This was just to make sure we had an if for 'else'ing
  }
#if defined(USE_ANT)
#if defined(USE_ANT_SDM)
  else
  if(!strcmp(text, DISTANCE_LEFT_REPLACE))
  {
    if(metres_left > 0)
    {
#if 0
      put_km_distance_in_string(metres_left, replace_text);
      strcat (replace_text,"km\n");
#else
      put_m_distance_in_string(metres_left, replace_text);
      strcat (replace_text,"m\n");
#endif
      strcat (replace_text,"to go");
    }
    else
    {
      strcpy (replace_text,"Finished\n :)");
    }
  }
#endif //defined(USE_ANT_SDM)
#endif //defined(USE_ANT)

#if defined(USE_ANT)
#if defined(USE_ANT_SDM)
  else
  if(!strcmp(text, DISTANCE_DONE_REPLACE))
  {
    unsigned long distance_done_display = min(cumulative_distance,metres_today);

#if 0
    put_km_distance_in_string(distance_done_display, replace_text);
    strcat (replace_text,"km\n");
#else
      put_m_distance_in_string(distance_done_display, replace_text);
      strcat (replace_text,"m\n");
#endif

    strcat (replace_text,"done");
  }
#endif //defined(USE_ANT_SDM)
#endif //defined(USE_ANT)

#if defined(USE_ANT)
#if defined(USE_ANT_SDM)
  else
  if(!strcmp(text, STRIDE_COUNT_REPLACE))
  {
    //2 steps to a stride
    ltoa(cumulative_stride_count * 2, replace_text, 10);
    strcat (replace_text,"\n");
    strcat (replace_text,"steps");
  }
#endif //defined(USE_ANT_SDM)
#endif //defined(USE_ANT)

#if defined(USE_ANT)
#if defined(USE_ANT_SDM)
#if 0 //Speed is not exciting -- removing
  else
  if(!strcmp(text, SPEED_REPLACE))
  {
    //strcat (replace_text,"Speed=");
    strcat (replace_text,"Speed\n");
    if(last_inst_speed_int_mps != -1)
    {
      itoa(last_inst_speed_int_mps, replace_text + strlen(replace_text), 10);
      strcat (replace_text,".");
      //Any "leading zeros" in places after the decimal
      if((last_inst_speed_frac_mps_d256*100)/256 <= 9)
      {
        strcat (replace_text,"0");
      }
      itoa( (last_inst_speed_frac_mps_d256*100)/256 , replace_text + strlen(replace_text), 10);
      strcat (replace_text," mps");
    }
    else
    {
      strcat (replace_text,"Fast?");
    }
  }
#endif
#endif //defined(USE_ANT_SDM)
#endif //defined(USE_ANT)

  else
  if(!strcmp(text, DISTANCE_TODAY_REPLACE))
  {

#if 0
    put_km_distance_in_string(metres_today, replace_text);
    strcat (replace_text,"km\n");
#else
      put_m_distance_in_string(metres_today, replace_text);
      strcat (replace_text,"m\n");
#endif

    strcat (replace_text,"Today");
  }
  else
  if(!strcmp(text, NAME_REPLACE))
  {
    //Put PRE NAME on line 1
    get_text_from_pm_char_ptr_array(replace_text, MAX_CHARS_TO_DISPLAY_STR,
                                    pre_names_texts, PRE_NAMES_TEXTS_COUNT,
#if defined(DEBUG_MODE)
                                    loop_pre_names++
#else
                                    random(PRE_NAMES_TEXTS_COUNT)
#endif //!defined(DEBUG_MODE)
                                    );
    strcat (replace_text,"\n");
    //And the NAME on line 2
    get_text_from_pm_char_ptr_array(replace_text + strlen(replace_text), MAX_CHARS_TO_DISPLAY_STR - strlen(replace_text),
                                    names_texts, NAMES_TEXTS_COUNT,
#if defined(DEBUG_MODE)
                                    loop_names++
#else
                                    random(NAMES_TEXTS_COUNT)
#endif //!defined(DEBUG_MODE)


                                    );
  }

  else
  if(!strcmp(text, MOTIVATE_REPLACE))
  {
    get_text_from_pm_char_ptr_array(replace_text, MAX_CHARS_TO_DISPLAY_STR,
                                    motivates_texts, MOTIVATES_TEXTS_COUNT,
#if defined(DEBUG_MODE)
                                    loop_motives++
#else
                                    random(MOTIVATES_TEXTS_COUNT)
#endif //!defined(DEBUG_MODE)
                                    );
  }

#if defined(USE_ANT)
#if defined(USE_ANT_HRM)
  else
  if(!strcmp(text, BPM_REPLACE))
  {
    if(last_computed_heart_rate != (-1))
    {
      itoa(last_computed_heart_rate, replace_text, 10);
      strcat (replace_text," BPM");
    }
    else
    {
      //Something a little better than the tagged replacement text.
      strcat (replace_text,"Be Heart Smart!");
    }
  }
#endif //defined(USE_ANT_HRM)
#endif //defined(USE_ANT)

#if defined(DEBUG_MODE)
#if defined(USE_ANT)
#if defined(USE_DISPLAY_7SEGMENT)
  else
  if(!strcmp(text, DEBUG_RX_REPLACE))
  {
    itoa(antplus.rx_packet_count, replace_text, 10);
    strcat (replace_text," Rx");
  }
  else
  if(!strcmp(text, DEBUG_TX_REPLACE))
  {
    itoa(antplus.tx_packet_count, replace_text, 10);
    strcat (replace_text," Tx");
  }
  else
  if(!strcmp(text, DEBUG_RXDP_REPLACE))
  {
    for(byte i=0; i < ANT_DEVICE_NUMBER_CHANNELS; i++)
    {
      if(channels_to_setup[i])
      {
        strcat (replace_text,"Chan Rx(");
        itoa(channels_to_setup[i]->channel_number, replace_text + strlen(replace_text), 10);
        strcat (replace_text,")");
        itoa(channels_to_setup[i]->data_rx, replace_text + strlen(replace_text), 10);
        strcat (replace_text,"!");
      }
    }
  }
#endif //defined(USE_DISPLAY_7SEGMENT)
#endif //defined(USE_ANT)
  else
  if(!strcmp(text, DEBUG_STATS_REPLACE))
  {
    //TODO: Only for e-paper
      strcat (replace_text,"Time=");
//      ltoa( millis()/1000, replace_text + strlen(replace_text), 10);
//      strcat (replace_text,"s");
//      strcat (replace_text,"\n[narc]=");
      ltoa( my_millis_function()/1000, replace_text + strlen(replace_text), 10);
      strcat (replace_text,"s");

      strcat (replace_text,"\n");
      strcat (replace_text,"MinFree/Free/disp=");
      itoa( FREE_MEM_STORED_MAX_PTRS(), replace_text + strlen(replace_text), 10);
      strcat (replace_text,"/");
      itoa( FREE_MEM_CALC(), replace_text + strlen(replace_text), 10);
      strcat (replace_text,"/");
      itoa( G_EPD.get_segment_buffer_size_bytes(), replace_text + strlen(replace_text), 10);
      strcat (replace_text,"B");

#if defined(USE_ANT)
#if 0 //These are on each page currently
      strcat (replace_text,"\n");
      strcat (replace_text,"Rx/Tx=");
      itoa(antplus.rx_packet_count, replace_text + strlen(replace_text), 10);
      strcat (replace_text,"/");
      itoa(antplus.tx_packet_count, replace_text + strlen(replace_text), 10);
#endif
      strcat (replace_text,"\n");
      strcat (replace_text,"p/n/m=");
      itoa(loop_pre_names, replace_text + strlen(replace_text), 10);
      strcat (replace_text,"/");
      itoa(loop_names, replace_text + strlen(replace_text), 10);
      strcat (replace_text,"/");
      itoa(loop_motives, replace_text + strlen(replace_text), 10);

#if defined(USE_ANT_HRM)
      strcat (replace_text,"\n");
      strcat (replace_text,"BPM=");
      itoa(last_computed_heart_rate, replace_text + strlen(replace_text), 10);
#endif //defined(USE_ANT_HRM)

#if defined(USE_ANT_SDM)
      strcat (replace_text,"\n");
      strcat (replace_text,"Left=");
      ltoa(metres_left, replace_text + strlen(replace_text), 10);
      strcat (replace_text,"m");
#endif //defined(USE_ANT_SDM)

#endif //defined(USE_ANT)
  }
#endif //defined(DEBUG_MODE)
  else
  {
    //No matches -- just copy across
    strncpy(replace_text, text, replace_text_size);
  }

  max_replace_text_len = max( strlen(replace_text) , max_replace_text_len );

  //Check how close to max we are
  SERIAL_DEVDEBUG_PRINT_F( "replace_text length=" );
  SERIAL_DEVDEBUG_PRINT( strlen(replace_text) );
  SERIAL_DEVDEBUG_PRINT_F( " of " );
  SERIAL_DEVDEBUG_PRINT( replace_text_size );
  SERIAL_DEVDEBUG_PRINT_F( ":max=" );
  SERIAL_DEVDEBUG_PRINTLN( max_replace_text_len );

  assert( strlen(replace_text) < replace_text_size );

  //SERIAL_DEBUG_PRINTLN_F( "}//replace_special_strings()" );
}


//TODO: out_decimals is not used for epaper.....
//TODO: fix buffer management here....
//void adjust_string(const char * text, char * out_text, boolean * out_decimals)
void adjust_string(const char * text, char * out_text, unsigned int out_text_size)
{
  //SERIAL_DEBUG_PRINTLN_F( "adjust_string(){" );

  //SERIAL_DEBUG_PRINT_F( "Text=" );
  //SERIAL_DEBUG_PRINTLN( text );

  //Clear
  for(int i = 0; i<out_text_size; i++)
  {
    out_text[i] = '\0';
  }

  //Replace motivation or name strings etc
  replace_special_strings(text, out_text, out_text_size);

  SERIAL_INFO_PRINT_F( "ConvertedText=" );
  SERIAL_INFO_PRINTLN( out_text );

  //SERIAL_DEBUG_PRINTLN_F( "}//adjust_string()" );
}




#if defined(USE_DISPLAY_EPAPER)
//TODO: Look at #pragma GCC optimize ("string"...)
//No wrapping (respects incoming '\n's). Will reduce text to fit longest line
void print_epaper_draw_chars( const char * text,
		const unsigned int width,
		const unsigned int height,
		const unsigned int segment_start_row = 0, //The start row being drawn to
		      unsigned int segment_num_rows  = 0)
{
#define MAX_NUM_LINES (7)
  unsigned char line_lengths[MAX_NUM_LINES]; //Anymore than MAX_NUM_LINES will have issues.....
  memset(line_lengths, 0, sizeof(line_lengths));
  unsigned int num_lines = 0;
  unsigned int max_line_length = 0;
  unsigned int line_start_j = 0;

  if(segment_num_rows == 0)
  {
	  segment_num_rows = height;
  }

  //SERIAL_DEBUG_PRINTLN_F("print_epaper_draw_chars()");

  for (unsigned int j = 0; j < strlen(text); ++j)
  {
    if(text[j] != '\n')
    {
      line_lengths[num_lines] = j-line_start_j+1;
    }
    else
    {
      num_lines++;
      line_start_j = j+1;
    }
  }
  if(text[strlen(text)] != '\n')
  {
    num_lines++;
  }
  assert( num_lines < MAX_NUM_LINES );

  for (unsigned int j = 0; j < num_lines; ++j)
  {
      max_line_length = max(max_line_length, line_lengths[j] );
      //SERIAL_DEBUG_PRINT_F("len=");
      //SERIAL_DEBUG_PRINT( line_lengths[j] );
      //SERIAL_DEBUG_PRINT_F(", ");
  }

  //SERIAL_DEBUG_PRINT_F("max_line_length=\t");
  //SERIAL_DEBUG_PRINT(max_line_length);
  //SERIAL_DEBUG_PRINT_F("|");
  //SERIAL_DEBUG_PRINT_F("num_lines=\t");
  //SERIAL_DEBUG_PRINTLN(num_lines);

  //TODO: Make this cleaner -- it is close to centre but not exact
  //+1 char -- we use padding of half a char on sides
  const unsigned int char_size_multiplier_width  = width /  ((max_line_length + 1) * (EPD_GFX_CHAR_PADDED_WIDTH));
  //+1 char -- we use padding of half a char on top and bottom
  const unsigned int char_size_multiplier_height = height / ((num_lines + 1)       * (EPD_GFX_CHAR_PADDED_HEIGHT));
//  SERIAL_DEBUG_PRINT_F("csm_w=");
//  SERIAL_DEBUG_PRINT( char_size_multiplier_width );
//  SERIAL_DEBUG_PRINT_F(" | csm_h=");
//  SERIAL_DEBUG_PRINT( char_size_multiplier_height );
//  SERIAL_DEBUG_PRINT_F(" -> ");
  const unsigned int char_size_multiplier = min(char_size_multiplier_width, char_size_multiplier_height);
//  SERIAL_DEBUG_PRINTLN( char_size_multiplier );

  unsigned int line_counter = 0;
#define CENTRE_TEXT
#if !defined(CENTRE_TEXT)
  //Left of screen -- adjusting for a half char buffer at size decided
  unsigned int x_start = 0 +                    (char_size_multiplier*(EPD_GFX_CHAR_BASE_WIDTH + 1))/2;
#else
  //Centre of screen -- adjusting left by half of strlen chars at size decided
  unsigned int row_width = (line_lengths[line_counter]*char_size_multiplier*(EPD_GFX_CHAR_BASE_WIDTH + 1));

  unsigned int x_start = width/2 -                    row_width/2;
#endif

  //Centre of each row -- adjusting to fit by subtracting a half char
  unsigned int row_height = (char_size_multiplier*(EPD_GFX_CHAR_BASE_HEIGHT +1));
  unsigned int y_start    = line_counter*(height/num_lines) + (height/num_lines)/2 - row_height/2;


//  SERIAL_DEBUG_PRINT_F("START[");
//  SERIAL_DEBUG_PRINT(x_start);
//  SERIAL_DEBUG_PRINT_F(",");
//  SERIAL_DEBUG_PRINT(y_start);
//  SERIAL_DEBUG_PRINT_F("]");


  unsigned int       x = x_start;
  unsigned int       y = y_start;

  for (unsigned int j = 0; j < strlen(text); ++j)
  {
    boolean do_new_line  = false;
    boolean display_char = true;
    if(text[j] == '\n')
    {
      do_new_line  = true;
      display_char = false;
    }
    if(do_new_line)
    {
      //SERIAL_DEBUG_PRINTLN_F("New line");
      line_counter++;
#if 1
      //Centreing in each row
      y_start    = line_counter*(height/num_lines) + (height/num_lines)/2 - row_height/2;
#else
      //Not centreing at each row
      y_start    = y + row_height;
#endif
#if defined(CENTRE_TEXT)
      row_width = (line_lengths[line_counter]*char_size_multiplier*(EPD_GFX_CHAR_BASE_WIDTH + 1));
      x_start = width/2 - row_width/2;
#endif
      x = x_start;
      y = y_start;
    }

    if(display_char)
    {
      //SERIAL_DEBUG_PRINT_F("c=");
      //SERIAL_DEBUG_PRINT( text[j] );
      //SERIAL_DEBUG_PRINT_F(" @[");
      //SERIAL_DEBUG_PRINT( x );
      //SERIAL_DEBUG_PRINT_F(",");
      //SERIAL_DEBUG_PRINT( y );
      //SERIAL_DEBUG_PRINT_F("]");
      G_EPD.drawChar(x, y, text[j], EPD_GFX::BLACK, EPD_GFX::WHITE, char_size_multiplier );
      x += (char_size_multiplier * (EPD_GFX_CHAR_BASE_WIDTH + 1));
    }
    //SERIAL_DEBUG_PRINTLN_F(".");
  }
}
#endif //defined(USE_DISPLAY_EPAPER)

#if defined(USE_DISPLAY_EPAPER)
#if defined(USE_LOGO)
void print_epaper_logo( )
{
  SERIAL_INFO_PRINTLN_F("Logo");
  G_EPD.clear( );
  G_EPD.drawBitmapFast( IMAGE_CANTOO_LOGO_BITS, CANTOO_LOGO_SUBSAMPLED_PARAM );
}
#endif //defined(USE_LOGO)
#endif //defined(USE_DISPLAY_EPAPER)


#if defined(USE_DISPLAY_EPAPER)
#if defined(DEBUG_OVERLAY)
void print_epaper_debug_time(int x, int y)
{
      char temp[25] = "Time=";
#if 1
      ltoa( my_millis_function()/1000, temp + strlen(temp), 10);
//              strcat (temp,"[");
//              ltoa( millis()/1000, temp + strlen(temp), 10);
//              strcat (temp,"]");
      strcat (temp,"s");
#else
      //This takes more progmem.....
      sprintf (temp, "Time=%ls", millis()/1000 );
#endif
      for (unsigned int j = 0; j < strlen(temp); ++j)
      {
        G_EPD.drawChar(x, y, temp[j], EPD_GFX::WHITE, EPD_GFX::BLACK, 1 );
        x += 1 * (EPD_GFX_CHAR_BASE_WIDTH + 1);
      }
}

void print_epaper_debug_ant(int x, int y)
{
    char temp[35] = "Rx/Tx=";

    ltoa(antplus.rx_packet_count, temp + strlen(temp), 10);
    strcat (temp,"/");
    ltoa(antplus.tx_packet_count, temp + strlen(temp), 10);
    strcat (temp,"[");
    ltoa(cumulative_distance, temp + strlen(temp), 10);
    strcat (temp,"m/");
    ltoa(cumulative_stride_count, temp + strlen(temp), 10);
    strcat (temp,"]");

    for (unsigned int j = 0; j < strlen(temp); ++j)
    {
      G_EPD.drawChar(x, y, temp[j], EPD_GFX::WHITE, EPD_GFX::BLACK, 1 );
      x += 1 * (EPD_GFX_CHAR_BASE_WIDTH + 1);
    }
}

#if defined(DEBUG_MODE)
void print_epaper_debug_counters(int x, int y)
{
    char temp[25] = "p/n/m=";

    itoa(loop_pre_names, temp + strlen(temp), 10);
    strcat (temp,"/");
    itoa(loop_names, temp + strlen(temp), 10);
    strcat (temp,"/");
    itoa(loop_motives, temp + strlen(temp), 10);

    for (unsigned int j = 0; j < strlen(temp); ++j)
    {
      G_EPD.drawChar(x, y, temp[j], EPD_GFX::WHITE, EPD_GFX::BLACK, 1 );
      x += 1 * (EPD_GFX_CHAR_BASE_WIDTH + 1);
    }
}
#endif //defined(DEBUG_MODE)



void print_epaper_debug_mem(int x, int y)
{
    char temp[30] = "Min/Free=";
    itoa( FREE_MEM_STORED_MAX_PTRS(), temp + strlen(temp), 10);
    strcat (temp,"/");
    itoa( FREE_MEM_CALC(), temp + strlen(temp), 10);
    strcat (temp,"B");

    for (unsigned int j = 0; j < strlen(temp); ++j)
    {
      G_EPD.drawChar(x, y, temp[j], EPD_GFX::WHITE, EPD_GFX::BLACK, 1 );
      x += 1 * (EPD_GFX_CHAR_BASE_WIDTH + 1);
    }
}
#endif //defined(DEBUG_OVERLAY)

void print_epaper( const char * text )
{
  	//SERIAL_DEBUG_PRINTLN_F("print_epaper(){");
	const int h        = G_EPD.real_height();
	const int seg_h    = G_EPD.height();
	const int w        = G_EPD.width();
        const unsigned int segments = G_EPD.get_segment_count();

      	G_EPD.clear( );

        for(unsigned int s=0; s < segments; s++)
        {
            G_EPD.set_current_segment(s);
            int start_row = s*seg_h;

            //Write text across segments
            print_epaper_draw_chars( text, w, h, start_row, seg_h );

#if defined(DEBUG_OVERLAY)
            if(s==(segments-2))
            {
              //Put counters in second last segment
#if defined(DEBUG_MODE)
              print_epaper_debug_counters( 0 , s*seg_h );
#endif //defined(DEBUG_MODE)
              //And mem
              print_epaper_debug_mem( (w*1)/2 , s*seg_h );
            }
            if(s==(segments-1))
            {
              //Put time in last segment
              print_epaper_debug_time( 0 , s*seg_h );
#if defined(USE_ANT)
              //And ANT stats
              print_epaper_debug_ant( (w*4)/9 , s*seg_h );
#endif //defined(USE_ANT)
            }
#endif //defined(DEBUG_OVERLAY)

            // Update the display -- first and last segments of a loop are indicated
            G_EPD.display( false, s==0, s==(segments-1) );
        }

        //SERIAL_DEBUG_PRINT_F("PE:FM=");
        //SERIAL_DEBUG_PRINTLN( FREE_MEM_CALC() );

      	//SERIAL_DEBUG_PRINTLN_F("}//print_epaper()");
}
#endif //defined(USE_DISPLAY_EPAPER)


//NOTE: Sleeps with screen on for at least display_duration_on_ms (more if more than 8 chars)
//NOTE: Sleeps with screen off for period delay_screen_off_ms
//TODO: move
/*static*/ byte logo_insert_counter = 0;

//void print_and_delay(const char * text, unsigned int display_duration_on_ms)
//No longer delays -- todo - rename
void print_and_delay(const char * text)
{
#if defined(USE_DISPLAY_7SEGMENT)
    adjust_string( text, adjusted_text, adjusted_text_decimals );
    mydisplay.shutdown(0, false);  // Turns on display
    mydisplay.setDisplayAndScroll(0, adjusted_text, adjusted_text_decimals, MAX_CHARS_TO_DISPLAY, display_duration_on_ms, my_delay_function );
    mydisplay.clearDisplay(0);
    mydisplay.shutdown(0, true);  // Turns off display (saving battery)
    my_delay_function( delay_screen_off_ms );
#endif //defined(USE_DISPLAY_7SEGMENT)

#if defined(USE_DISPLAY_EPAPER)
#if defined(BUFFER_PROTECTION)
    static boolean clear_first_through = true;
    if(clear_first_through)
    {
      //Reset
      memset(adjusted_text_buffer_protection, 'B', sizeof(adjusted_text_buffer_protection)/sizeof(adjusted_text_buffer_protection[0]) );
      clear_first_through = false;
    }

    //Check
    SERIAL_DEBUG_PRINT_F("BuffProt:");
    SERIAL_DEBUG_PRINT(__LINE__);
    for (int i=0; i< sizeof(adjusted_text_buffer_protection)/sizeof(adjusted_text_buffer_protection[0]); i++ )
    {
      if(adjusted_text_buffer_protection[i] != 'B')
      {
        SERIAL_DEBUG_PRINT_F("[");
        SERIAL_DEBUG_PRINT( i );
        SERIAL_DEBUG_PRINT_F("]=");
        SERIAL_DEBUG_PRINT( adjusted_text_buffer_protection[i] );
        SERIAL_DEBUG_PRINT_F("/");
        SERIAL_DEBUG_PRINT2( (unsigned char)adjusted_text_buffer_protection[i], HEX );
      	SERIAL_DEBUG_PRINT_F(",");
      }
    }
    SERIAL_DEBUG_PRINTLN();
    //Reset
    memset(adjusted_text_buffer_protection, 'B', sizeof(adjusted_text_buffer_protection)/sizeof(adjusted_text_buffer_protection[0]) );
#endif //defined(BUFFER_PROTECTION)

    adjust_string( text, adjusted_text, sizeof(adjusted_text)/sizeof(adjusted_text[0]) );

#if defined(BUFFER_PROTECTION)
    //Check
    SERIAL_DEBUG_PRINT_F("BuffProt:");
    SERIAL_DEBUG_PRINT(__LINE__);
    for (int i=0; i< sizeof(adjusted_text_buffer_protection)/sizeof(adjusted_text_buffer_protection[0]); i++ )
    {
      if(adjusted_text_buffer_protection[i] != 'B')
      {
        SERIAL_DEBUG_PRINT_F("[");
        SERIAL_DEBUG_PRINT( i );
        SERIAL_DEBUG_PRINT_F("]=");
        SERIAL_DEBUG_PRINT( adjusted_text_buffer_protection[i] );
        SERIAL_DEBUG_PRINT_F("/");
        SERIAL_DEBUG_PRINT2( (unsigned char)adjusted_text_buffer_protection[i], HEX );
      	SERIAL_DEBUG_PRINT_F(",");
      }
    }
    SERIAL_DEBUG_PRINTLN();
    //Reset
    memset(adjusted_text_buffer_protection, 'B', sizeof(adjusted_text_buffer_protection)/sizeof(adjusted_text_buffer_protection[0]) );
#endif //defined(BUFFER_PROTECTION)

    print_epaper( adjusted_text );
#endif //defined(USE_DISPLAY_EPAPER)
}



// **************************************************************************************************
// ***********************************  ANT+  *******************************************************
// **************************************************************************************************

#if defined(USE_ANT)
void process_packet( ANT_Packet * packet )
{
#if defined(USE_SERIAL_CONSOLE) && defined(ANTPLUS_DEBUG)
  //This function internally uses Serial.println
  //Only use it if the console is available and if the ANTPLUS library is in debug mode
  antplus.printPacket( packet, false );
#endif //defined(USE_SERIAL_CONSOLE) && defined(ANTPLUS_DEBUG)

  switch ( packet->msg_id )
  {
    case MESG_BROADCAST_DATA_ID:
    {
      const ANT_Broadcast * broadcast = (const ANT_Broadcast *) packet->data;
      SERIAL_DEBUG_PRINT_F( "CHAN " );
      SERIAL_DEBUG_PRINT( broadcast->channel_number );
      SERIAL_DEBUG_PRINT_F( " " );
      const ANT_DataPage * dp = (const ANT_DataPage *) broadcast->data;

      //Update received data
      if( channels_to_setup[broadcast->channel_number] )
      {
        channels_to_setup[broadcast->channel_number]->data_rx = true;

#if defined(USE_ANT_HRM)
        //To determine the device type -- and the data pages -- check channel setups
        if(channels_to_setup[broadcast->channel_number]->device_type == DEVCE_TYPE_HRM)
        {
            switch(dp->data_page_number)
            {
              case DATA_PAGE_HEART_RATE_0:
              case DATA_PAGE_HEART_RATE_0ALT:
              case DATA_PAGE_HEART_RATE_1:
              case DATA_PAGE_HEART_RATE_1ALT:
              case DATA_PAGE_HEART_RATE_2:
              case DATA_PAGE_HEART_RATE_2ALT:
              case DATA_PAGE_HEART_RATE_3:
              case DATA_PAGE_HEART_RATE_3ALT:
              case DATA_PAGE_HEART_RATE_4:
              case DATA_PAGE_HEART_RATE_4ALT:
              {
                //As we only care about the computed heart rate
                // we use a same struct for all HRM pages
                const ANT_HRMDataPage * hrm_dp = (const ANT_HRMDataPage *) dp;
                SERIAL_INFO_PRINT_F( "HR[X]:BPM=");
                SERIAL_INFO_PRINTLN( hrm_dp->computed_heart_rate );
                last_computed_heart_rate = hrm_dp->computed_heart_rate;
              }
              break;

              default:
                  //TODO: Other pages....
                  SERIAL_DEVDEBUG_PRINT_F(" HRM DP# ");
                  SERIAL_DEVDEBUG_PRINTLN( dp->data_page_number );
                break;
            }
        }
#endif //defined(USE_ANT_HRM)

#if defined(USE_ANT_SDM)
        if(channels_to_setup[broadcast->channel_number]->device_type == DEVCE_TYPE_SDM)
        {
              switch(dp->data_page_number)
              {
                case DATA_PAGE_SPEED_DISTANCE_1:
                {
                  const ANT_SDMDataPage1 * sdm_dp = (const ANT_SDMDataPage1 *) dp;
                  SERIAL_DEBUG_PRINT_F( "SD[1]:");
                  //Time
  //                SERIAL_DEBUG_PRINT_F( "Distance=");
  //                SERIAL_DEBUG_PRINT( sdm_dp->distance_int );
  //                SERIAL_INFO_PRINT_F( ":");
  //                SERIAL_DEBUG_PRINT( sdm_dp->distance_frac );
                  //As for DP2
#if 0 //Speed is not exciting -- removing
  //                SERIAL_INFO_PRINT_F( " | Inst Speed=");
  //                SERIAL_INFO_PRINT( sdm_dp->inst_speed_int );
  //                SERIAL_INFO_PRINT_F( ":");
  //                SERIAL_INFO_PRINT( sdm_dp->inst_speed_frac );
                  //last_inst_speed_mps = ((float)sdm_dp->inst_speed_int) + (sdm_dp->inst_speed_frac/256.0);
                  last_inst_speed_int_mps       = sdm_dp->inst_speed_int;
                  last_inst_speed_frac_mps_d256 = sdm_dp->inst_speed_frac;
#endif
                  /////
  //                SERIAL_DEBUG_PRINT_F( " | Stride count=");
  //                SERIAL_DEBUG_PRINT( sdm_dp->stride_count );
                  //Latency

                  //Processed
                  //SERIAL_INFO_PRINT_F( " | ");
                  SERIAL_INFO_PRINT_F( "CumStrid.=");
                  antplus.update_sdm_rollover( sdm_dp->stride_count, &cumulative_stride_count, &prev_msg_stride_count );
                  SERIAL_INFO_PRINT( cumulative_stride_count );
                  SERIAL_INFO_PRINT_F( " | CumDist.=");
                  antplus.update_sdm_rollover( sdm_dp->distance_int, &cumulative_distance, &prev_msg_distance );
                  SERIAL_INFO_PRINT( cumulative_distance );
  //                SERIAL_INFO_PRINT_F( " | ");
  //                SERIAL_INFO_PRINT_F( "Speed=");
  //                SERIAL_INFO_PRINT( last_inst_speed_mps );
                  SERIAL_INFO_PRINTLN( );
                }
                break;
                case DATA_PAGE_SPEED_DISTANCE_2:
                {
                  const ANT_SDMDataPage2 * sdm_dp = (const ANT_SDMDataPage2 *) dp;
                  SERIAL_DEBUG_PRINT_F( "SD[2]:");
                  //Reserved1
                  //Reserved2
                  SERIAL_INFO_PRINT_F( "Cad.=");
                  SERIAL_INFO_PRINT( sdm_dp->cadence_int );
                  SERIAL_INFO_PRINT_F( ":");
                  SERIAL_INFO_PRINT( sdm_dp->cadence_frac );
                  //As for DP1
#if 0 //Speed is not exciting -- removing
  //                SERIAL_INFO_PRINT_F( " | Inst Speed=");
  //                SERIAL_INFO_PRINT( sdm_dp->inst_speed_int );
  //                SERIAL_INFO_PRINT_F( ":");
  //                SERIAL_INFO_PRINT( sdm_dp->inst_speed_frac );
                  //last_inst_speed_mps = ((float)sdm_dp->inst_speed_int) + (sdm_dp->inst_speed_frac/256.0);
                  last_inst_speed_int_mps       = sdm_dp->inst_speed_int;
                  last_inst_speed_frac_mps_d256 = sdm_dp->inst_speed_frac;
#endif
                  ////
                  //Reserved6
                  //Status
                  //Processed
                  //SERIAL_INFO_PRINT_F( " | ");
                  //SERIAL_INFO_PRINT_F( "Speed=");
                  //SERIAL_INFO_PRINT( last_inst_speed_mps );
                  SERIAL_INFO_PRINTLN( );
                }
                break;

                default:
                  //TODO: Other pages....
                  SERIAL_DEVDEBUG_PRINT_F(" SDM DP# ");
                  SERIAL_DEVDEBUG_PRINTLN( dp->data_page_number );
                  break;
              }
          }
#endif //defined(USE_ANT_SDM)
        }
    }
    break;

    default:
      //Non-broadcast data
      //SERIAL_DEBUG_PRINTLN_F("...");
      break;
  }
}
#endif //defined(USE_ANT)

// **************************************************************************************************
// ******************************  Loop functions  **************************************************
// **************************************************************************************************


#if defined(USE_ANT)
void loop_antplus()
{
  byte packet_buffer[ANT_MAX_PACKET_LEN];
  ANT_Packet * packet = (ANT_Packet *) packet_buffer;
  MESSAGE_READ ret_val = MESSAGE_READ_NONE;

  if(rts_ant_received == 1)
  {
#if 0
    SERIAL_DEBUG_PRINTLN_F("Rx RTS Int.");
#endif
    antplus.rTSHighAssertion();
    //Clear the ISR flag
    rts_ant_received = 0;
  }

  //Read messages (no timeout) until we get a none
  while( (ret_val = antplus.readPacket(packet, ANT_MAX_PACKET_LEN, 0 )) != MESSAGE_READ_NONE )
  {
    if((ret_val == MESSAGE_READ_EXPECTED) || (ret_val == MESSAGE_READ_OTHER))
    {
  #if 0
      if( (ret_val == MESSAGE_READ_EXPECTED) )
      {
        SERIAL_DEBUG_PRINTLN_F( "Expected" );
      }
      else
      if( (ret_val == MESSAGE_READ_OTHER) )
      {
        SERIAL_DEBUG_PRINTLN_F( "Other" );
      }
  #endif
      process_packet(packet);
    }
    else
    {
      SERIAL_WARN_PRINT_F( "ReadPacket=" );
      SERIAL_WARN_PRINTLN( ret_val );
      if(ret_val == MESSAGE_READ_ERROR_MISSING_SYNC)
      {
        //Nothing -- allow a re-read to get back in sync
      }
      else
      if(ret_val == MESSAGE_READ_ERROR_BAD_CHECKSUM)
      {
        //Nothing -- fully formed package just bit errors
      }
      else
      {
        break;
      }
    }
  }


  for(byte i=0; i < ANT_DEVICE_NUMBER_CHANNELS; i++)
  {
    if(channels_to_setup[i])
    {
      if(channels_to_setup[i]->channel_establish != ANT_CHANNEL_ESTABLISH_COMPLETE)
      {
        antplus.progress_setup_channel( channels_to_setup[i] );
        if(channels_to_setup[i]->channel_establish == ANT_CHANNEL_ESTABLISH_COMPLETE)
        {
          SERIAL_INFO_PRINT( channels_to_setup[i]->channel_number );
          SERIAL_INFO_PRINTLN_F( "-Established." );
        }
        else
        if(channels_to_setup[i]->channel_establish == ANT_CHANNEL_ESTABLISH_PROGRESSING)
        {
          //SERIAL_DEBUG_PRINT( channels_to_setup[i]->channel_number );
          //SERIAL_DEBUG_PRINTLN_F( "-Progressing." );
        }
        else
        {
          SERIAL_WARN_PRINT( channels_to_setup[i]->channel_number );
          SERIAL_WARN_PRINTLN_F( "-ERROR!" );
        }
      }
    }
  }
}
#endif //defined(USE_ANT)

// ******************************************************************************************************

/*static*/ int loop_display_counter = 0;

void loop_display()
{


  char text[MAX_CHARS_TO_DISPLAY_STR];

  //SERIAL_DEBUG_PRINT( loop_display_counter );
  //SERIAL_DEBUG_PRINT_F(" | ");
  //SERIAL_DEBUG_PRINT( LOOP_TEXTS_COUNT );
  //SERIAL_DEBUG_PRINT_F(" | ");
  //SERIAL_DEBUG_PRINTLN( loop_display_counter % LOOP_TEXTS_COUNT );

  //Pull a string from the loop texts
  //These are one per loop() and some of them get replaced if they are magic texts
  get_text_from_pm_char_ptr_array(text, MAX_CHARS_TO_DISPLAY_STR, loop_texts, LOOP_TEXTS_COUNT, loop_display_counter++);

#define CALL_LOOP_ANTPLUS_IN_SLEEP_PERIOD_MS (1000)

#if defined(USE_DISPLAY_EPAPER)
#if defined(USE_LOGO)
    if((logo_insert_counter++ % LOGO_INSERT_MESSAGE_COUNT) == 0)
    {
      print_epaper_logo();
      //Execute extra loop_antplus to clear the buffers
      my_delay_function(DISPLAY_DURATION_MS, loop_antplus, CALL_LOOP_ANTPLUS_IN_SLEEP_PERIOD_MS );
    }
#endif //defined(USE_LOGO)
#endif //defined(USE_DISPLAY_EPAPER)

  //Delay is actually NOT in here anymore
  //TODO:Fix naming
  print_and_delay( text );
  //Execute extra loop_antplus to clear the buffers
  my_delay_function(DISPLAY_DURATION_MS, loop_antplus, CALL_LOOP_ANTPLUS_IN_SLEEP_PERIOD_MS );


#if defined(USE_ANT)
#if defined(USE_ANT_SDM)
  //Make the final values stick
  if(cumulative_distance > metres_today)
  {
    cumulative_distance = metres_today;
  }
  metres_left = metres_today - cumulative_distance;
  if(metres_left < 0)
  {
    metres_left = 0;
  }
#endif //defined(USE_ANT_SDM)
#endif //defined(USE_ANT)
}


// **************************************************************************************************
// ************************************  Setup  *****************************************************
// **************************************************************************************************

#if defined(USE_DISPLAY_7SEGMENT)
void setup_display_7segment()
{
  mydisplay.shutdown(0, false);  // turns on display
  mydisplay.setIntensity(0, DISPLAY_INTENSITY); // 0..15 = brightest
  mydisplay.clearDisplay(0);
  mydisplay.shutdown(0, true);  // turns off display
}
#endif //defined(USE_DISPLAY_7SEGMENT)


#if defined(USE_DISPLAY_EPAPER)
void setup_display_epaper()
{
	//pinMode(Pin_RED_LED, OUTPUT);

	pinMode(Pin_PWM, OUTPUT);
	pinMode(Pin_BUSY, INPUT);
	pinMode(Pin_RESET, OUTPUT);
	pinMode(Pin_PANEL_ON, OUTPUT);
	pinMode(Pin_DISCHARGE, OUTPUT);
	pinMode(Pin_BORDER, OUTPUT);
	pinMode(Pin_EPD_CS, OUTPUT);


	//digitalWrite(Pin_RED_LED, LOW);
	digitalWrite(Pin_PWM, LOW);
	digitalWrite(Pin_RESET, LOW);
	digitalWrite(Pin_PANEL_ON, LOW);
	digitalWrite(Pin_DISCHARGE, LOW);
	digitalWrite(Pin_BORDER, LOW);
	digitalWrite(Pin_EPD_CS, LOW);

	// set up graphics EPD library
	// and clear the screen

	G_EPD.begin();
}
#endif //defined(USE_DISPLAY_EPAPER)


void setup_display()
{
#if defined(USE_DISPLAY_7SEGMENT)
  setup_display_7segment();
#endif //defined(USE_DISPLAY_7SEGMENT)
#if defined(USE_DISPLAY_EPAPER)
  setup_display_epaper();
#endif //defined(USE_DISPLAY_EPAPER)


}

void setup()
{
#if defined(USE_SERIAL_CONSOLE)
  Serial.begin(CONSOLE_BAUD_RATE);
#endif //defined(USE_SERIAL_CONSOLE)

  SERIAL_DEBUG_PRINTLN_F("++++++++++");
  SERIAL_INFO_PRINTLN("CanToo Runner!");
  SERIAL_DEBUG_PRINTLN_F("Setup.");

  setup_display();

#if defined(USE_NARCOLEPTIC_DISABLE)
  Narcoleptic.disableTimer2();
//  Narcoleptic.disableTimer1(); //Needed for SPI
//  Narcoleptic.disableMillis();
#if !(defined(USE_SERIAL_CONSOLE) || defined(ANTPLUS_ON_HW_UART))
  //Don't need serial if not debugging nor using the hardware USART usage.
  Narcoleptic.disableSerial();
#endif
  Narcoleptic.disableADC();

#endif

  SERIAL_DEBUG_PRINT_F("SRAM Free/Disp/ReplText=");
  SERIAL_DEBUG_PRINT( FREE_MEM_CALC() );
#if defined(USE_DISPLAY_EPAPER)
  SERIAL_DEBUG_PRINT_F("/");
  SERIAL_DEBUG_PRINT( G_EPD.get_segment_buffer_size_bytes() );
#else
  SERIAL_DEBUG_PRINT( 0 );
#endif //defined(USE_DISPLAY_EPAPER)
  SERIAL_DEBUG_PRINT_F("/");
  SERIAL_DEBUG_PRINT( sizeof(adjusted_text) ); //TODO: + decimals for the 7-seg version
  SERIAL_DEBUG_PRINTLN_F("B.");

#if defined(USE_DISPLAY_EPAPER)
#if defined(USE_LOGO)
   print_epaper_logo();
   my_delay_function(DISPLAY_DURATION_STARTUP_MS);
#endif //defined(USE_LOGO)
#endif //defined(USE_DISPLAY_EPAPER)

  //Print the startup messages
  for(unsigned int counter_setup = 0;
      counter_setup < STARTUP_TEXTS_COUNT; counter_setup++)
  {
    char text[MAX_CHARS_TO_DISPLAY_STR];
    get_text_from_pm_char_ptr_array(text, MAX_CHARS_TO_DISPLAY_STR,
                                    startup_texts, STARTUP_TEXTS_COUNT, counter_setup);
    print_and_delay( text );
    my_delay_function( DISPLAY_DURATION_STARTUP_MS );
  }

#if defined(USE_ANT)
  SERIAL_DEBUG_PRINTLN_F("ANT+ Config.");

  //We setup an interrupt to detect when the RTS is received from the ANT chip.
  //This is a 50 usec HIGH signal at the end of each valid ANT message received from the host at the chip
  attachInterrupt(RTS_PIN_INT, isr_rts_ant, RISING);


#if defined(ANTPLUS_ON_HW_UART)
  //Using hardware UART
  Serial.begin(ANTPLUS_BAUD_RATE);
  antplus.begin( Serial );
#else
  //Using soft serial
  ant_serial.begin( ANTPLUS_BAUD_RATE );
  antplus.begin( ant_serial );
#endif

  for(byte i=0; i < ANT_DEVICE_NUMBER_CHANNELS; i++)
  {
    if(channels_to_setup[i])
    {
      //For now we do a simple direct mapping
      channels_to_setup[i]->channel_number = i;
      SERIAL_DEBUG_PRINT_F("Configured to establish ANT channel #");
      SERIAL_DEBUG_PRINTLN( channels_to_setup[i]->channel_number );
    }
  }

  SERIAL_DEBUG_PRINTLN_F("ANT+ Config Finished.");
#endif //defined(USE_ANT)

  SERIAL_INFO_PRINTLN_F("Setup Finished.");
}


// **************************************************************************************************
// ************************************  Loop *******************************************************
// **************************************************************************************************

//#define SKIP_ANT_LOOP //!<Keep ANT in but remove it from locking us out.

void loop()
{
#if defined(USE_ANT) && !defined(SKIP_ANT_LOOP)
  loop_antplus();
#endif //defined(USE_ANT)

  boolean channels_to_setup_established = true;
#if defined(USE_ANT)  && !defined(SKIP_ANT_LOOP)
  //Get the ANT+ channels up quickly
  for(byte i=0; i < ANT_DEVICE_NUMBER_CHANNELS; i++)
  {
    if(channels_to_setup[i])
    {
      channels_to_setup_established &= (channels_to_setup[i]->channel_establish == ANT_CHANNEL_ESTABLISH_COMPLETE);
    }
  }
#endif //defined(USE_ANT)


  // then send some display messages
  //This sometimes takes a while to execute and it calls loop_antplus() inside also
  if(channels_to_setup_established)
  {
    loop_display();
  }
}

