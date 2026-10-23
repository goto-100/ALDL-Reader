#include <LiquidCrystal.h>

/*

     +--------+     
     |  ALDL  |     
     | Reader |     
     +--------+     

Reads diagnostic data from an ALDL port then displays real time data and error codes on a text display
Written by Vitor de Paulo 06/12/2024
ALDL protocol code based on the "ALDLTAPr3" project by R.C. Stanley
Serial data interpretation taken from various sources - using data stream A28 for a 1227165 ECM (works with my 1987 Corvette)
Modification for other data streams (see "ALDL stuff" folder) may be necessary depending on vehicle
This software is licensed under the GNU General License. The licensee is free to copy and modify this software
Compiled with Arduino 1.8.13 (not Arduino IDE!) for use in a Arduino MEGA256 board

Connections to the GM ALDL:
        - Arduino Rx1 and DIO 4 pins are connected together to the ALDL data pin "E" on the ALDL connector
        - The Tx1 Arduino pin connects to the cathode of a 1N4001(or similar) diode with the anode connected to ALDL pin "E"
          This allows Tx to pull down pin "E" during transmission without interfering with the ECM serial output.
        - Arduino GND is connected to ECM GND at ALDL pin "A"
        - A 10K resistor between ALDL pins "A" and "B" puts the ECM into ALDL mode (this enables MODE 1, but also adds 10 degrees of ignition timing at idle)

On startup, the code expects the ECM to be transmitting in ALDL mode at 160 Baud. Input pin 4 is used to listen for a silent period between the data packets.
The 8192 Baud Mode1 command (MODE 1 request) is then sent from the Arduino to the ECM, switching to 8192 baud command mode. Sometimes this takes a few seconds.
Once communications are in sync, the Arduino requests a MODE1 data packet from the ECM and the ECM responds with a preamble, 63 bytes of data, and a checksum.
Data is interpreted and displayed on the LCD.
Note: MODE 10 (clear error codes) is implemented here, but ignored (not implemented) on the 1987 Corvette ECM. Should work on newer cars, but not tested!

Connections to push buttons, all normally open, between the following pins and GND:
        - Page up key to DIO 8
        - Page down key to DIO 7
        - Help key to DIO 9
        - Cancel/clear codes key to DIO 10
Usage:
        - Navigate data pages with up/down keys
        - Pressing Help key will show content depending on which page is selected at the moment
        - Pressing it again will cycle through multiple help pages (if more than one) and then return to the original data page
        - Cancel/clear codes key: see note above, optional
                              
Connections to common anode RGB LED (optional), with anode through a PAREI resistor to the PAREI power rail, and the following pins for each cathode:
        - Red cathode to DIO 22
        - Green cathode to DIO 26
        - Blue cathode to DIO 24

Connections of text 20x4 LCD; display pins to the following pins:
        - Reset to DIO 12
        - Enable to DIO 11
        - Data4 to DIO 5
        - Data5 to DIO 6
        - Data6 to DIO 3
        - Data7 to DIO 2

*/

//Test without car
//#define OFFLINE

#define ALDLTestPin  4                                                    // Input used to listen for 160 BAUD ALDL activity before commanding 8192 BAUD
int ALDLbyte = 0;                                                         // One byte of aldl data
int bytecounter =0;                                                       // count bytes for line spacing in output
const int linecount = 64;                                                 // 32 bytes per line
byte M1Cmd[4] = { 0x80, 0x56, 0x01, 0x29 };                               // Mode 1 command to start 8192 Mode 1 DataStream (80 56 01 29 HEX)
byte M10Cmd[4] = { 0x80, 0x56, 0x0A, 0x20 };                              // Mode 10 command to clear error codes (80 56 0A 20 HEX)
byte Preamble[3] = { 0x80, 0x95, 0x01 };                                  // Preamble from ECM indicates start of 8192 Mode 1 DataStream (80 95 01 HEX)
bool clearRequested;
bool PreambleFound = false;                                               // Reset preamble found flag
bool SilenceFound = false;                                                // Flag to indicate that ECM silence in 160 baud mode has been found
bool CommInit8192 = false;                                                // Flag indicates that 8192 baud communications has been initialized
const double SilenceWait = 15000;                                         // Minimum silence period before transmitting a Mode 1 command
unsigned long PreambleTimer;                                              // Timeout timer for preamble
const int ByteCount = 64;                                                 // Number of data bytes following preamble in data stream including checksum
byte DataBytes[ByteCount];                                                // Array to hold the serial data stream
int DataStreamIndex = 1;                                                  // Data stream byte Index, start at 1 and end at 63 to match Assembly listing
int i = 0;                                                                // Preamble index counter
unsigned long StartTime;                                                  // Microsecond counter for measuring the time of incoming data
unsigned int  CheckTotal;                                                 // Total of bytes recieved for calculating the checksum 
byte CheckSum;                                                            // Actual calculated Checksum
#define BUFFERDELAY 50                                                    // Originally was 300ms

float RPM, RPMTargetIdle;                                                 // Engine RPM (current and target value for idle)
float TPS;                                                                // Percent TPS
float MAF, MAFRaw;                                                        // Mass Air Flow in g/s (corrected and raw from sensor)
float injPW;                                                              // Injector Pulse Width
float O2mV;                                                               // O2 sensor mV
int BLCELL;                                                               // Currently used block learn cell #. There are 16 cells organized bye RPM and air flow 0, is idle, 15 is max RPM & Airflow
int BLM;                                                                  // Block Learn Value for cell being currently used
int INTEGRATOR;                                                           // Current fueling control integrator value over 128 indicates fuel is added, under 128 fuel is taken out, 160 max
float MAT;                                                                // Intake Manifold Air Temperature
float ECT, ECT0;                                                          // Engine Coolant Temperature (current and at startup)
unsigned int MPH;
unsigned int O2Crosses;
unsigned int runTime;                                                     // Engine Run time
unsigned int IACPos, IACDir;
unsigned int loadFiltered, loadRaw;
float loadFilteredPc, loadRawPc;
float EGRDc, canDc, fanDc;
float Vbat, Vpump;
float sparkTDC, sparkRefPulse;
char sparkTempA;
int sparkTempB;
float sparkRetard;
unsigned int ESCSignal;
unsigned char flag0, flag1, flag2, flag3, flag4, flag5, flag6, flag7;
unsigned char flagsInput;
char strBufA[6], strBufB[6];                                                // Used to overcome lack of sprintf() support for floats - use with dtostrf()

#define LCDLINEBUF        21                                                // LCD size is 20 wide, plus \0
#define LCDLINEBUF_GUARD  31                                                // Additional 10 bytes to serve as shielding againts string misuse
#define LCDEOLPOS         20                                                // Position to manually add a \0 to buffers to avoid printing characters beyond the LCD viewport (wasting time)
volatile char obuf0[LCDLINEBUF_GUARD];
volatile char obuf1[LCDLINEBUF_GUARD];
volatile char obuf2[LCDLINEBUF_GUARD];
volatile char obuf3[LCDLINEBUF_GUARD];
const char clearLine[] = "                    ";
const int rs = 12, en = 11, d4 = 5, d5 = 6, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

/*
1985-1991 Diagnostic Trouble Codes
Code #12: Normal No Codes.
Code #13: Open Oxygen Sensor Circuit.
Code #14: Coolant Sensor Circuit Low.
Code #15: Coolant Sensor Circuit High.
Code #21: Throttle Position Sensor High.
Code #22: Throttle Position Sensor Low.
Code #23: Manifold Air Temperature Circuit High.
Code #24: Vehicle Speed Sensor.
Code #25: Manifold Air Temperature Circuit Low.
Code #32: EGR System Failure.
Code #33: Mass Air Flow Sensor High. 85 Manual says Mass Air FLow Sensor Low.
Code #34: Mass Air Flow Sensor Low. 85 Manual says Mass Air Flow Sensor High.
Code #36: Mas Air Flow Sensor Burn-Off Function Fault.
Code #41: Cylinder Select Error.
Code #42: Electronic Spark Timing.
Code #43: Electronic Spark Control.
Code #44: Lean Exhaust indication.
Code #45: Rich Exhaust Indication.
Code #46: Vehicle Anti Theft Fault.
Code #51: Faulty Mem-Cal. 85 Manual says PROM.
Code #52: Fuel Calpak Missing.
Code #52(1990-91 Corvette Only): Engine Oil Temperature Sensor Low.
Code #53: System Over Voltage.
Code #54: Fuel Pump Circuit Low Voltage.
Code #55: Defective ECM.
Code #62: Engine Oil Temperature Sensor Circuit High
*/
enum Dtc_codes { DTC23, DTC22, DTC21, DTC15, DTC14, DTC13, DTC12, DTC34, DTC33, DTC32, DTC25, DTC24, DTC51, DTC46, DTC45, DTC44, DTC43, DTC42, DTC41, DTC36, DTC62, DTC55, DTC54, DTC53, DTC52, DTC_TOTAL };
char dtcFlags[DTC_TOTAL];
const char DTC23Name[] PROGMEM = "23 IAT OPEN/LOW     ";
const char DTC22Name[] PROGMEM = "22 TPS LOW          ";
const char DTC21Name[] PROGMEM = "21 TPS HIGH         ";
const char DTC15Name[] PROGMEM = "15 ECT LOW          ";
const char DTC14Name[] PROGMEM = "14 ECT HIGH         ";
const char DTC13Name[] PROGMEM = "13 O2 OPEN          ";
const char DTC12Name[] PROGMEM = "12 DISTR REF PULSES ";
const char DTC34Name[] PROGMEM = "34 MAF LOW          ";
const char DTC33Name[] PROGMEM = "33 MAF HIGH         ";
const char DTC32Name[] PROGMEM = "32 EGR DIAGNOSTICS  ";
const char DTC25Name[] PROGMEM = "25 IAT HIGH         ";
const char DTC24Name[] PROGMEM = "24 VEHICLE SPEED    ";
const char DTC51Name[] PROGMEM = "51 PROM ERROR       ";
const char DTC46Name[] PROGMEM = "46 VATS FAILED      ";
const char DTC45Name[] PROGMEM = "45 O2 RICH          ";
const char DTC44Name[] PROGMEM = "44 O2 LEAN          ";
const char DTC43Name[] PROGMEM = "43 SPARK CONTROL-ESC";
const char DTC42Name[] PROGMEM = "42 SPARK TIMING-EST ";
const char DTC41Name[] PROGMEM = "41 CYLINDER SELECT  ";
const char DTC36Name[] PROGMEM = "36 BURNOFF          ";
const char DTC62Name[] PROGMEM = "62 OIL TEMP HIGH    ";
const char DTC55Name[] PROGMEM = "55 ECM FAILURE      ";
const char DTC54Name[] PROGMEM = "54 FUELPUMP LOW VOLT";
const char DTC53Name[] PROGMEM = "53 BAT OVERVOLTAGE  ";
const char DTC52Name[] PROGMEM = "52 CALPACK MISSING  ";
const char * const dtcNames[] PROGMEM = { DTC23Name, DTC22Name, DTC21Name, DTC15Name, DTC14Name, DTC13Name, DTC12Name, DTC34Name, DTC33Name, DTC32Name, DTC25Name, DTC24Name, DTC51Name, DTC46Name, DTC45Name, DTC44Name, DTC43Name, DTC42Name, DTC41Name, DTC36Name, DTC62Name, DTC55Name, DTC54Name, DTC53Name, DTC52Name };

#define UPKEYPIN      8
#define DOWNKEYPIN    7
#define HELPKEYPIN    9
#define CANCELKEYPIN  10
unsigned int key_was_pressed = 0;
#define KEYDELAY      1

#define REDLED    22
#define GREENLED  26
#define BLUELED   24
#define TURN_ON_RED_LED     digitalWrite(REDLED, 0)
#define TURN_OFF_RED_LED    digitalWrite(REDLED, 1)
#define TURN_ON_GREEN_LED   digitalWrite(GREENLED, 0)
#define TURN_OFF_GREEN_LED  digitalWrite(GREENLED, 1)
#define TURN_ON_BLUE_LED    digitalWrite(BLUELED, 0)
#define TURN_OFF_BLUE_LED   digitalWrite(BLUELED, 1)
#define MAKE_LED_BLACK      TURN_OFF_RED_LED; TURN_OFF_GREEN_LED; TURN_OFF_BLUE_LED;
#define MAKE_LED_WHITE      TURN_ON_RED_LED; TURN_ON_GREEN_LED; TURN_ON_BLUE_LED;
#define MAKE_LED_RED        TURN_ON_RED_LED; TURN_OFF_GREEN_LED; TURN_OFF_BLUE_LED;
#define MAKE_LED_GREEN      TURN_OFF_RED_LED; TURN_ON_GREEN_LED; TURN_OFF_BLUE_LED;
#define MAKE_LED_BLUE       TURN_OFF_RED_LED; TURN_OFF_GREEN_LED; TURN_ON_BLUE_LED;
#define MAKE_LED_YELLOW     TURN_ON_RED_LED; TURN_ON_GREEN_LED; TURN_OFF_BLUE_LED;
#define MAKE_LED_CYAN       TURN_OFF_RED_LED; TURN_ON_GREEN_LED; TURN_ON_BLUE_LED;
#define MAKE_LED_MAGENTA    TURN_ON_RED_LED; TURN_OFF_GREEN_LED; TURN_ON_BLUE_LED;

int pageNum = 0;
#define LASTPAGE    9

float T_interpolate(byte DS_Temp) {                                       // Subroutine to interpolate MAT temperature from data stream. Input data is one byte of AD from the temperature thermistor (Very Non Linear)

  //Temperature scale for interpolating air temperature taken from the ADS file There are 38 values in the table ranging from -40C for a value of 1 to 200 C for a value of 256
  const float TempScale[38] = {1,5,6,9,11,15,19,25,31,38,47,57,67,79,91,104,117,130,142,154,164,175,184,192,200,206,212,217,222,226,230,233,235,238,240,242,244,256};  // Data Values (38)
  const float TempValue[38] = {-40,-30,-25,-20,-15,-10,-5,0,5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105,110,115,120,125,130,135,140,145,200};      // Temp Values (38)
  float T_Range;                                                          // Interpolation range
  float T_Diff;                                                           // Difference between input value and the bottom of the interpolation range
  float TempRange;                                                        // Difference between points on the output value scale
  float Temperature;                                                      // Interpolated Temperature value
  int i = 0;

  while (i<38) {                                                          // Loop through increasing values to find start of the interpolation
    if  (TempScale[i]> DS_Temp) break;                                    // Until a higher value is found - exit with i pointing to the higher value for interpolation
    i++;
  }
  if (i>0) {                                                              // Figure out the linear interpolation range and difference along the scale
    T_Range = TempScale[i] - TempScale[i-1];                              // Range between these points along the input scale (all calculated as a floating point)
    T_Diff =  DS_Temp - TempScale[i-1];                                   // Difference between data and the lower point
    TempRange = TempValue[i] - TempValue[i-1];                            // Difference between points along the output value scale  
    Temperature =  TempValue[i-1] + (T_Diff/T_Range)*TempRange;           // Interpolated Temperature
  } else {
    Temperature = TempValue[0];                                           // ...unless the input date is <= to the bottom of the scale and the minimum value is used
  }
  return Temperature;
}



void lcdSlam(char * p1, char * p2, char * p3, char * p4) {

    static char * p;
    
    p = p1 + LCDEOLPOS;
    *p = '\0';
    p = p2 + LCDEOLPOS;
    *p = '\0';
    p = p3 + LCDEOLPOS;
    *p = '\0';
    p = p4 + LCDEOLPOS;
    *p = '\0';
    lcd.setCursor(0,0);
    lcd.print(p1);
    lcd.setCursor(0,1);
    lcd.print(p2);
    lcd.setCursor(0,2);
    lcd.print(p3);
    lcd.setCursor(0,3);
    lcd.print(p4);
}



void readDTCs(unsigned char byte0, unsigned char byte1, unsigned char byte2, unsigned char byte3, unsigned char byte4) {

  memset(dtcFlags, 0, sizeof(dtcFlags));

  if (byte0 & 0x01) { dtcFlags[DTC23] = 1; }
  if (byte0 & 0x02) { dtcFlags[DTC22] = 1;  }
  if (byte0 & 0x04) { dtcFlags[DTC21] = 1; }
  //if (byte0 & 0x08) { }
  if (byte0 & 0x10) { dtcFlags[DTC15] = 1; }
  if (byte0 & 0x20) { dtcFlags[DTC14] = 1; }
  if (byte0 & 0x40) { dtcFlags[DTC13] = 1; }
  if (byte0 & 0x80) { dtcFlags[DTC12] = 1; }
  
  //if (byte1 & 0x01) { }
  if (byte1 & 0x02) { dtcFlags[DTC34] = 1; }
  if (byte1 & 0x04) { dtcFlags[DTC33] = 1; }
  if (byte1 & 0x08) { dtcFlags[DTC32] = 1; }
  //if (byte1 & 0x10) { }
  //if (byte1 & 0x20) { }
  if (byte1 & 0x40) { dtcFlags[DTC25] = 1; }
  if (byte1 & 0x80) { dtcFlags[DTC24] = 1; }

  if (byte2 & 0x01) { dtcFlags[DTC51] = 1; }
  if (byte2 & 0x02) { dtcFlags[DTC46] = 1; }
  if (byte2 & 0x04) { dtcFlags[DTC45] = 1; }
  if (byte2 & 0x08) { dtcFlags[DTC44] = 1; }
  if (byte2 & 0x10) { dtcFlags[DTC43] = 1; }
  if (byte2 & 0x20) { dtcFlags[DTC42] = 1; }
  if (byte2 & 0x40) { dtcFlags[DTC41] = 1; }
  if (byte2 & 0x80) { dtcFlags[DTC36] = 1; }
  
  //if (byte3 & 0x01) { }
  if ((byte3 | byte4) & 0x02) { dtcFlags[DTC62] = 1; }
  //if (byte3 & 0x04) { }
  //if (byte3 & 0x08) { }
  if ((byte3 | byte4) & 0x10) { dtcFlags[DTC55] = 1; }
  if (byte3 & 0x20) { dtcFlags[DTC54] = 1; }
  if (byte3 & 0x40) { dtcFlags[DTC53] = 1; }
  if (byte3 & 0x80) { dtcFlags[DTC52] = 1; }
}



void waitForHelpKey() {
  do {
    delay(50);
  } while(digitalRead(HELPKEYPIN)==0);
  while(digitalRead(HELPKEYPIN)) {
    delay(50);
  }
  do {
    delay(50);
  } while(digitalRead(HELPKEYPIN)==0);
}




void help() {

  MAKE_LED_BLACK;
  switch(pageNum) {
    case 0:
      //Screen layout:
      //+ 01234567890123456789
      //0 NN -----------------
      //1 NN -----------------
      //2 NN -----------------
      //3 NN -----------------
      memcpy(obuf0, dtcNames[0], LCDLINEBUF);
      memcpy(obuf1, dtcNames[1], LCDLINEBUF);
      memcpy(obuf2, dtcNames[2], LCDLINEBUF);
      memcpy(obuf3, dtcNames[3], LCDLINEBUF);
      lcdSlam(obuf0, obuf1, obuf2, obuf3);
      waitForHelpKey();
      memcpy(obuf0, dtcNames[4], LCDLINEBUF);
      memcpy(obuf1, dtcNames[5], LCDLINEBUF);
      memcpy(obuf2, dtcNames[6], LCDLINEBUF);
      memcpy(obuf3, dtcNames[7], LCDLINEBUF);
      lcdSlam(obuf0, obuf1, obuf2, obuf3);
      waitForHelpKey();
      memcpy(obuf0, dtcNames[8], LCDLINEBUF);
      memcpy(obuf1, dtcNames[9], LCDLINEBUF);
      memcpy(obuf2, dtcNames[10], LCDLINEBUF);
      memcpy(obuf3, dtcNames[11], LCDLINEBUF);
      lcdSlam(obuf0, obuf1, obuf2, obuf3);
      waitForHelpKey();
      memcpy(obuf0, dtcNames[12], LCDLINEBUF);
      memcpy(obuf1, dtcNames[13], LCDLINEBUF);
      memcpy(obuf2, dtcNames[14], LCDLINEBUF);
      memcpy(obuf3, dtcNames[15], LCDLINEBUF);
      lcdSlam(obuf0, obuf1, obuf2, obuf3);
      waitForHelpKey();
      memcpy(obuf0, dtcNames[16], LCDLINEBUF);
      memcpy(obuf1, dtcNames[17], LCDLINEBUF);
      memcpy(obuf2, dtcNames[18], LCDLINEBUF);
      memcpy(obuf3, dtcNames[19], LCDLINEBUF);
      lcdSlam(obuf0, obuf1, obuf2, obuf3);
      waitForHelpKey();
      memcpy(obuf0, dtcNames[20], LCDLINEBUF);
      memcpy(obuf1, dtcNames[21], LCDLINEBUF);
      memcpy(obuf2, dtcNames[22], LCDLINEBUF);
      memcpy(obuf3, dtcNames[23], LCDLINEBUF);
      lcdSlam(obuf0, obuf1, obuf2, obuf3);
      waitForHelpKey();
      memcpy(obuf0, dtcNames[24], LCDLINEBUF);
      memcpy(obuf1, clearLine, LCDLINEBUF);
      memcpy(obuf2, clearLine, LCDLINEBUF);
      memcpy(obuf3, clearLine, LCDLINEBUF);
      lcdSlam(obuf0, obuf1, obuf2, obuf3);
      waitForHelpKey();
      break;
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
      lcdSlam("No help             ", clearLine, clearLine, clearLine);
      waitForHelpKey();
      break;
    case 6:
      lcdSlam("0=Overdrive     [54]", "1=14|15 this startup", "2=Ref pulse occured ", "3=8192 and Mode 4   ");
      waitForHelpKey();
      lcdSlam("4=Diag. position[54]", "5=ALDL position     ", "6=High bat,sol. dis.", "7=Shift light       ");
      waitForHelpKey();
      lcdSlam("0=P|N(aut)      [55]", "1=!3rd gear(aut)    ", "2=OD(aut),!1st(man) ", "3=Excess PS press.  ");
      waitForHelpKey();
      lcdSlam("4=Diag sw closed[55]", "5=Torq. conv. lock  ", "6=Fan request       ", "7=!A/C request      ");
      waitForHelpKey();
      lcdSlam("0=P|N(aut)      [56]", "1=3rd gear(aut)     ", "2=4th(aut),1st(man) ", "3=PS cramp          ");
      waitForHelpKey();
      lcdSlam("4=Not used      [56]", "5=EGR diagnostic    ", "6=A/C hi-press., fan", "7=!A/C request      ");
      waitForHelpKey();
      break;
    case 7:
      lcdSlam("0=Retard spark  [57]", "1=Chk eng lite delay", "2=Interrupt problem ", "3=Fan disabled      ");
      waitForHelpKey();
      lcdSlam("4=TC speed 1st  [57]", "5=!A/C clutch       ", "6=Bypass chk enable ", "7=Engine running    ");
      waitForHelpKey();
      lcdSlam("0=O2 sens. ready[58]", "1=Clos. loop timeout", "2=Not used          ", "3=Improper shutdown ");
      waitForHelpKey();
      lcdSlam("4=Not used      [58]", "5=IAC kickdown enab.", "6=Kwarm kickd. enab.", "7=Spark monitor fail");
      waitForHelpKey();
      lcdSlam("0=OD on         [59]", "1=1st gear direct   ", "2=1st upshift       ", "3=4th gear          ");
      waitForHelpKey();
      lcdSlam("4=1st gear OD   [59]", "5=OD off request    ", "6=OD on request     ", "7=OD active         ");
      waitForHelpKey();
      break;
    case 8:
      lcdSlam("0=ALDL reset    [60]", "1=Clean cannister   ", "2=Field service mode", "3=1st CL pass f.s.m.");
      waitForHelpKey();
      lcdSlam("4=F.s.m. O2     [60]", "5=One second flag   ", "6=200ms flash rate  ", "7=Pullup for coolant");
      waitForHelpKey();
      lcdSlam("0=Old CCP purge [61]", "1=!AIR diverted     ", "2=AIR to port       ", "3=Burnoff complete  ");
      waitForHelpKey();
      lcdSlam("4=Skip BO (>17V)[61]", "5=D.E. qsec         ", "6=Burnoff           ", "7=Decel. enleanment ");
      waitForHelpKey();
      lcdSlam("0=Exp. 1st 160 i[62]", "1=Exp. 2nd 160 int. ", "2=8192 baud mode    ", "3=8192 locked       ");
      waitForHelpKey();
      lcdSlam("4=BLM reset req.[62]", "5=IAC res. requested", "6=DTCs res. req.    ", "7=Not used          ");
      waitForHelpKey();
      break;
    case 9:
      lcdSlam("0=Not used      [63]", "1=Enable learn store", "2=Not used          ", "3=Not used          ");
      waitForHelpKey();
      lcdSlam("4=VSS failure   [63]", "5=Slow O2 rich/lean ", "6=Rich              ", "7=Closed loop       ");
      waitForHelpKey();
      break;
  }
  lcd.clear();
}



void setup() {

  pinMode(REDLED, OUTPUT);
  pinMode(GREENLED, OUTPUT);
  pinMode(BLUELED, OUTPUT);
  MAKE_LED_RED;
  delay(250);
  MAKE_LED_GREEN;
  delay(250);
  MAKE_LED_BLUE;
  delay(250);
  MAKE_LED_BLACK;
  delay(250);
  MAKE_LED_WHITE;

  lcd.begin(20,4);
  lcdSlam("     +--------+     ", "     |  ALDL  |     ", "     | Reader |     ", "     +--------+     ");
  pinMode(ALDLTestPin,INPUT);                                               // Define input pin to listen for the 160 baud input data
  Serial1.begin(8192);
  delay(1500);                                                              // Delay for diagnostic print
  pinMode(UPKEYPIN, INPUT_PULLUP);
  pinMode(DOWNKEYPIN, INPUT_PULLUP);
  pinMode(HELPKEYPIN, INPUT_PULLUP);
  pinMode(CANCELKEYPIN, INPUT_PULLUP);
  lcdSlam("Ready               ", "Not connected       ", clearLine, clearLine);
  i=0;                                                                      // Reset the preamble index flag
  if(digitalRead(CANCELKEYPIN)) {
    clearRequested = false;
  } else {
    clearRequested = true;
  }
}



float convertFtoC(float F) {
  return (F-32.0) / 1.8;
}



void loop() {

  bool upKeyPinState, downKeyPinState, helpKeyPinState, cancelKeyPinState;

#ifdef OFFLINE
  goto OfflineFakeData;
#endif

  // Wait for silence period on the ALDL
  SilenceFound = false;                                                     // Reset silence flag
  StartTime= micros();                                                      // First look for an active signal or a timeout - intialize timer

  while ((micros() - StartTime)< 15000) {                                   // Should exit this loop on the start of a bit. Wait for a 15 ms silent period 
    if (digitalRead(ALDLTestPin)== 0)                                       // Any line activity resets the start time
     {
       StartTime= micros();                                                 // Timing starts over
     }
  }                                                             
  SilenceFound = true;                                                      // Set the silence flag on exit
  MAKE_LED_BLACK;

  while (SilenceFound == true) {                                            // While silence found flag is set, continuously request and transmit Mode 1 data 
    PreambleFound = false;                                                  // Reset preamble found flag                                                             
 
    while (PreambleFound == false) {                                        // First look at data until the preamble has been found, will read data forever until a preamble is read
      i=0;                                                                  // Use bytecounter to 
      if(clearRequested) {
        //MAKE_LED_MAGENTA;
        while (i<4) {
          Serial1.write(M10Cmd[i]);                                         // Send the Mode 10 (clear codes) command sequence
          i++;
        }
        lcdSlam("Clearing error codes", "Request sent        ", clearLine, clearLine);
        delay(250);
        clearRequested = false;
        return;
      } else {
        while (i<4) {
          Serial1.write(M1Cmd[i]);                                         // Send the Mode 1 (read data at 8192 baud) command sequence
          i++;
        }
      }
 
      i=0;                                                                  // Then reset byte counter and scan incoming data for the preamble
      PreambleTimer = millis();                                             // Initialize timer to detect timeout
      // First look at data for 100 ms or until the preamble has been found will read data forever until a preamble is read
      while ((((millis() - PreambleTimer) < 100)) && (PreambleFound == false)) {

        if (Serial1.available() > 0) {                                      // Check for available data on the serial port
          ALDLbyte = Serial1.read();                                        // Read it and look for the preamble
          if ( ALDLbyte == Preamble[i]) {                                   // Look for matching byte of data preamble in serial stream 
            i++;                                                            // Increment the preamble index and look for the next character
            if (i>2)PreambleFound = true;                                   // Once three characters are found, the preamble has been found, time to read in the data
          } else {                                                          // If there does not match, start over looking from the beginning
            PreambleFound = false;                                          // Reset preamble found flag
            i=0;                                                            // Reset the preamble index flag          
          }                                                                 // End of Preamble check
        }                                                                   // End of Serial Available & read
      }                                                                     // End of the preamble finding routine either preamble found or timeout          
    }

    //MAKE_LED_YELLOW;
                                                                            // While loop only exits after a valid preamble is found - *** need to add a timeout option here later
    // Read a complete data stream packet
    DataStreamIndex = 1;                                                    // Once a valid preamble has been found set the data stream index to the first byte and read data up to byte #63 + 1 byte Checksum = 64 Bytes total
    while (DataStreamIndex < 65) {
      if (Serial1.available() > 0) {                                        // Check for available data on the serial port
        DataBytes[DataStreamIndex] = Serial1.read();                        // And read bytes into the array as they come
        DataStreamIndex++;                                                  // Update the index
      }                                                                     // End of read if
    }                                                                       // End of datastream read while - reads 63 bytes of data and skips the checksum byte for new...
    
    // Checksum Calculation  - Checksum Byte = 0x200 - Sum of all bytes transmitted including the Preamble
    i=1;                                                                    // Use bytecounter as an index
    CheckTotal = 0x80+0x95+0x01;                                            // First start by summing the preamble bytes
    while (i< (ByteCount)) {                                                // Add recived bytes to the Checksum except the Checksum byte itself
      CheckTotal = CheckTotal + DataBytes[i];                               // Add a byte
      i++;
    }   
    CheckSum = 0x200 - CheckTotal;                                          // Two's complement the checksum and cast it to a byte
    // Once the data stream has been read, resend it to the the outgoing serial port

    if (CheckSum != DataBytes[ByteCount]) {                                 // Last byte (64'th) transmitted is ECM's checksum - verify Checksums match and print good data or error message
      lcdSlam("ERROR: Bad checksum \0","\0","\0","\0");
      MAKE_LED_RED;
      delay(500);
    } else {
      MAKE_LED_BLUE;
    }

#ifdef OFFLINE
OfflineFakeData:
    for(i=0; i<ByteCount; i++) {
      DataBytes[i] = random(0,255);
    }
#endif

    upKeyPinState = digitalRead(UPKEYPIN);
    downKeyPinState = digitalRead(DOWNKEYPIN);
    helpKeyPinState = digitalRead(HELPKEYPIN);
    cancelKeyPinState = digitalRead(CANCELKEYPIN);

    if(key_was_pressed == 0) {
        if(!upKeyPinState) {
          key_was_pressed = KEYDELAY;
          pageNum++;
          if(pageNum>LASTPAGE) {
            pageNum=0;
          }
        } else {
          if(!downKeyPinState) {
            key_was_pressed = KEYDELAY;
            pageNum--;
            if(pageNum<0) {
              pageNum=LASTPAGE;
            }
          }
        }
    } else if(upKeyPinState && downKeyPinState) {
      if(key_was_pressed!=0) {
        key_was_pressed--;
      }
    }

    if(!helpKeyPinState) {
      help();
    }

    switch(pageNum) {
      case 0: // Diagnostic Trouble Codes
        //Screen layout:
        //+ 01234567890123456789
        //0 Errors  |23|22|21|15
        //1 14|13|12|34|33|32|25
        //2 24|51|46|45|44|43|42
        //3 41|36|62|55|54|53|52
        readDTCs(DataBytes[3], DataBytes[4], DataBytes[5], DataBytes[6], DataBytes[7]);
        sprintf(obuf0, "Errors  |  |  |  |  ");
        if(dtcFlags[DTC23]) { memcpy(&obuf0[9], "23", 2); }
        if(dtcFlags[DTC22]) { memcpy(&obuf0[12], "22", 2); }
        if(dtcFlags[DTC21]) { memcpy(&obuf0[15], "21", 2); }
        if(dtcFlags[DTC15]) { memcpy(&obuf0[18], "15", 2); }
        sprintf(obuf1, "  |  |  |  |  |  |  ");
        if(dtcFlags[DTC14]) { memcpy(&obuf1[0], "14", 2); }
        if(dtcFlags[DTC13]) { memcpy(&obuf1[3], "13", 2); }
        if(dtcFlags[DTC12]) { memcpy(&obuf1[6], "12", 2); }
        if(dtcFlags[DTC34]) { memcpy(&obuf1[9], "34", 2); }
        if(dtcFlags[DTC33]) { memcpy(&obuf1[12], "33", 2); }
        if(dtcFlags[DTC32]) { memcpy(&obuf1[15], "32", 2); }
        if(dtcFlags[DTC25]) { memcpy(&obuf1[18], "25", 2); }
        sprintf(obuf2, "  |  |  |  |  |  |  ");
        if(dtcFlags[DTC24]) { memcpy(&obuf2[0], "24", 2); }
        if(dtcFlags[DTC51]) { memcpy(&obuf2[3], "51", 2); }
        if(dtcFlags[DTC46]) { memcpy(&obuf2[6], "46", 2); }
        if(dtcFlags[DTC45]) { memcpy(&obuf2[9], "45", 2); }
        if(dtcFlags[DTC44]) { memcpy(&obuf2[12], "44", 2); }
        if(dtcFlags[DTC43]) { memcpy(&obuf2[15], "43", 2); }
        if(dtcFlags[DTC42]) { memcpy(&obuf2[18], "42", 2); }
        sprintf(obuf3, "  |  |  |  |  |  |  ");
        if(dtcFlags[DTC41]) { memcpy(&obuf3[0], "41", 2); }
        if(dtcFlags[DTC36]) { memcpy(&obuf3[3], "36", 2); }
        if(dtcFlags[DTC62]) { memcpy(&obuf3[6], "62", 2); }
        if(dtcFlags[DTC55]) { memcpy(&obuf3[9], "55", 2); }
        if(dtcFlags[DTC54]) { memcpy(&obuf3[12], "54", 2); }
        if(dtcFlags[DTC53]) { memcpy(&obuf3[15], "53", 2); }
        if(dtcFlags[DTC52]) { memcpy(&obuf3[18], "52", 2); }
        break;
      case 1: // Engine management parameters and sensors (1/5)
        //Screen layout:
        //+ 01234567890123456789
        //0 ****RPM ****mV-TPS
        //1 ***.*g/s-MAF***°C-AT
        //2 ***BLM ***LT ***ST
        //3 ***ms-inj. ****mV-O2
        RPM = DataBytes[11] * 25;                                                   // Engine RPM
        TPS = DataBytes[10] * 19.608;                                               // TPS millivolts
        MAF = 0.003906 * (unsigned int)((DataBytes[36]*256) + (DataBytes[37]));     // Mass Air Flow g/s
        MAT = T_interpolate(DataBytes[30]);                                         // Intake Manifold Air Temperature
        BLCELL = DataBytes[21];                                                     // Block Learn Cell # being used
        BLM = DataBytes[20];                                                        // Long Term Fuel correction (value of current Block Learn Cell)
        INTEGRATOR = DataBytes[22];                                                 // Currently calculated fuel trim number
        injPW = 0.015259 * (unsigned int)((DataBytes[45]*256) + (DataBytes[46]));   // Injector Pulse Width in Msec
        O2mV = DataBytes[17] * 4.44;                                                // O2 sensor mV
        sprintf(obuf0, "%4dRPM %4dmV-TPS  ", (int)RPM, (int)TPS);
        dtostrf(MAF, 5, 1, strBufA);
        sprintf(obuf1, "%sg/s-MAF%3d%cC-AT", strBufA, (int)MAT, (char)223);
        sprintf(obuf2, "%3dBLM %3dLT %3dST  ", (int)BLCELL, (int)BLM, (int)INTEGRATOR);
        sprintf(obuf3, "%3dms-inj. %4dmV-O2", (int)injPW, (int)O2mV);
        break;
      case 2: // Engine management parameters and sensors (2/5)
        //Screen layout:
        //+ 01234567890123456789
        //0 ***°C-CT,***°C-start
        //1 ***MPH ***O2 crosses
        //2 ***IACpos ***IACdir
        //3 ****RPM-target idle 
        ECT = convertFtoC(DataBytes[8] * 1.35 - 40.0);
        ECT0 = convertFtoC(DataBytes[9] * 1.35 - 40.0);
        MPH = DataBytes[14];
        O2Crosses = DataBytes[18];
        IACPos = DataBytes[23];
        IACDir = DataBytes[24];
        RPMTargetIdle = DataBytes[25] * 12.5;
        sprintf(obuf0, "%3d%cC-CT,%3d%cC-start", (int)ECT, (char)223, (int)ECT0, (char)223);
        sprintf(obuf1, "%3dMPH %3dO2 crosses", (int)MPH, (int)O2Crosses);
        sprintf(obuf2, "%3dIACpos %3dIACdir", (int)IACPos, (int)IACDir);
        sprintf(obuf3, "%4dRPM-target idle ", (int)RPMTargetIdle);
        break;
      case 3: // Engine management parameters and sensors (3/5)
        //Screen layout:
        //+ 01234567890123456789
        //0 ***-Ld,filtered ***%
        //1 ***-Load,raw    ***%
        //2 ***%DC-EGR***%DC-fan
        //3 ***%DC-canister     
        loadFiltered = DataBytes[26];
        loadFilteredPc = loadFiltered * 0.3515625;                          // 0.3515625 == 100 / 255
        loadRaw = DataBytes[28];
        loadRawPc = loadRaw * 0.3515625;                                    // 0.3515625 == 100 / 255
        EGRDc = DataBytes[31] * 0.3515625;                                  // 0.3515625 == 100 / 255
        canDc = DataBytes[32] * 0.3515625;                                  // 0.3515625 == 100 / 255
        fanDc = DataBytes[33] * 0.3515625;                                  // 0.3515625 == 100 / 255
        sprintf(obuf0, "%3d-ld,filtered %3d%%", (int)loadFiltered, (int)loadFilteredPc);
        sprintf(obuf1, "%3d-load,raw    %3d%%", (int)loadRaw, (int)loadRawPc);
        sprintf(obuf2, "%3d%%DC-EGR%3d%%DC-fan", (int)EGRDc, (int)fanDc);
        sprintf(obuf3, "%3d%%DC-canister     ", (int)canDc);
        break;
      case 4: // Engine management parameters and sensors (4/5)
        //Screen layout:
        //+ 01234567890123456789
        //0 **.*V-bat **.*V-pump
        //1 ***%-MAF raw        
        //2 ***°-spark TDC      
        //3 ***°-spark ref. pul.
        Vbat = DataBytes[34] * 0.1;
        Vpump = DataBytes[35] * 0.1;
        MAFRaw = DataBytes[38] * 0.3906;
        sparkTempB = (((int)DataBytes[39]<<8)|(int)DataBytes[40]);
        sparkTDC = 0.3515625 * sparkTempB;
        sparkTempB = (((int)DataBytes[39]<<8)|(int)DataBytes[40]);
        sparkRefPulse = 0.3515625 * sparkTempB;
        dtostrf(Vbat, 4, 1, strBufA);
        dtostrf(Vpump, 4, 1, strBufB);
        sprintf(obuf0, "%sV-bat %sV-pump", strBufA, strBufB);
        sprintf(obuf1, "%3d%%-MAF raw        ", (int)MAFRaw);
        sprintf(obuf2, "%+3d%c-spark TDC      ", (int)sparkTDC, (char)223);
        sprintf(obuf3, "%3d%c-spark ref. pul.", (int)sparkRefPulse, (char)223);
        break;
      case 5: // Engine management parameters and sensors (5/5)
        //Screen layout:
        //+ 01234567890123456789
        //0 ***-ESC **.*°-retard
        //1 ROM ID MS: 0x**     
        //2        LS: 0x**     
        //3 *****s-run time     
        ESCSignal = DataBytes[43];
        sparkRetard = DataBytes[44] * 0.1;
        runTime = (DataBytes[52]*256 + DataBytes[53]);
        dtostrf(sparkRetard, 4, 1, strBufA);
        sprintf(obuf0, "%3d-ESC %s%c-retard", (int)ESCSignal, strBufA, (char)223);
        sprintf(obuf1, "ROM ID MS: 0x%X     ", (int)DataBytes[1]);
        sprintf(obuf2, "       LS: 0x%X     ", (int)DataBytes[2]);
        sprintf(obuf3, "%5us-run time     ", (unsigned int)runTime);
        break;
      case 6:  // Miscellaneous 'flags' or bitmap 'words' (1/4)
        //Screen layout:
        //+ 01234567890123456789
        //0 Word 0 1 2 3 4 5 6 7
        //1 [54] * * * * * * * *
        //2 [55] * * * * * * * *
        //3 [56] * * * * - * * *
        sprintf(obuf0, "Word 0 1 2 3 4 5 6 7");
        flagsInput=(unsigned char)DataBytes[54];
        flag0 = 48+(flagsInput&0x01);
        flag1 = 48+((flagsInput&0x02)>>1);
        flag2 = 48+((flagsInput&0x04)>>2);
        flag3 = 48+((flagsInput&0x08)>>3);
        flag4 = 48+((flagsInput&0x10)>>4);
        flag5 = 48+((flagsInput&0x20)>>5);
        flag6 = 48+((flagsInput&0x40)>>6);
        flag7 = 48+((flagsInput&0x80)>>7);
        sprintf(obuf1, "[54] %c %c %c %c %c %c %c %c", flag0, flag1, flag2, flag3, flag4, flag5, flag6, flag7);
        flagsInput=(unsigned char)DataBytes[55];
        flag0 = 48+(flagsInput&0x01);
        flag1 = 48+((flagsInput&0x02)>>1);
        flag2 = 48+((flagsInput&0x04)>>2);
        flag3 = 48+((flagsInput&0x08)>>3);
        flag4 = 48+((flagsInput&0x10)>>4);
        flag5 = 48+((flagsInput&0x20)>>5);
        flag6 = 48+((flagsInput&0x40)>>6);
        flag7 = 48+((flagsInput&0x80)>>7);
        sprintf(obuf2, "[55] %c %c %c %c %c %c %c %c", flag0, flag1, flag2, flag3, flag4, flag5, flag6, flag7);
        flagsInput=(unsigned char)DataBytes[56];
        flag0 = 48+(flagsInput&0x01);
        flag1 = 48+((flagsInput&0x02)>>1);
        flag2 = 48+((flagsInput&0x04)>>2);
        flag3 = 48+((flagsInput&0x08)>>3);
        flag5 = 48+((flagsInput&0x20)>>5);
        flag6 = 48+((flagsInput&0x40)>>6);
        flag7 = 48+((flagsInput&0x80)>>7);
        sprintf(obuf3, "[56] %c %c %c %c %c %c %c %c", flag0, flag1, flag2, flag3, '-', flag5, flag6, flag7);
        break;
      case 7: // Miscellaneous 'flags' or bitmap 'words' (2/4)
        //Screen layout:
        //+ 01234567890123456789
        //0 Word 0 1 2 3 4 5 6 7
        //1 [57] * * * * * * * *
        //2 [58] * * - * - * * *
        //3 [59] * * * * * * * *
        sprintf(obuf0, "Word 0 1 2 3 4 5 6 7");
        flagsInput=(unsigned char)DataBytes[57];
        flag0 = 48+(flagsInput&0x01);
        flag1 = 48+((flagsInput&0x02)>>1);
        flag2 = 48+((flagsInput&0x04)>>2);
        flag3 = 48+((flagsInput&0x08)>>3);
        flag4 = 48+((flagsInput&0x10)>>4);
        flag5 = 48+((flagsInput&0x20)>>5);
        flag6 = 48+((flagsInput&0x40)>>6);
        flag7 = 48+((flagsInput&0x80)>>7);
        sprintf(obuf1, "[57] %c %c %c %c %c %c %c %c", flag0, flag1, flag2, flag3, flag4, flag5, flag6, flag7);
        flagsInput=(unsigned char)DataBytes[58];
        flag0 = 48+(flagsInput&0x01);
        flag1 = 48+((flagsInput&0x02)>>1);
        flag3 = 48+((flagsInput&0x08)>>3);
        flag5 = 48+((flagsInput&0x20)>>5);
        flag6 = 48+((flagsInput&0x40)>>6);
        flag7 = 48+((flagsInput&0x80)>>7);
        sprintf(obuf2, "[58] %c %c %c %c %c %c %c %c", flag0, flag1, '-', flag3, '-', flag5, flag6, flag7);
        flagsInput=(unsigned char)DataBytes[59];
        flag0 = 48+(flagsInput&0x01);
        flag1 = 48+((flagsInput&0x02)>>1);
        flag2 = 48+((flagsInput&0x04)>>2);
        flag3 = 48+((flagsInput&0x08)>>3);
        flag4 = 48+((flagsInput&0x10)>>4);
        flag5 = 48+((flagsInput&0x20)>>5);
        flag6 = 48+((flagsInput&0x40)>>6);
        flag7 = 48+((flagsInput&0x80)>>7);
        sprintf(obuf3, "[59] %c %c %c %c %c %c %c %c", flag0, flag1, flag2, flag3, flag4, flag5, flag6, flag7);
        break;
      case 8: // Miscellaneous 'flags' or bitmap 'words' (3/4)
        //Screen layout:
        //+ 01234567890123456789
        //0 Word 0 1 2 3 4 5 6 7
        //1 [60] * * * * * * * *
        //2 [61] * * * * * * * *
        //3 [62] * * * * * * * -
        sprintf(obuf0, "Word 0 1 2 3 4 5 6 7");
        flagsInput=(unsigned char)DataBytes[60];
        flag0 = 48+(flagsInput&0x01);
        flag1 = 48+((flagsInput&0x02)>>1);
        flag2 = 48+((flagsInput&0x04)>>2);
        flag3 = 48+((flagsInput&0x08)>>3);
        flag4 = 48+((flagsInput&0x10)>>4);
        flag5 = 48+((flagsInput&0x20)>>5);
        flag6 = 48+((flagsInput&0x40)>>6);
        flag7 = 48+((flagsInput&0x80)>>7);
        sprintf(obuf1, "[60] %c %c %c %c %c %c %c %c", flag0, flag1, flag2, flag3, flag4, flag5, flag6, flag7);
        flagsInput=(unsigned char)DataBytes[61];
        flag0 = 48+(flagsInput&0x01);
        flag1 = 48+((flagsInput&0x02)>>1);
        flag2 = 48+((flagsInput&0x04)>>2);
        flag3 = 48+((flagsInput&0x08)>>3);
        flag4 = 48+((flagsInput&0x10)>>4);
        flag5 = 48+((flagsInput&0x20)>>5);
        flag6 = 48+((flagsInput&0x40)>>6);
        flag7 = 48+((flagsInput&0x80)>>7);
        sprintf(obuf2, "[61] %c %c %c %c %c %c %c %c", flag0, flag1, flag2, flag3, flag4, flag5, flag6, flag7);
        flagsInput=(unsigned char)DataBytes[62];
        flag0 = 48+(flagsInput&0x01);
        flag1 = 48+((flagsInput&0x02)>>1);
        flag2 = 48+((flagsInput&0x04)>>2);
        flag3 = 48+((flagsInput&0x08)>>3);
        flag4 = 48+((flagsInput&0x10)>>4);
        flag5 = 48+((flagsInput&0x20)>>5);
        flag6 = 48+((flagsInput&0x40)>>6);
        sprintf(obuf3, "[62] %c %c %c %c %c %c %c %c", flag0, flag1, flag2, flag3, flag4, flag5, flag6, '-');
        break;
      case 9: // Miscellaneous 'flags' or bitmap 'words' (4/4)
        //Screen layout:
        //+ 01234567890123456789
        //0 Word 0 1 2 3 4 5 6 7
        //1 [63] - * - - * * * *
        //2                     
        //3                     
        sprintf(obuf0, "Word 0 1 2 3 4 5 6 7");
        flagsInput=(unsigned char)DataBytes[63];
        flag1 = 48+((flagsInput&0x02)>>1);
        flag2 = 48+((flagsInput&0x04)>>2);
        flag3 = 48+((flagsInput&0x08)>>3);
        flag4 = 48+((flagsInput&0x10)>>4);
        flag5 = 48+((flagsInput&0x20)>>5);
        flag6 = 48+((flagsInput&0x40)>>6);
        flag7 = 48+((flagsInput&0x80)>>7);
        sprintf(obuf1, "[63] %c %c %c %c %c %c %c %c", '-', flag1, '-', '-', flag4, flag5, flag6, flag7);
        sprintf(obuf2, clearLine);
        sprintf(obuf3, clearLine);
        break;
      default:
        pageNum=0;
    }
    
    lcdSlam(obuf0, obuf1, obuf2, obuf3);
    MAKE_LED_BLACK;

    // The following code implements a delay with an incoming serial buffer purge to allow it to run alongside other data captures such as TunerPro - Artduino Tx1 is disconnected
    // Not keeping the incoming serial buffer empty before the next command and read loop causes a parity error
    unsigned long StartTime = millis();                                   // Get the starting time value
    while (millis() < StartTime+BUFFERDELAY) {                            // Wait for N milliseconds - allows user to read the displayed data
      if (Serial1.available() > 0) ALDLbyte = Serial1.read();             // Check for available data on the serial port and throw it away - i.e. flush the buffer
    }  

    if(!cancelKeyPinState) {
      lcdSlam("Clearing error codes", clearLine, clearLine, clearLine);
      MAKE_LED_RED;
      //delay(250);
      clearRequested = true;
      SilenceFound = false;
      return;
    }
  }                                                                       // Loop back for next Mode 1 Dump
}
