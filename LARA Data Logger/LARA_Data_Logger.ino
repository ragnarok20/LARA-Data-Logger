//
// LARA Data Logger
//
// Records Distance Travelled & Lever Repetitions
// Developed with [embedXcode](http://embedXcode.weebly.com)
//
// Author 		Sage Thayer
// 				Sage Thayer
//
// Date			12/16/15 11:15 AM
// Version		1.0
//
// Copyright	© Sage Thayer, 2015
// Licence		<#licence#>
//
// See         ReadMe.txt for references
//


// Core library for code-sense - IDE-based
#if defined(WIRING) // Wiring specific
#   include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#   include "WProgram.h"
#elif defined(MPIDE) // chipKIT specific
#   include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#   include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
#   include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
#   include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
#   include "Arduino.h"
#elif defined(SPARK) || defined(PARTICLE) // Particle / Spark specific
#   include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
#   include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
#   include "Arduino.h"
#elif defined(ESP8266) // ESP8266 specific
#   include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#   include "Arduino.h"
#else // error
#   error Platform not defined
#endif // end IDE

// Include application, user and local libraries
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include "MPU9250.h"
#include "I2Cdev.h"
#include <RTClib.h>
#include <avr/sleep.h>
#include <avr/wdt.h>    

//---- Defines for different outputs ------//
//#define ECHO        // Sets up a serial Monitor at 9600 baud
#define LOG      //logging macro define (takes a bit of memory to log)
//#define SET_RTC       //resets RTC time and date to the time of this compilation

//#define logFreq 0.01667f
//#define logFreq 1.0f
#define logFreq 30.0f
#define sampleFreq 30.0f		// sample frequency in Hz

//int logFreq = 30;
//int sampleFreq = 30;
#define SAMPLE_RATE_DIV (1000/sampleFreq) - 1



#define betaDef		.01f		// 2 * proportional gain

//----------- Prototypes --------------//
void sleepNow();
void halfSleep();        //sort of sleep for tests. will slow donw sample rate to 1hz & loging rate to 1hz
void wakeUpNow();   //ISR
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
float invSqrt(float x);
void IMU_int();


//-----------Define variables and constants-----------//
const uint8_t MY_LED = 13;
// variables to figure out cycle rate
double begin_of_loop = 0;
double loop_time = 0;
double measured_cycle_rate = sampleFreq;
int delay_time = 0;

//----------SD Card & Logging-------------------//
// set up variables using the SD utility library functions:

bool SD_card_present = false;
// the logging file
String filenameString;
char filename[13];      // limited to 12 characters with FAT partitioned SD card...
char README[] = "README.txt";
File readme;
File logfile;
double logTime = 0;

// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
const int chipSelect = 10; //chip select for SD card

//------------------MPU-9250-------------------//
//9dof board
const int NCS1 = 8;   //chip select for mpu9250 1 Right
const int NCS2 = 9;   //chip select for mpu9250 2 Left
MPU9250 IMU1(NCS1,1000000);   //initialize for SPI at 1 MHz for 1st IMU
MPU9250 IMU2(NCS2,1000000);   //initialize for SPI at 1 MHz for 2nd IMU

bool IMU1_online = false;
bool IMU2_online = false;

extern volatile float beta;				// algorithm gain
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

int16_t ax, ay, az;
int16_t gx, gy, gz;

int16_t ax2, ay2, az2;
int16_t gx2, gy2, gz2;

double pitch, yaw, roll;
double pitch2, yaw2, roll2;

// --------- Sleep Handeling -----------//
volatile bool mpu_int_triggered = false;
double motionTimer = 0;
double motionTimePeriod = 6000;       // period of no motion to shutdown/stop logging (currently 1 minute)
byte adcsra, mcucr1, mcucr2;           //sleep register place holders
volatile bool wdt_int = false;
void wdt_int_reset();                   // will reset wdt and int flag


//---------RTC-----------------//
RTC_DS1307 RTC;
// a variable to reference miliseconds to to have a millisecond counter relative to the hour
double millisAtTopHour = 0;
int prevHour;
bool relMillisSet = false;

//-------- Reed Switch Encoder ----------//
// 2 reed switches are used
//input pull up applied so 1 is no read; 0 is magnet read

const int reedLeft[2] = {7,6};      // 7 is white wire
const int reedRight[2] = {5,4};     // 5 is white wire
bool reed_stat_left[2];
bool reed_stat_right[2];

// will help the logic with the reed switch states
typedef enum {UNDETERMINED, FORWARD, REVERSE, RELEASE_FORWARD, RELEASE_REVERSE} direction;
direction WheelLeft = UNDETERMINED;
direction WheelRight = UNDETERMINED;
direction ReedTriggeredLeft = UNDETERMINED;
direction ReedTriggeredRight = UNDETERMINED;

// The 4 possible states of the 2 reed switches per side
typedef enum {NONE, FIRST, BOTH, LAST} state;
state EncoderLeft = NONE;
state EncoderRight = NONE;

// rotation counters
float WheelLeftRot = 0;
float WheelRightRot = 0;

float wheelResolution = 4;    // how many magnets on the wheel.

//----------- Setup ----------------/
void setup()
{
    //trigger reset on start just in case
    wdt_int_reset();
    
    pinMode(13, OUTPUT);
    pinMode(reedLeft[0], INPUT_PULLUP);
    pinMode(reedLeft[1], INPUT_PULLUP);
    pinMode(reedRight[0], INPUT_PULLUP);
    pinMode(reedRight[1], INPUT_PULLUP);
    
    Wire.begin();
    
#ifdef ECHO
    // Open serial communications and wait for port to open:
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }
#endif
    
    //---------- Set up RTC --------------------//
    RTC.begin();
    
    if (!RTC.isrunning()) {
        // following line sets the RTC to the date & time this sketch was compiled
        RTC.adjust(DateTime(__DATE__, __TIME__));
    }
    
#ifdef SET_RTC
    RTC.adjust(DateTime(__DATE__, __TIME__));
#endif
    
    
    //--------IMU initialization------------//
#ifdef  ECHO
    Serial.println("Initializing MPU & Calibrating Offsets");
#endif
    //IMU int pin ISR for motion detection
    attachInterrupt(0,IMU_int, RISING); // use interrupt 0 (pin 2) and run function
    
    IMU1.initialize();
    IMU2.initialize();

    // check the who am i register to see if the IMU is responding
    if(IMU1.testConnection()) {
        IMU1_online = true;
        IMU1.calibrateGyro();   //DONT move the MPU when Calibrating
        IMU1.initialize();      // reinitialize to reset sample rate dividers and DLPFs....
        IMU1.setFullScaleGyroRange(MPU9250_GYRO_FS_2000);
        IMU1.setRate((uint8_t)SAMPLE_RATE_DIV);
        IMU1.setMotionIntProcess();
    }
    else {
        IMU1_online = false;
    }
    
    // check the who am i register to see if the IMU is responding
    if(IMU2.testConnection()) {
        IMU2_online = true;
        IMU2.calibrateGyro();   //DONT move the MPU when Calibrating
        IMU2.initialize();      // reinitialize to reset sample rate dividers and DLPFs....
        IMU2.setFullScaleGyroRange(MPU9250_GYRO_FS_2000);
        IMU2.setRate((uint8_t)SAMPLE_RATE_DIV);
        IMU2.setMotionIntProcess(); //sets the INT pin on the MPU to send an interrupt when it senses motion
    }
    else {
        IMU2_online = false;
    }
    

    //--------- Set up Logging -----------------//
    
    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
        SD_card_present = false;
    }
    else {
        SD_card_present = true;
    }

    // Look for a readme and create if it does not exist
    /*
    if(! SD.exists(README)) {
        readme = SD.open(README,FILE_WRITE);
        
        readme.println("Each CSV file will contain data for that hour if the chair is operated.");
        readme.println("Each CSV file is named beginning with the day-month followed by an h to indicate the hour.");
        readme.print("TODO: EXPLAIN DATA STRUCTURE");
        
        readme.close();
    }
*/
    
    // create a new file
    DateTime now = RTC.now();
    String Day = String(now.day());
    String Month = String(now.month());
    String Hour = String(now.hour());

    filenameString = String(Month+"-"+Day+"-"+Hour+".csv");
    
    filenameString.toCharArray(filename, sizeof(filename));
  
    /*
    if (! SD.exists(filename)) {
        // only open a new file if it doesn't exist
        //const char *FILENAME = filename;
        logfile = SD.open(filename, FILE_WRITE);
    }
    */
    
    
    // ------------ Screen output ------------------//
#ifdef ECHO
    
    //RTC Flags
    if (!RTC.isrunning()) {
        Serial.println("RTC is not set... setting to this compile time and date");
        // following line sets the RTC to the date & time this sketch was compiled
        RTC.adjust(DateTime(__DATE__, __TIME__));
    }
    else {
        Serial.println("RTC Running and Set");
    }
    
    // IMU 1 online flag
    if(IMU1_online) {
        Serial.println("MPU_9250 1 Online");
    }
    else {
        Serial.println("MPU_9250 1 not responding");
    }
    
    // IMU 2 online flag
    if(IMU2_online) {
        Serial.println("MPU_9250 2 Online");
    }
    else {
        Serial.println("MPU_9250 2 not responding");
    }
    
    // Check for SD card flags
    if(SD_card_present) {
        Serial.println("SD Card Online");
    }
    else {
        Serial.println("No SD Card");
    }
    
    delay(5000);
    
#endif
    
    // -------------- Initialize WDT and reset-------------------//
    wdt_int_reset();

    
}

//----------- loop ----------------//
void loop()
{
    wdt_int_reset();    //reset wdt

    
    begin_of_loop = millis();
    
    //----------- IMU Get Data & process ----------------//
    
    if(IMU1_online) {
        //IMU1.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        IMU1.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        //IMU1.getRotation(&gx, &gy, &gz);
        
        
        ax = ax*2.0f/32768.0f; // 2 g full range for accelerometer
        ay = ay*2.0f/32768.0f;
        az = az*2.0f/32768.0f;
        
        gx = gx*2000.0f/32768.0f; // 250 deg/s full range for gyroscope
        gy = gy*2000.0f/32768.0f;
        gz = gz*2000.0f/32768.0f;
        /*
        mx = mx*10.0f*1229.0f/4096.0f; // milliGauss (1229 microTesla per 2^12 bits, 10 mG per microTesla)
        my = my*10.0f*1229.0f/4096.0f; // apply calibration offsets in mG that correspond to your environment and magnetometer
        mz = mz*10.0f*1229.0f/4096.0f;
        */
      /*
        gx = gx*PI/180.0f;
        gy = gy*PI/180.0f;
        gz = gz*PI/180.0f;
        
        //update quanternion
        MadgwickAHRSupdateIMU( gx, gy, gz, ax, ay, az);
        
        //calculate euler
        
        yaw   = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
        pitch = -asin(2.0f * (q1 * q3 - q0 * q2));
        roll  = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
        pitch = pitch * 180.0f / PI;
        yaw   = yaw *180.0f / PI - 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
        roll  = roll * 180.0f / PI;
        
        
        //integrate gyro rate to get angle
        */
        pitch += gx*1/measured_cycle_rate;
        roll += gy*1/measured_cycle_rate;
        yaw += gz*1/measured_cycle_rate;
        
    }

    if(IMU2_online) {
        
        IMU2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
        
        
        ax2 = ax2*2.0f/32768.0f; // 2 g full range for accelerometer
        ay2 = ay2*2.0f/32768.0f;
        az2 = az2*2.0f/32768.0f;
        
        gx2 = gx2*2000.0f/32768.0f; // 250 deg/s full range for gyroscope
        gy2 = gy2*2000.0f/32768.0f;
        gz2 = gz2*2000.0f/32768.0f;
        /*
         mx = mx*10.0f*1229.0f/4096.0f; // milliGauss (1229 microTesla per 2^12 bits, 10 mG per microTesla)
         my = my*10.0f*1229.0f/4096.0f; // apply calibration offsets in mG that correspond to your environment and magnetometer
         mz = mz*10.0f*1229.0f/4096.0f;
         */
        /*
        gx2 = gx2*PI/180.0f;
        gy2 = gy2*PI/180.0f;
        gz2 = gz2*PI/180.0f;
        
        //update quanternion
        MadgwickAHRSupdateIMU( gx2, gy2, gz2, ax2, ay2, az2);
        
        //calculate euler
    
        yaw2   = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
        pitch2 = -asin(2.0f * (q1 * q3 - q0 * q2));
        roll2  = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
        pitch2 = pitch2 * 180.0f / PI;
        yaw2   = yaw2 *180.0f / PI - 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
        roll2  = roll2 * 180.0f / PI;
      */
        //integrate gyro to find absolute angle
        pitch2 += gx2*1/measured_cycle_rate;
        roll2 += gy2*1/measured_cycle_rate;
        yaw2 += gz2*1/measured_cycle_rate;
        

    }
    
    //----------- Reed Switch Data and Encoder Signal Processing LEFT Wheel ----------------//
    // read the reed switch
    reed_stat_left[0] = digitalRead(reedLeft[0]);
    reed_stat_left[1] = digitalRead(reedLeft[1]);

    // State of the Encoder
    if (reed_stat_left[0] == 1 && reed_stat_left[1] == 1) {
        EncoderLeft = NONE;
    }
    if (reed_stat_left[0] == 0 && reed_stat_left[1] == 1) {
        EncoderLeft = FIRST;
    }
    if (reed_stat_left[0] == 0 && reed_stat_left[1] == 0) {
        EncoderLeft = BOTH;
    }
    if (reed_stat_left[0] == 1 && reed_stat_left[1] == 0) {
        EncoderLeft = LAST;
    }
     //rotorary Encoder logic
    
    //Forward Trigger
    if (EncoderLeft == FIRST && ReedTriggeredLeft == UNDETERMINED) {
        WheelLeft = FORWARD;
    }
    if (EncoderLeft == BOTH && WheelLeft == FORWARD && ReedTriggeredLeft == UNDETERMINED) {
        ReedTriggeredLeft = FORWARD;
    }
    
    //Reverse Trigger
    if (EncoderLeft == LAST && ReedTriggeredLeft == UNDETERMINED) {
        WheelLeft = REVERSE;
    }
    if (EncoderLeft == BOTH && WheelLeft == REVERSE && ReedTriggeredLeft == UNDETERMINED) {
        ReedTriggeredLeft = REVERSE;
    }
    
    //forward release
    if (EncoderLeft == LAST && ReedTriggeredLeft == FORWARD) {
        ReedTriggeredLeft = RELEASE_FORWARD;
    }
    //reverse release
    if (EncoderLeft == FIRST && ReedTriggeredLeft == REVERSE) {
        ReedTriggeredLeft = RELEASE_REVERSE;
    }
    
    //forward count
    if (EncoderLeft == NONE && ReedTriggeredLeft == RELEASE_FORWARD) {
        ReedTriggeredLeft = UNDETERMINED;
        WheelLeftRot = WheelLeftRot + (1/wheelResolution);
        //WheelLeftRot++;
    }
    //reverse count
    if (EncoderLeft == NONE && ReedTriggeredLeft == RELEASE_REVERSE) {
        ReedTriggeredLeft = UNDETERMINED;
        WheelLeftRot = WheelLeftRot - (1/wheelResolution);
        //WheelLeftRot--;
    }
    
    // reset if something weird happens
    if (EncoderLeft == NONE) {
        ReedTriggeredLeft = UNDETERMINED;
    }
    
    //----------- Reed Switch Data and Encoder Signal Processing Right Wheel ----------------//
    // read the reed switch
    reed_stat_right[0] = digitalRead(reedRight[0]);
    reed_stat_right[1] = digitalRead(reedRight[1]);
    
    // State of the Encoder
    if (reed_stat_right[0] == 1 && reed_stat_right[1] == 1) {
        EncoderRight = NONE;
    }
    if (reed_stat_right[0] == 0 && reed_stat_right[1] == 1) {
        EncoderRight = FIRST;
    }
    if (reed_stat_right[0] == 0 && reed_stat_right[1] == 0) {
        EncoderRight = BOTH;
    }
    if (reed_stat_right[0] == 1 && reed_stat_right[1] == 0) {
        EncoderRight = LAST;
    }
    //rotorary Encoder logic
    
    //Forward Trigger
    if (EncoderRight == FIRST && ReedTriggeredRight == UNDETERMINED) {
        WheelRight = FORWARD;
    }
    if (EncoderRight == BOTH && WheelRight == FORWARD && ReedTriggeredRight == UNDETERMINED) {
        ReedTriggeredRight = FORWARD;
    }
    
    //Reverse Trigger
    if (EncoderRight == LAST && ReedTriggeredRight == UNDETERMINED) {
        WheelRight = REVERSE;
    }
    if (EncoderRight == BOTH && WheelRight == REVERSE && ReedTriggeredRight == UNDETERMINED) {
        ReedTriggeredRight = REVERSE;
    }
    
    //forward release
    if (EncoderRight == LAST && ReedTriggeredRight == FORWARD) {
        ReedTriggeredRight = RELEASE_FORWARD;
    }
    //reverse release
    if (EncoderRight == FIRST && ReedTriggeredRight == REVERSE) {
        ReedTriggeredRight = RELEASE_REVERSE;
    }
    
    //forward count
    if (EncoderRight == NONE && ReedTriggeredRight == RELEASE_FORWARD) {
        ReedTriggeredRight = UNDETERMINED;
        WheelRightRot = WheelRightRot + (1/wheelResolution);
        //WheelRightRot++;
    }
    //reverse count
    if (EncoderRight == NONE && ReedTriggeredRight == RELEASE_REVERSE) {
        ReedTriggeredRight = UNDETERMINED;
        WheelRightRot = WheelRightRot - (1/wheelResolution);
        //WheelRightRot--;
    }
    
    // reset if something weird happens
    if (EncoderRight == NONE) {
        ReedTriggeredRight = UNDETERMINED;
    }

    
   //------------------ SD Card Logging -------------------------------------//

#ifdef LOG
    //Log at the desired logging frequency
    if (((millis()-logTime)/1000) > (float)1/logFreq) {
        
        
        // create a new file
        DateTime now = RTC.now();
        String Day = String(now.day());
        String Month = String(now.month());
        String Hour = String(now.hour());
        
        filenameString = String(Month+"-"+Day+"-"+Hour+".csv");
        
        filenameString.toCharArray(filename, sizeof(filename));
    
        // open the file. note that only one file can be open at a time,
        // so you have to close this one before opening another.
        
        if (SD_card_present) {
            logfile = SD.open(filename, FILE_WRITE);
        }
        
        
        // if the file is available, write to it:
        if (logfile && RTC.isrunning()) {
            DateTime now = RTC.now();
            
            // reset millis every hour
            if(now.hour() != prevHour) {
                //upon a reset not at top of hour, will get a millisecond counter
                //accurate to the relative second of the hour.
                if(now.minute() || now.second()) {
                    millisAtTopHour = (long)(now.minute()*60000) + (long)(now.second() * 1000);
                    millisAtTopHour = -millisAtTopHour;
                }
                else {
                    millisAtTopHour = millis();
                }
                prevHour = now.hour();
            }

            // log time relative to hour
            logfile.print(now.minute());
            logfile.print(",");
            logfile.print(now.second());
            logfile.print(",");
            logfile.print((long)(millis()-millisAtTopHour));
            logfile.print(",");

            //----------- IMU Data -------------------//
            if (IMU1_online) {
                // Yaw angle onboard signal processing
                logfile.print(yaw);
                logfile.print(",");
                
                //raw gyro data
                logfile.print(gx);
                logfile.print(",");
                logfile.print(gy);
                logfile.print(",");
                logfile.print(gz);
                logfile.print(",");
                
                //raw accelerometer data
                logfile.print(ax);
                logfile.print(",");
                logfile.print(ay);
                logfile.print(",");
                logfile.print(az);
                logfile.print(",");
                
            }
            else {
                // Yaw angle onboard signal processing
                logfile.print("NA");
                logfile.print(",");
                
                //raw gyro data
                logfile.print("NA");
                logfile.print(",");
                logfile.print("NA");
                logfile.print(",");
                logfile.print("NA");
                logfile.print(",");
                
                //raw accelerometer data
                logfile.print("NA");
                logfile.print(",");
                logfile.print("NA");
                logfile.print(",");
                logfile.print("NA");
                logfile.print(",");
            }
            
            if (IMU2_online) {
                // Yaw angle onboard signal processing
                logfile.print(yaw2);
                logfile.print(",");
                
                //raw gyro data
                logfile.print(gx2);
                logfile.print(",");
                logfile.print(gy2);
                logfile.print(",");
                logfile.print(gz2);
                logfile.print(",");
                
                //raw accelerometer data
                logfile.print(ax2);
                logfile.print(",");
                logfile.print(ay2);
                logfile.print(",");
                logfile.print(az2);
                logfile.print(",");
                
            }
            else {
                // Yaw angle onboard signal processing
                logfile.print("NA");
                logfile.print(",");
                
                //raw gyro data
                logfile.print("NA");
                logfile.print(",");
                logfile.print("NA");
                logfile.print(",");
                logfile.print("NA");
                logfile.print(",");
                
                //raw accelerometer data
                logfile.print("NA");
                logfile.print(",");
                logfile.print("NA");
                logfile.print(",");
                logfile.print("NA");
                logfile.print(",");
            }
            
            //------------- Reed Data ----------------//
            // reed binary data
            logfile.print(reed_stat_left[0]);
            logfile.print(",");
            logfile.print(reed_stat_left[1]);
            logfile.print(",");
            logfile.print(WheelLeftRot);
            logfile.print(",");
            
            // reed binary data
            logfile.print(reed_stat_right[0]);
            logfile.print(",");
            logfile.print(reed_stat_right[1]);
            logfile.print(",");
            logfile.print(WheelRightRot);
            logfile.print(",");
            
            // ----------- Sample Rate ------------//
            logfile.println(measured_cycle_rate);
            logfile.close();
        }
        logTime = millis();
        
    }
    
#endif
    
#ifdef ECHO
    /*
    if(IMU1_online && RTC.isrunning()) {
        Serial.print("yaw: ");
        Serial.print(yaw);
        Serial.print("\t pitch: ");
        Serial.print(pitch);
        Serial.print("\t roll: ");
        Serial.print(roll);
    }
    if (IMU2_online && RTC.isrunning() ) {
        Serial.print("\t yaw2: ");
        Serial.print(yaw2);
        Serial.print("\t pitch2: ");
        Serial.print(pitch2);
        Serial.print("\t roll2: ");
        Serial.print(roll2);
    }
    */
    
    Serial.print("reed 1: ");
    Serial.print(reed_stat_left[0]);
    
    Serial.print("\t reed 2: ");
    Serial.print(reed_stat_left[1]);
    
    Serial.print("\t reed 3: ");
    Serial.print(reed_stat_right[0]);
    
    Serial.print("\t reed 4: ");
    Serial.print(reed_stat_right[1]);
    
    Serial.print("\t Yaw Left: ");
    Serial.print(yaw2);
    
    Serial.print("\t Yaw Right: ");
    Serial.print(yaw);
    
    Serial.print("\t Left rot: ");
    Serial.print(WheelLeftRot);
    
    Serial.print("\t Right rot: ");
    Serial.print(WheelRightRot);
    
    Serial.print("\t Freq: ");
    Serial.println(measured_cycle_rate);
    
       /*
        DateTime now = RTC.now();
        
        Serial.print(now.year(), DEC);
        Serial.print('/');
        Serial.print(now.month(), DEC);
        Serial.print('/');
        Serial.print(now.day(), DEC);
        Serial.print(' ');
        Serial.print(now.hour(), DEC);
        Serial.print(':');
        Serial.print(now.minute(), DEC);
        Serial.print(':');
        Serial.print(now.second(), DEC);
        Serial.println();
        */
    
#endif //echo
    
    loop_time = millis() - begin_of_loop;     //milliseconds
    measured_cycle_rate = 1000*(1/loop_time);   //hz
    if (measured_cycle_rate > sampleFreq) {
        delay_time = ((float)(1/sampleFreq)*1000) - loop_time;
        delay(delay_time);
    }
    // remeasure
    loop_time = millis() - begin_of_loop;     //milliseconds
    measured_cycle_rate = 1000*(1/loop_time);   //hz
    
    //clear the flag if WDT occured
    if (wdt_int) {
        wdt_int = false;
        wdt_int_reset();
    }
    
    // ----------- Sleep Handeling -------------//
    // MPU ISR flag handeling
    if(IMU1_online) {
        // Motion sensing processing
        if (mpu_int_triggered) {
            motionTimer = millis();
        }
        
        //reset the trigger if it occured
        mpu_int_triggered = false;
        
        if ((millis() - motionTimer) >= motionTimePeriod) {
            sleepNow();
        }
        
    }
    
}   // END OF LOOP


// IMU int pin ISR
void IMU_int() {
    // timers will not work in an ISR
    // So just set a flag
    mpu_int_triggered = true;
    //logFreq = 30;
    //sampleFreq = 30;
}

ISR(WDT_vect) {
    wdt_int = true;
}

void wdt_int_reset() {
    /* Clear the reset flag. */
    MCUSR &= ~(1<<WDRF);
    
    /* In order to change WDE or the prescaler, we need to
     * set WDCE (This will allow updates for 4 clock cycles).
     */
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    WDTCSR = 0b01101001;    // bit 6 sets interrupt enabled. Bit 3 initializes reset on overflow:
    //bit 6 and 3 set enables interrupt first over flow and reset second overflow
    //bit 5, 2-0,  selects overflow period: Currently 8 Seconds
    //wdt_enable(WDTO_2S);        // 2 second overflow

}

void sleepNow() {        // here we put the arduino to sleep
    
#ifdef ECHO
    Serial.println("going to sleep");
    delay(1000);
#endif
    
    

    /* In the avr/sleep.h file, the call names of these sleep modes are to be found:
     *
     * The 5 different modes are:
     *     SLEEP_MODE_IDLE         -the least power savings
     *     SLEEP_MODE_ADC
     *     SLEEP_MODE_PWR_SAVE
     *     SLEEP_MODE_STANDBY
     *     SLEEP_MODE_PWR_DOWN     -the most power savings
     *
     * For now, we want as much power savings as possible, so we
     * choose the according
     * sleep mode: SLEEP_MODE_PWR_DOWN
     *
     */
    
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
    adcsra = ADCSRA;               //save the ADC Control and Status Register A
    ADCSRA = 0;                    //disable ADC
    sleep_enable();          // enables the sleep bit in the mcucr register
    
    // so sleep is possible. just a safety pin
    
    /* Now it is time to enable an interrupt. We do it here so an
     * accidentally pushed interrupt button doesn't interrupt
     * our running program. if you want to be able to run
     * interrupt code besides the sleep function, place it in
     * setup() for example.
     *
     * In the function call attachInterrupt(A, B, C)
     * A   can be either 0 or 1 for interrupts on pin 2 or 3.
     *
     * B   Name of a function you want to execute at interrupt for A.
     *
     * C   Trigger mode of the interrupt pin. can be:
     *             LOW        a low level triggers
     *             CHANGE     a change in level triggers
     *             RISING     a rising edge of a level triggers
     *             FALLING    a falling edge of a level triggers
     *
     * In all but the IDLE sleep modes only LOW can be used.
     */
    
    //attachInterrupt(0,wakeUpNow, CHANGE); // use interrupt 0 (pin 2) and run function
    // wakeUpNow when pin 2 gets LOW
    
    cli();
    mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);  //turn off the brown-out detector
    mcucr2 = mcucr1 & ~_BV(BODSE);
    MCUCR = mcucr1;
    MCUCR = mcucr2;
    sei();
    
    sleep_mode();            // here the device is actually put to sleep!!
    // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
    
    sleep_disable();         // first thing after waking from sleep:
    // disable sleep...
    ADCSRA = adcsra;               //restore ADCSRA
    
    //watchdog timer
    wdt_int_reset();
}


//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
    
    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;
        
        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        
        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }
    
    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);
    
    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

 
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}


