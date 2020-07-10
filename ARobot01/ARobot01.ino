#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <ODriveArduino.h>

double Pk1 = 3700;  
double Ik1 = 9800;
double Dk1 = 105;

double Pk2 = 3700;  
double Ik2 = 9800;
double Dk2 = 105;

float velGain = 0.0004;

double Setpoint1, Input1, Output1;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

double Setpoint2, Input2, Output2;    // PID variables
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup

RF24 radio(9, 10); // CE, CSN
const byte addresses[][6] = {"00003", "00004"};



// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

//ODrive Object
ODriveArduino odrive1(Serial1);
ODriveArduino odrive2(Serial2);

//**************remote control****************
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO 

    int16_t RLR;
    int16_t RFB;
    int16_t LLR;
    int16_t LFB;

};

RECEIVE_DATA_STRUCTURE mydata_remote;

int requested_state;   

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define LED_PIN 13 // (Arduino is 11, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


float pitch;
float roll;
long velocity;

float RFB;
float RLR;
float LLR;
long LLRaccum;

long drive1;
long drive2;

int switch1 = 1;

int switchFlag = 0;
int switchToggle = 0;
int switchToggleFlag = 0;

float pot1;
float pot2;

unsigned long currentMillis;

long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timers

int loopTime;

unsigned long previousSafetyMillis;

int IMUdataReady = 0;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high



// ****************** SETUP ******************************

void setup() {


    radio.begin();
    radio.openWritingPipe(addresses[0]); // 00002
    radio.openReadingPipe(1, addresses[1]); // 00001
    radio.setPALevel(RF24_PA_MIN);

    radio.startListening();
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200);

    //ODrive serial ports
    Serial1.begin(115200);
    Serial2.begin(115200);

    // initialize device
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(105);
    mpu.setYGyroOffset(-1);
    mpu.setZGyroOffset(11);
    mpu.setXAccelOffset(-1311);
    mpu.setYAccelOffset(-503);
    mpu.setZAccelOffset(1117);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(33, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    PID1.SetMode(AUTOMATIC);              
    PID1.SetOutputLimits(-50000, 50000);
    PID1.SetSampleTime(10);

    PID2.SetMode(AUTOMATIC);              
    PID2.SetOutputLimits(-50000, 50000);
    PID2.SetSampleTime(10);

    pinMode(A0, INPUT);
    pinMode(A1, INPUT);

    pinMode(2, INPUT_PULLUP);

    delay(1000);

}

// ********************* MAIN LOOP *******************************

void loop() {  
      
        currentMillis = millis();
        if (currentMillis - previousMillis >= 10) {  // start timed event
          
        previousMillis = currentMillis;

        switch1 = digitalRead(2);       // is the switch pressed?

        pot1 = analogRead(A0);          // read trim pots
        pot2 = analogRead(A1);

        pot1 = pot1 - 512;
        pot2 = pot2 - 512;

        pot1 = (float) (pot1) / 100;
        pot2 = (float) (pot2) / 100;        

        if (switch1 == 0 && switchFlag == 0) {             // init the Odrives          
          OdriveInit();
          switchFlag = 1;
        }   

        if (radio.available()) {
                    radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE));
                    previousSafetyMillis = currentMillis; 
        }  

       // check if remote has become disconnected

        if(currentMillis - previousSafetyMillis > 200) {         
            Serial.println("*no data* ");
            mydata_remote.RLR = 512;
            mydata_remote.RFB = 512;
            mydata_remote.LLR = 512;
            mydata_remote.LFB = 512;
        } 

        // Threshold remote data for slop in sticks

        RLR = mydata_remote.RLR - 512;      // get to +/- zero value
        RFB = mydata_remote.RFB - 512;
        LLR = mydata_remote.LLR - 512;
        

        if (RLR > 50) {
          RLR = RLR -50;
        }
        else if (RLR < -50) {
          RLR = RLR +50;
        }
        else {
          RLR = 0;
        }
        //*******
        if (RFB > 50) {
          RFB = RFB -50;
        }
        else if (RFB < -50) {
          RFB = RFB +50;
        }
        else {
          RFB = 0;
        }
        //*******
        if (LLR > 50) {
          LLR = LLR -50;
        }
        else if (LLR < -50) {
          LLR = LLR +50;
        }
        else {
          LLR = 0;
        }

        //scale stick data to degrees        

        RFB = (float) (RFB) / 100;
        RLR = (float) (RLR) / 100;
        LLR = (float) (LLR) * -50;

        // get IMU data into degrees

        if (IMUdataReady == 1) {
          readAngles();
        }

        pitch = (ypr[1] * 180/M_PI);  
        roll = (ypr[2] * 180/M_PI);

        // use the ODrive init switch for motor enable/disable once the init is done.
        // if statements and flags are for debounce

        if (switch1 == 0 && switchFlag == 1 && switchToggleFlag == 0) {
          if (switchToggle == 0) {
              switchToggle = 1;
          }
          else if (switchToggle == 1) {
              switchToggle = 0;
          }
          switchToggleFlag = 1;
        }

        else if (switch1 == 1 && switchFlag == 1) {
          switchToggleFlag = 0;
        }

        // do PID calcs

        Setpoint1 = pot1 + RFB;
        Setpoint2 = pot2 + RLR;
        Input1 = pitch;
        Input2 = roll;
        PID1.Compute();
        PID2.Compute();
        
        if (switchToggle == 0) {        // drive the motors
            odrive1.SetVelocity(0, (Output1)+LLR); 
            odrive1.SetVelocity(1, (Output1*-1)+LLR);
            odrive2.SetVelocity(0, (Output2)+LLR); 
            odrive2.SetVelocity(1, (Output2*-1)+LLR);
          
        }

        else if (switchToggle == 1) {     // stop the motors
            Serial.println("Motors halted");
            odrive1.SetVelocity(0, 0); 
            odrive1.SetVelocity(1, 0);
            odrive2.SetVelocity(0, 0); 
            odrive2.SetVelocity(1, 0);
        }


        // print control data, count and mode to terminal   
            
        Serial.print("Data: ");
        Serial.print(RFB);
        Serial.print("  ");
        Serial.print(RLR);
        Serial.print("  ");
        Serial.print(LLR);
        Serial.print("  ");
        Serial.print(pitch);
        Serial.print("  ");
        Serial.print(roll);
        Serial.print("  ");
        Serial.print(pot1);
        Serial.print("  ");
        Serial.print(pot2);  
        Serial.print("  ");
        Serial.println(Output1);
         
        

      
      }     // end of timed loop        
   
}           // end  of main loop


// filter function

float filter(float lengthOrig, float currentValue, int filter) {
  float lengthFiltered =  (lengthOrig + (currentValue * filter)) / (filter + 1);
  return lengthFiltered;  
}




