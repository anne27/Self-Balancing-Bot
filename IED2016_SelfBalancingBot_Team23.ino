#define version_0.66

#define DEBUGGING
#define BLUETOOTH

#include <Wire.h>
#include <I2Cdev.h>
#include <digitalIOPerformance.h>
// improves performance of DigitalIOpins which are decalred as const int, i.e known at compile-time

#include <SoftwareSerial.h>
#include <PID_v1.h>
#include <MPU6050_6Axis_MotionApps20.h>
//enables MPU chip to do most of the processing on the module itself, instead of by the microcontroller


#include <helper_3dmath.h>

#define DIGITAL_NO_INTERRUPT_SAFETY  //for digitalIOPerformance.h
#define DIGITALIO_NO_MIX_ANALOGWRITE  //for digitalIOPerformance.h

const int rx=9;  //rx pin of arduino which is connected to tx of BT
const int tx=10;  

SoftwareSerial bluetooth(rx,tx);  //declaring a bluetooth object which uses pins rx and tx instead of actual arduino serial commmunication pins

#define RESTRICT_PITCH    //Restricts Pitch to ±90deg 

MPU6050 mpu;

//MPU Variables
bool dmpReady = false;  // holds status of DMP initialization
uint8_t mpuIntStatus;   // MPU interrupt status
uint8_t devStatus;      // status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint16_t packetSize;    // expected DMP packet size (default - 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO

//DMP and Math Library Stuff - Motion/Orientation Vectors
Quaternion q;
// A quaternion stores 4 scalar values, 3 for the axes and the fourth for the rotation around that axis, to avoid the problem of Gimbal lock:
// as the Euler angles ( yaw, pitch, roll) are calculated sequentially, if the second rotation in the sequence approaches 90 degrees, the first and
// third axes are locked i.e aligned with each other.
VectorFloat gravity;
float ypr[3];   //[yaw,pitch,roll]

volatile bool mpuInterrupt = false;   //whether MPU interrupt pin has gone high

void dmpDataReady()
{
  mpuInterrupt=true;
}

//Balance PID
#define BALANCE_KP 15
#define BALANCE_KI 90
#define BALANCE_KD 0.8
#define BALANCE_PID_MIN -255
#define BALANCE_PID_MAX 255

//Rotation PID
#define ROTATION_KP 50
#define ROTATION_KI 300
#define ROTATION_KD 4

#define M11 4  //digital control pins for M1
#define M12 12
#define M21 7  //digital control pins for M2
#define M22 8
#define M1PWM 6  //analog control pin or M1
#define M2PWM 11  //analog control pin or M2

float M1slack = 70;  //minimum PWM value at which M1 starts to move, compensated later in function 'compensate_slack'
float M2slack = 45;

double yaw,input,out,setpoint,originalSetpoint,Buffer[3];
double yinput,yout,ysetpoint,yoriginalSetpoint;
double balkp,balki,balkd,rotkp,rotki,rotkd;

String content="";

PID pid(&input,&out,&setpoint,BALANCE_KP,BALANCE_KI,BALANCE_KD,DIRECT);  

// Input : The variable we are trying to control
// Output : The variable which we are trying to adjust using the Pid algorithm
// Setpoint : The value we want to maintain
// Kp, Ki, Kd constants
// Direction determines which direction the output will move towards when faced with a given error.( DIRECT or REVERSE )

PID rot(&yinput,&yout,&ysetpoint,ROTATION_KP,ROTATION_KI,ROTATION_KD,DIRECT);  //for rotation PIDS, used in direction calculation

int M1speed, M2speed, speedval=1, dirval=1;

void setup() {

  #ifdef DEBUGGING
    Serial.begin(115200);
  #endif

  #ifdef BLUETOOTH
    bluetooth.begin(9600);
    bluetooth.setTimeout(10);   //waits for maximum time to wait for Bluetooth data to come in
  #endif

  init_mpu();
  init_motors();

  pid.SetMode(AUTOMATIC);  //sets whether the PID should be ON ( automatic) or off( Manual)
  pid.SetOutputLimits(-210,210); //vary the balance pids betweeen -210 and 210
  pid.SetSampleTime(10); //Determines how frequently the PID algorithm evaluates (in ms )
  rot.SetMode(AUTOMATIC);
  rot.SetOutputLimits(-20,20);  //similarly for Rotation PIDs, which are used for calculating direction
  rot.SetSampleTime(10);

  setpoint=0; 
  originalSetpoint=0;
  ysetpoint=0;
  yoriginalSetpoint=0;

  balkp=BALANCE_KP;
  balki=BALANCE_KI;
  balkd=BALANCE_KD;
  rotkp=ROTATION_KP;
  rotki=ROTATION_KI;
  rotkd=ROTATION_KD;

  pid.SetTunings(balkp,balki,balkd);  //changes the PID calculations according to new Kp, Ki, Kd constants. This is initialisation
  rot.SetTunings(rotkp,rotki,rotkd);
  
}

void loop() {

  readvalues();  //in the loop, first reads MPU readings
  newpid();     //calculates new PID values 

  #ifdef BLUETOOTH
    BT_Control();
  #endif

  #ifdef DEBUGGING
    printvalues();
  #endif
}

void init_mpu()    //DMP internal code
{
    Wire.begin();
    mpu.initialize();
    devStatus=mpu.dmpInitialize();


   //MPU calibration is required to remove zero error : where the sensor is totally level, but thinks that it it is at an angle
   // MPU Calibration code takes care of the offsets by taking the average of a few 100 readings
   //MPU offsets which are calculated by calibrating the MPU according to its orientation in the robot when the robot is vertical, i.e the sensor is horizontal
    mpu.setXGyroOffset(58);  
    mpu.setYGyroOffset(-31);
    mpu.setZGyroOffset(31);
    mpu.setXAccelOffset(-881);
    mpu.setYAccelOffset(1009);
    mpu.setZAccelOffset(1475);

    if(devStatus==0)
    {
      //Turning DMP On
        mpu.setDMPEnabled(true);
      //Enabling interrupt on Arduino
        attachInterrupt(0,dmpDataReady,RISING);
        mpuIntStatus= mpu.getIntStatus();
        dmpReady=true;

        //expected packetsize
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
}

void init_motors()  //initialisation of motors 
{
  pinMode(M11,OUTPUT);
  pinMode(M12,OUTPUT);
  pinMode(M21,OUTPUT);
  pinMode(M22,OUTPUT);
  analogWrite(M1PWM,0);  // move with 0 speed on setup
  analogWrite(M2PWM,0);
}

void readvalues()
{
     // DMP should be ready
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
      //Deliberate empty loop
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // current FIFO count
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02)
    {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
     } 
         yinput = ypr[0]* 180/M_PI;
         input = -ypr[1] * 180/M_PI;
}

void newpid()
{
  //Compute error
  pid.Compute();  //contains the pid algorithm. At a frequency  described by SetSampleTime it will calculate a new output. ( for Balance PIDs)
  rot.Compute();  //similarly for Rotation PIDs
   // Convert PID output to motor control
   
     M1speed = compensate_slack(yout,out,1);  // changes the speed according to the slack in the motor, the output of the Rotation PID ( yout) and the output of the Balance PID( out)
     M2speed = compensate_slack(yout,out,0);
     movemotors(M1speed, M2speed);            //change speed
}

double compensate_slack(double yOutput,double Output,bool one)  //bool one is 1 for motor 1, 0 for Motor 2 
{
   // Compensate for DC motor slack where small values don't result in movement
   //yOutput is for left,right control
  if(one)
  {
   if (Output >= 0) 
   Output = Output + M1slack - yOutput;   //  for M1, if the final PWM is >= 0, adds the slack PWM, and subtracts the calculated balance PID output
   if (Output < 0) 
   Output = Output - M1slack - yOutput;   //  if the final PWM is < than 0, subtracts the slack PWM, and subtracts the calculated balance PID output
  }
  else
  {
    if (Output >= 0)      //similarly for M2
   Output = Output + M2slack + yOutput;
   if (Output < 0) 
   Output = Output - M2slack + yOutput;
  }
   Output = constrain(Output, BALANCE_PID_MIN, BALANCE_PID_MAX); //constrains the value of Output to the inputs provided if even after calculations it is outside that range
  return Output;
}

void movemotors(int M1speed, int M2speed) 
{
  // Motor 1 control
  if (M1speed >= 0) 
  {
    digitalWrite(M11,LOW);  //motor moves in fwd direction 
    digitalWrite(M12,HIGH);
  }
  else 
  {
    digitalWrite(M11,HIGH);  //backward direction
    digitalWrite(M12,LOW);
  }
  
  analogWrite(M1PWM,abs(M1speed));  //writes PWM value calculated to the PWM pin of the first motor

  // Motor 2 control
  if (M2speed >= 0) 
  {
    digitalWrite(M21,LOW);
    digitalWrite(M22,HIGH);
  }
  else 
  {
    digitalWrite(M21,HIGH);
    digitalWrite(M22,LOW);
  }
  analogWrite(M2PWM, abs(M2speed));
}

void BT_Control()
{
  if(bluetooth.available())
  {
   content=bluetooth.readString();  //reads a string sent from the direction control / PID part of the app.
     if(content[0]=='F')    // changes direction of motion to forward 
     {
        setpoint = originalSetpoint - speedval; 
     }
     else if(content[0]=='B')  //backward movement
     {
        setpoint = originalSetpoint + speedval;
     }
     else if(content[0]=='L')  //left
     {
        ysetpoint = constrain((ysetpoint + yoriginalSetpoint - dirval),-180,180);
     }
     else if(content[0]=='R')  //right
     {
        ysetpoint = constrain(ysetpoint + yoriginalSetpoint + dirval,-180,180);
     }
     else if(content[0]=='S')  //change of speed
     {
        speedval = (content.substring(2)).toInt();  // converts the rest of the string to an integer
     }
     else if(content[0]=='D') // changes direction
     {
        dirval = content.substring(2).toInt();
     }
    else if(content[0]=='P')  //PID values
    {
        if(content[1]=='C')  //changing PID values
        {
          if(content[2]=='B')
          {
            change_pid(1);                //change balance pid values
          }
          else
          {
            change_pid(0);                //change rotation pid values
          }
        }
    }
  }
}

void change_pid(bool b)        //change pid values
{
  #ifdef DEBUGGING
    Serial.print("Changing PID\n");
  #endif
  
  while(!bluetooth.available());
  for(int i=0;i<3;i++)
  {
    Buffer[i]=bluetooth.parseFloat();    //stores the new P, I and D values by splitting the input: parseFloat reads a float until any other char is encountered
  }
  if(b)
  {
    balkp=Buffer[0];
    balki=Buffer[1];
    balkd=Buffer[2];
    pid.SetTunings(balkp,balki,balkd);  //recalculates the PID values with the new balance Kp, Ki, Kd, values 
  }
  else
  {
    rotkp=Buffer[0];
    rotki=Buffer[1];
    rotkd=Buffer[2];
    rot.SetTunings(rotkp,rotki,rotkd);
  }
}

void printvalues()  //for debugging purposes
{
  Serial.print(yinput);
  Serial.print("\t"); 
  Serial.print(yoriginalSetpoint);
  Serial.print("\t");
  Serial.print(ysetpoint);
  Serial.print("\t");
  Serial.print(yout);
  Serial.print("\t");  
  Serial.print(input);
  Serial.print("\t");
  Serial.print(originalSetpoint);
  Serial.print("\t");
  Serial.print(setpoint);
  Serial.print("\t");
  Serial.print(out);
  Serial.print("\t");
  Serial.print(M1speed);
  Serial.print("\t");
  Serial.print(M2speed);
  Serial.print("\t");
  Serial.print(speedval);
  Serial.print("\t");
  Serial.print(dirval);
  Serial.println("\t");
}
