// Stewart Platform v2 with Dynamixel XL-320

#include <Servo.h>
#include <Wire.h>
#include <stdint.h>

//MIN and MAX PWM pulse sizes, they can be found in servo documentation - PUT XL320 GOAL POSITION LIMITS
#define MAX 1023  // 2500
#define MIN 1     // 500

//Positions of servos mounted in opposite direction
#define INV1 1
#define INV2 3
#define INV3 5

//constants for computation of positions of connection points
#define pi  3.14159
#define deg30 pi/6

// ************************************************************************************

unsigned long time;


//Zero positions of servos, in this positions their arms are perfectly horizontal
static int zero[6] = {512, 512, 512, 512, 512, 512};

//In this array is stored requested position for platform - x,y,z,rot(x),rot(y),rot(z)
static float arr[6] = {0, 0, 0, radians(0), radians(0), radians(0)};

//Actual degree of rotation of all servo arms, they start at 0 - horizontal, used to reduce
//complexity of calculating new degree of rotation
static float theta_a[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//Array of current servo positions in us
static int servo_pos[6];

//rotation of servo arms in respect to axis x
const float beta[] = {4*pi/3, pi/3, 2*pi/3, -pi/3, 0*pi, pi},


// ******************************* UPDATED VALUES FOR PLATFORM GEOMETRY *****************************************
//maximum and minimum servo positions, 0 is horizontal position
servo_min = radians(-100), servo_max = radians(100),
// servo_min = radians(-150), servo_max = radians(150),         // for larger range - might affect other parts of the code

// servo_mult - multiplier used for conversion radians->servo pulse in us     [=400/(pi/4) for previous]
//         L1 - (inches) effective length of servo arm,             [= 0.79 for previous]
//         L2 - (inches) length of base and platform connecting arm       [= 2.33 for previous]
//     z_home - height of platform above base, 0 is height of servo arms (fully extended?)  [= 2 for previous]

servo_mult = 1023 / (300*pi/180), L1 = 0.5, L2 = 2.33, z_home = 2.5;  // EDITED FOR THE NEW PLATFORM (z_home increased from 2 to 2.5)

//         RD - distance from center of platform to attachment points (arm attachment point)  [= 3.188 for previous]
//         PD - distance from center of base to center of servo rotation points (servo axis)  [= 3.188 for previous]
//    theta_p - angle between two servo axis points                       [= 0.542505644 for previous]
//    theta_r - angle between platform attachment points                    [= 0.410790778 for previous]

const float RD = 3.188, PD = 3.0398,
theta_p = 0.540042611, theta_r = 0.410790778,

// ******************************* AIMEE: END UPDATE VALUES *************************************

//  p[][] = x y values for servo rotation points
// re[]{} = x y z values of platform attachment points positions
// equations used for p and re will affect postion of X axis, they can be changed to achieve
// specific X axis position
p[2][6] = {
  {
    - PD * cos(deg30 - (theta_p / 2)), 
    - PD * cos(deg30 + (theta_p / 2)),
      PD * cos(deg30 + (theta_p / 2)), 
      PD * cos(deg30 - (theta_p / 2)),
      PD * sin(theta_p / 2), 
    - PD * sin(theta_p / 2)
  },
  {
      PD * sin(deg30 - (theta_p / 2)), 
      PD * sin(deg30 + (theta_p / 2)),
      PD * sin(deg30 + (theta_p / 2)), 
      PD * sin(deg30 - (theta_p / 2)),
    - PD * cos(theta_p / 2), 
    - PD * cos(theta_p / 2)
  }
},
re[3][6] = {
  {
    - RD * cos(deg30 - (theta_p / 2)), 
    - RD * cos(deg30 + (theta_p / 2)),
      RD * cos(deg30 + (theta_p / 2)), 
      RD * cos(deg30 - (theta_p / 2)),
      RD * sin(theta_p / 2), 
    - RD * sin(theta_p / 2)
  }, {
      RD * sin(deg30 - (theta_p / 2)), 
      RD * sin(deg30 + (theta_p / 2)),
      RD * sin(deg30 + (theta_p / 2)), 
      RD * sin(deg30 - (theta_p / 2)),
    - RD * cos(theta_p / 2), 
    - RD * cos(theta_p / 2)
  }, {
    0, 0, 0, 0, 0, 0
  }
};

//arrays used for servo rotation calculation
//H[]-center position of platform can be moved with respect to base, this is
//translation vector representing this move
static float M[3][3], rxp[3][6], T[3], H[3] = {0, 0, z_home};

// *********************INITIAL SET UP - ZERO POSITION ********************************

Dynamixel Dxl(1);       // Create dxl object

void setup() {

  pinMode(14, OUTPUT);			// LED pin - if required for debugging
  
  //begin of serial and dynamixel serial communication
  Serial2.begin(57600);
  Dxl.begin(3);

//  Dxl.writeWord(1, 24, 1); // need to run only once to enable torque
//  Dxl.writeWord(3, 24, 1);
//  Dxl.writeWord(4, 24, 1);
//  Dxl.writeWord(5, 24, 1);
//  
  Dxl.goalTorque(1, 1023); // set torque limit (0-1023) start small and increase to up to 1023 (or around 1020)
  Dxl.goalTorque(3, 1023);
  Dxl.goalTorque(4, 1023);
  Dxl.goalTorque(5, 1023);



  // Initialize servos to jointMode
  Dxl.jointMode(1);
  Dxl.jointMode(2);
  Dxl.jointMode(3);
  Dxl.jointMode(4);
  Dxl.jointMode(5);
  Dxl.jointMode(6);

  //putting into base position
  float arr_initial[6] = {0,0,0,radians(0),radians(0),radians(0)};
  setPos(arr_initial);
}

// *************** setPos() FUNCTION & SUPPORTING FUNCTIONS *******************************

//function calculating translation vector - desired move vector + home translation vector
void getT(float pe[])
{
  T[0] = pe[0] + H[0];
  T[1] = pe[1] + H[1];
  T[2] = pe[2] + H[2];
}

//function calculating rotation matrix
void getmatrix(float pe[])
{
  float psi = pe[5];
  float theta = pe[4];
  float phi = pe[3];
  M[0][0] = cos(psi) * cos(theta);
  M[1][0] = -sin(psi) * cos(phi) + cos(psi) * sin(theta) * sin(phi);
  M[2][0] = sin(psi) * sin(phi) + cos(psi) * cos(phi) * sin(theta);

  M[0][1] = sin(psi) * cos(theta);
  M[1][1] = cos(psi) * cos(phi) + sin(psi) * sin(theta) * sin(phi);
  M[2][1] = cos(theta) * sin(phi);

  M[0][2] = -sin(theta);
  M[1][2] = -cos(psi) * sin(phi) + sin(psi) * sin(theta) * cos(phi);
  M[2][2] = cos(theta) * cos(phi);
}

//calculates wanted position of platform attachment poins using calculated rotation matrix
//and translation vector
void getrxp(float pe[])
{
  for (int i = 0; i < 6; i++) {
    rxp[0][i] = T[0] + M[0][0] * (re[0][i]) + M[0][1] * (re[1][i]) + M[0][2] * (re[2][i]);
    rxp[1][i] = T[1] + M[1][0] * (re[0][i]) + M[1][1] * (re[1][i]) + M[1][2] * (re[2][i]);
    rxp[2][i] = T[2] + M[2][0] * (re[0][i]) + M[2][1] * (re[1][i]) + M[2][2] * (re[2][i]);
  }
}

//function calculating needed servo rotation value
float getAlpha(int *i) {
  static int n;
  static float th = 0;
  static float q[3], dl[3], dl2;
  double min = servo_min;
  double max = servo_max;
  n = 0;
  th = theta_a[*i];
  while (n < 20) {
    //calculation of position of base attachment point (point on servo arm where leg is connected)
    q[0] = L1 * cos(th) * cos(beta[*i]) + p[0][*i];
    q[1] = L1 * cos(th) * sin(beta[*i]) + p[1][*i];
    q[2] = L1 * sin(th);
    //calculation of distance between according platform attachment point and base attachment point
    dl[0] = rxp[0][*i] - q[0];
    dl[1] = rxp[1][*i] - q[1];
    dl[2] = rxp[2][*i] - q[2];
    dl2 = sqrt(dl[0] * dl[0] + dl[1] * dl[1] + dl[2] * dl[2]);
    //if this distance is the same as leg length, value of theta_a is corrent, we return it
    if (abs(L2 - dl2) < 0.01) {
      return th;
    }
    //if not, we split the searched space in half, then try next value
    if (dl2 < L2) {
      max = th;
    } else {
      min = th;
    }
    n += 1;
    if (max == servo_min || min == servo_max) {
      return th;
    }
    th = min + (max - min) / 2;
  }
  return th;
}

// MAIN FUNCTION
  unsigned char setPos(float pe[]) {
  unsigned char errorcount;
  errorcount = 0;
  for (int i = 0; i < 6; i++)
  {
    getT(pe);
    getmatrix(pe);
    getrxp(pe);
    theta_a[i] = getAlpha(&i);
    //SerialUSB.println(theta_a[i]); // checking for any unnatural theta_a[i] values
    if (i == INV1 || i == INV2 || i == INV3) {   // if servo is the odd servo, calculate the servo position to go in the negative direction
      servo_pos[i] = constrain(zero[i] - (theta_a[i]) * servo_mult, MIN, MAX);
    }
    else {                                       // if servo is the even servo, calculate the servo position to go in the positive direction
      servo_pos[i] = constrain(zero[i] + (theta_a[i]) * servo_mult, MIN, MAX);
    }
  }

  for (int i = 0; i < 6; i++)
  {
    if (theta_a[i] == servo_min || theta_a[i] == servo_max || servo_pos[i] == MIN || servo_pos[i] == MAX) {
      errorcount++;

     SerialUSB.println(theta_a[i]); SerialUSB.println(servo_min); SerialUSB.println(servo_max);
     SerialUSB.println("");
     SerialUSB.println(servo_pos[i]); SerialUSB.println(MIN); SerialUSB.println(MAX);
     SerialUSB.println("");
    }

    Dxl.goalPosition(i+1, servo_pos[i]);
  }
  return errorcount;

}

// ********************** MAIN CONTROL LOOP ****************************************************
// obtain requested action from serial connection, then execute it

long previousMillis = 0; // store last time value was updated
float thresh = 5;       // interval at which to update value
int controlMode = 1; // 0 home, 1 position, 2 mixWave, 3 sinWave, 4 sinBreath
float waveArray[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

void loop()
{

  long currentMillis = millis();

  if(currentMillis - previousMillis > thresh){
  
    previousMillis = currentMillis; // save the last time value was updated 

// NOTES: 
//      - inputs: Time, ampX, freqX, ampY, freqY, ampZ, freqZ, ampX_rot, freqX_rot, ampY_rot, freqY_rot, ampZ_rot, freqZ_rot
//      - ONLY use decimals, NO fractions!
//      - ampX,Y,Z < 0.25 inch for a total of 0.5 inch movement
//      - ampX,Y,Z_rot < 15 deg for a total of 30 deg movement

// controlMode = 3;       // No need when using Python API

serialEvent();

switch (controlMode){
      case 0: // home
        arr[0] = 0; arr[1] = 0; arr[2] = 0;
        arr[3] = radians(0); arr[4] = radians(0); arr[5] = radians(0);
        break;
      case 1: // position
        updateArr();
        break;
      case 2: // mixWave
        // variable used is waveArray
        arr[6] = mixWave(previousMillis/1000.0, waveArray[0], waveArray[1], waveArray[2], waveArray[3], waveArray[4], waveArray[5], waveArray[6], waveArray[7], waveArray[8], waveArray[9], waveArray[10], waveArray[11]);        
        break;
      case 3: // sinWave
        arr[6] = sinWave(previousMillis/1000.0, waveArray[0], waveArray[1], waveArray[2], waveArray[3], waveArray[4], waveArray[5], waveArray[6], waveArray[7], waveArray[8], waveArray[9], waveArray[10], waveArray[11]);
        break;
      case 4: // sinBreath
        arr[6] = sinBreathe(previousMillis/1000.0, waveArray[0], waveArray[1], waveArray[2], waveArray[3], waveArray[4], waveArray[5], waveArray[6], waveArray[7], waveArray[8], waveArray[9], waveArray[10], waveArray[11]);
        break;
   }

    setPos(arr);

  }

}

// ********************** FUNCTIONS TO CHOOSE DESIRED PLATFORM MOVEMENT ****************************************************

// generate sin wave after inputting amplitude and frequency for xyz, rot_xyz
float sinWave(float T, float ampX, float freqX, float ampY, float freqY, float ampZ, float freqZ, float ampX_rot, float freqX_rot, float ampY_rot, float freqY_rot, float ampZ_rot, float freqZ_rot){

  arr[0] = ampX*sin(2*pi*freqX*T);
  arr[1] = ampY*sin(2*pi*freqY*T);
  arr[2] = ampZ*sin(2*pi*freqZ*T);
  arr[3] = radians(ampX_rot*sin(2*pi*freqX_rot*T));
  arr[4] = radians(ampY_rot*sin(2*pi*freqY_rot*T));
  arr[5] = radians(ampZ_rot*sin(2*pi*freqZ_rot*T));

  return arr[6];
  
}

// generate breathing wave after inputting amplitude and frequency for xyz, rot_xyz
float sinBreathe(float T, float ampX, float freqX, float ampY, float freqY, float ampZ, float freqZ, float ampX_rot, float freqX_rot, float ampY_rot, float freqY_rot, float ampZ_rot, float freqZ_rot){

  arr[0] = (exp(sin(freqX*T)) - 1/exp(1))*((2*ampX)/(exp(1)-(1/exp(1))));
  arr[1] = (exp(sin(freqY*T)) - 1/exp(1))*((2*ampY)/(exp(1)-(1/exp(1))));
  arr[2] = (exp(sin(freqZ*T)) - 1/exp(1))*((2*ampZ)/(exp(1)-(1/exp(1))));
  arr[3] = radians(((exp(sin(freqX_rot*T)) - 1/exp(1))*((2*ampX_rot)/(exp(1)-(1/exp(1)))))-ampX_rot);
  arr[4] = radians(((exp(sin(freqY_rot*T)) - 1/exp(1))*((2*ampY_rot)/(exp(1)-(1/exp(1)))))-ampY_rot);
  arr[5] = radians(((exp(sin(freqZ_rot*T)) - 1/exp(1))*((2*ampZ_rot)/(exp(1)-(1/exp(1)))))-ampZ_rot);

  return arr[6];
  
}

// xyz == breathing, xyz_rot == sine wave
float mixWave(float T, float ampX, float freqX, float ampY, float freqY, float ampZ, float freqZ, float ampX_rot, float freqX_rot, float ampY_rot, float freqY_rot, float ampZ_rot, float freqZ_rot){

  arr[0] = (exp(sin(freqX*T)) - 1/exp(1))*((2*ampX)/(exp(1)-(1/exp(1))));
  arr[1] = (exp(sin(freqY*T)) - 1/exp(1))*((2*ampY)/(exp(1)-(1/exp(1))));
  arr[2] = (exp(sin(freqZ*T)) - 1/exp(1))*((2*ampZ)/(exp(1)-(1/exp(1))));
  arr[3] = radians(ampX_rot*sin(2*pi*freqX_rot*T));
  arr[4] = radians(ampY_rot*sin(2*pi*freqY_rot*T));
  arr[5] = radians(ampZ_rot*sin(2*pi*freqZ_rot*T));

  return arr[6];
  
}

// changes all values to a particular set values
float movValsToArray(float x=0, float y=0, float z=0, float rot_x=0, float rot_y=0, float rot_z=0) {

  arr[0] = x;
  arr[1] = y;
  arr[2] = z;
  arr[3] = radians(rot_x);
  arr[4] = radians(rot_y);
  arr[5] = radians(rot_z);

  return arr[6];
}


// ********************** VATSAL: FUNCTIONS TO ADD PY API CAPABILITY ****************************************************

double deltas[6] = {0, 0, 0, radians(0), radians(0), radians(0)};

void updateArr() {
  arr[6] = movValsToArray(deltas[0], deltas[1], deltas[2], deltas[3], deltas[4], deltas[5]);
}

int bytesToRead = 0; // bytes expecting to read
int bufferCount = 0; // knows when buffer is full

char currCommand; // what command is sent to the robot (the latest command)
char commBuffer[24] = {}; // thing that holds all of the bytes - fill up until full then send all info out
boolean reading = false;

// communication starts
void serialEvent() { // serial interrupt routines (task assignment + buffer management)
  
    if (Serial2.available()) {
      currCommand = Serial2.read();
      if (currCommand == 'a') {                 // position commands (xyz, rpy)
        controlMode = 1;
        for (int i=0; i<12; i++){
          commBuffer[i] = Serial2.read();
        }
        decodePosition();
      }
      
      else if (currCommand == 'h'){
        // go home
        controlMode = 0;
      }

      else if (currCommand == 'b'){
        controlMode = 2;        
        reading = true;
        bytesToRead = 24;
        for (int i=0; i<24; i++){
          commBuffer[i] = Serial2.read();
        }
        decode24();
      }

      else if (currCommand == 'c'){
        controlMode = 3;
       for (int i=0; i<24; i++){
         commBuffer[i] = Serial2.read();
       }
       decode24();
      }

      else if (currCommand == 'd'){
        controlMode = 4;
        for (int i=0; i<24; i++){
          commBuffer[i] = Serial2.read();
        }
        decode24();
      }
    }
}

// Converting the string serial data back to float values
// NOTES:
// ASCII Conversion - int('0') is 48
// In python script, each float is converted to a positive 2-digit integer value, and each digit is sent as a string byte
// float --> str conversion for XYZ values: add 0.5 and multiply by 100 (range:-0.49 to 0.49, resolution: 0.12)
// float --> str conversion for XrYrZr values: add 15 (range:-15 to 74, resolution: 12.)
// float --> str conversion for frequencies: multiply by 100 (range: 0.00 to 0.99, resolution: 0.12)

void decodePosition(){
  float vals[6] = {};
  for (int i=0; i<6; i++){
    vals[i] = (int(commBuffer[2*i])-48) + 10*(int(commBuffer[2*i+1])-48);
    if (i<3){
      deltas[i] = (vals[i]-50)/100;
    }
    else if (i>=3 && i<6) {
      deltas[i] = vals[i]-15;
    }
  }
}


void decode24() {
  float vals[12] = {};
  for (int i=0; i<12; i++){
    vals[i] = (int(commBuffer[2*i])-48) + 10*(int(commBuffer[2*i+1])-48);
    if (i == 0 || i == 2 || i == 4){
      waveArray[i] = (vals[i]-50)/100;
    }
    else if (i == 1 || i == 3 || i ==5 || i == 7 || i == 9 || i == 11){
      waveArray[i] = vals[i]/100;
    }
    else if (i == 6 || i == 8 || i == 10){
      waveArray[i] = vals[i]-15;
    }
  }
}
