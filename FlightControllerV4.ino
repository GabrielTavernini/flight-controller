#include <Wire.h>
#include <Servo.h>
#define MPU 0x68

float OFFSET_GX = 0;                                    // Offset of the gyroscope x-axis, calculated using the function getGyroOffset()  
float OFFSET_GY = 0;                                    // Offset of the gyroscope y-axis, calculated using the function getGyroOffset()
float OFFSET_GZ = 0;                                    // Offset of the gyroscope z-axis, calculated using the function getGyroOffset()

float GX = 0;                                            
float GY = 0;
float GZ = 0;
float TMP = 0;
float AX = 0;
float AY = 0;
float AZ = 0;
float OFFSET_AX_P = -0.03; 
float OFFSET_AY_P = 0.01;     
float OFFSET_AZ_P = +0.09;
float OFFSET_AX_N = -0.03;  
float OFFSET_AY_N = 0.01;      
float OFFSET_AZ_N = +0.13;
float A_PITCH = 0;
float A_ROLL = 0;
float delta_r = 0;
float delta_p = 0;
int   stable_r = 0;
int   stable_p = 0;
float ROLL = 0;
float PITCH = 0;
float YAW = 0;
bool  ud = false;

long  TIME = 0;

int   AIL = 0;
int   ELE = 0;
int   RUD = 0;
int   THR = 0;
int   FMO = 0;

boolean reading = false;
int arming = 0;
boolean armed;

//PID STUFF
float I_OUT_PITCH = 0;
float I_OUT_ROLL = 0;
float I_OUT_YAW = 0;

//SETTINGS
int max_motor_speed = 1500;
int max_angle = 45;   //deg 
int max_angular_speed = 10; // deg/s
int angle_step = 830/(2*max_angle);     // 830 = 1890-1060
int ang_speed_step = 830/(2*max_angular_speed);

float pitch_offset = 0.8;
float roll_offset = -0.05;

float P_PITCH = 1.5 ; //
float I_PITCH = 0 ; //
float D_PITCH = 0.6; //                 

float P_ROLL = 1.5; // 
float I_ROLL = 0;
float D_ROLL = 0.6;  //                

float P_YAW = 3;
float I_YAW = 0.01;
float D_YAW = 0;
                                             
float dr = PI/180;
float rd = 180/PI;

int A = 900;
int B = 900;
int C = 900;
int D = 900;

Servo a;
Servo b;
Servo c;
Servo d;

int co = 0;

void setup() 
{
  Wire.begin();
  Serial.begin(9600);

  pinMode(4,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(19,INPUT);  // THROTTLE
  pinMode(18,INPUT);  // AILERON
  pinMode(15,INPUT);  // ELEVATOR
  pinMode(14,INPUT);  // RUDDER
  pinMode(16,INPUT);  // FLIGHT MODE 
  
  digitalWrite(4,LOW);
  digitalWrite(7,LOW);
  analogWrite(8,168);

  delay(500);
  
  a.attach(9);
  b.attach(6);
  c.attach(5);
  d.attach(10);

  a.writeMicroseconds(A);
  b.writeMicroseconds(B);
  c.writeMicroseconds(C);
  d.writeMicroseconds(D);

  delay(5000);
  IMU_SETUP();  
}

void loop() 
{
  IMU_UPDATE();
  CHANNELS_READ();

  if(!armed)
  {
    A = 900;
    B = 900;
    C = 900;
    D = 900;
    if(reading)
    {
      if(THR<1080 && RUD < 1080)
      {
        arming = arming+1;
        if(arming == 100)
        {
          I_OUT_PITCH = 0;
          I_OUT_ROLL = 0;
          armed = true;
          arming = 0;
        }
      }else
      {
        arming = 0;
      }
    }
  }else
  {
    if(reading)
    {
      if(THR<1080 && RUD<1080)
      {
        arming = arming+1;
        if(arming == 100)
        {
          armed = false;
          arming = 0; 
        }
      }else
      {
        arming = 0;
      }
    }
    BALANCE();
  }
   
  a.writeMicroseconds(A);
  b.writeMicroseconds(B);
  c.writeMicroseconds(C);
  d.writeMicroseconds(D);
}


void getGyroOffset(int samples)
{
  float avx = 0;
  float avy = 0;
  float avz = 0;
  int16_t gx = 0;
  int16_t gy = 0;
  int16_t gz = 0;
  for(int i = 0;i<samples;i++)
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,6,true);
    gx = Wire.read()<<8|Wire.read();
    gy = Wire.read()<<8|Wire.read();
    gz = Wire.read()<<8|Wire.read();
    
    avx = avx + (float) gx;
    avy = avy + (float) gy;
    avz = avz + (float) gz;
  }     
  avx = avx/samples;
  avy = avy/samples;
  avz = avz/samples;
  OFFSET_GX = avx;
  OFFSET_GY = avy;
  OFFSET_GZ = avz;
}

void IMU_SETUP()
{
  delay(100);
  writeRegister(0x6b,0x00);    
  writeRegister(0x1b,0x08);     
  writeRegister(0x1c,0x08);
  delay(100);
  getGyroOffset(2000);
  writeRegister(0x1a, 6); //6
}

void readRegister(byte address,int b)
{
  Wire.beginTransmission(MPU);
  Wire.write(address);
  Wire.requestFrom(MPU,b);
  Serial.print("Read: ");
  Serial.print(Wire.read());
  Serial.print("(");
  Serial.print(Wire.endTransmission(false));
  Serial.print(")\n");
}

void writeRegister(byte address, byte value)
{
  Wire.beginTransmission(MPU);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(false);
}

float divide(float a, float b)
{
  float r = a/b;
  if(isnan(r))
  {
    return 0;
  }
  return r;
}

float sqrt2(float a)
{
  float r = sqrt(a);
  if(isnan(r))
  {

  }
  return r;
}

float asin2(float a)
{
  float r = asin(a);
  if(isnan(r))
  { 
    return asin(-1.0);
  }
  return r;
}

void IMU_UPDATE()
{
  long t = micros();
  float gx;
  float gy;
  float gz; 
  Wire.beginTransmission(MPU);
  Wire.write(0x3b);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14);
  AX = ((double) (Wire.read()<<8|Wire.read()))/8192;
  AY = ((double) (Wire.read()<<8|Wire.read()))/8192;
  AZ = ((double) (Wire.read()<<8|Wire.read()))/8192;
  TMP = ((double) (Wire.read()<<8|Wire.read()))/340.0 + 36.53;
  gx = (float) (Wire.read()<<8|Wire.read());
  gy = (float) (Wire.read()<<8|Wire.read());
  gz = (float) (Wire.read()<<8|Wire.read());
  gx = (gx-OFFSET_GX)/65.5;
  gy = (gy-OFFSET_GY)/65.5;
  gz = (gz-OFFSET_GZ)/65.5;
  
  // ACCELEROMETER
  if(AX>0)
  {
    AX = AX+OFFSET_AX_P;  
  }else
  {
    AX = AX+OFFSET_AX_N;
  }
  if(AZ>0)
  {
    AZ = AZ+OFFSET_AZ_P;
  }else
  {
    AZ = AZ+OFFSET_AZ_N;
  }
  if(AY>0)
  {
    AY = AY + OFFSET_AY_P;
  }else
  {
    AY = AY + OFFSET_AY_N;
  }
  float PA_ROLL = A_ROLL;
  float PA_PITCH = A_PITCH;
  double m = sqrt(AX*AX + AY*AY + AZ*AZ);
  AX = AX/m; 
  AY = AY/m; 
  AZ = AZ/m;
  double co1 = sqrt(1 - AX*AX);
  A_ROLL = AX;
  if(isnan(co1))
  {
    A_ROLL = 1;
  }
  double co2 = sqrt(1 - AY*AY);
  A_PITCH = AY;
  if(isnan(co2))
  {
    A_PITCH = 1;
  }

  //
  
  delta_r = delta_r + PA_ROLL-A_ROLL;
  delta_p = delta_p + PA_PITCH-A_PITCH;
  if(delta_r < 0.016 && delta_r > -0.016) 
  {
     stable_r = stable_r+1;
  }else
  {
    delta_r = 0;
    stable_r = 0;
  }
  if(delta_p < 0.01 && delta_p > -0.01)
  {
     stable_p = stable_p+1;
  }else
  {
    delta_p = 0;
    stable_p = 0;
  }
  if(stable_r >= 50) 
  {
    ROLL = (A_ROLL+PA_ROLL)/2; 
    stable_r = 0;
  }
  if(stable_p>= 50)
  {
    PITCH = (A_PITCH + PA_PITCH)/2; 
    stable_p = 0;
  }
  if(TIME == 0)
  {
    PITCH = A_PITCH;
    ROLL = A_ROLL;
    TIME = micros();
    return;
  } 
  
  // GYROSCOPE
  float dt = (micros() - TIME)*0.000001;
  TIME = micros();
  float t_x =  ((gx+GX)/2)*dt;
  float t_y = -((gy+GY)/2)*dt;  read
  float t_z =  ((gz+GZ)/2)*dt;
  float d;
  float d2;
  if(ud)
  {
    d = asin2(divide(PITCH,sqrt2(1-ROLL*ROLL))) - t_x*dr;
    if(d > 3.14159/2)
    {
        ud = false;
    }
    if(d < -3.14159/2)
    {
      ud = false;
    }
  }else
  {
     d = asin2(divide(PITCH,sqrt2(1-ROLL*ROLL))) + t_x*dr;
     if(d > 3.14159/2)
     {
        ud = true;
     }
     if(d < -3.14159/2)
     {
        ud = true;
     }
  }  
  PITCH = sin(d)*sqrt2(1-ROLL*ROLL);
  if(ud)
  {
    d2 = asin2(divide(ROLL, sqrt2(1-PITCH*PITCH))) - t_y*dr;
    if(d2 > 3.14159/2)
     {
        ud = false;
     }
     
     if(d2 < -3.14159/2)
     {
        ud = true;
     }
  }else
  {
     d2 = asin2(divide(ROLL,sqrt2(1-PITCH*PITCH))) + t_y*dr;
     if(d2 > 3.14159/2)
     {
        ud = true;
     }
     if(d2 < -3.14159/2)
     {
        ud = true;
     }
  } 
  ROLL = sin(d2)*sqrt2(1-PITCH*PITCH); 
  ROLL = ROLL*cos(t_z*dr) + sin(t_z*dr)*divide(PITCH, sqrt2(1-ROLL*ROLL))*sqrt2(1-ROLL*ROLL); 
  PITCH = PITCH*cos(t_z*dr) - sin(t_z*dr)*divide(ROLL, sqrt2(1-PITCH*PITCH))*sqrt2(1-PITCH*PITCH);

  GX = gx;
  GY = gy;
  GZ = gz;
}

void CHANNELS_READ()
{
  co++;
  if(co == 1)
  {
     THR = pulseIn(19,HIGH);
  }
  if(co == 3)
  {
     AIL = pulseIn(14,HIGH);
  }
  if(co == 5)
  {
     ELE = pulseIn(15,HIGH);
  }
  if(co == 7)
  {
     RUD = pulseIn(18,HIGH);
  }
  if(co == 9)
  {
     FMO = pulseIn(16,HIGH);
     co = 0; 
  }

  if(THR < 800)
  {
    reading = false;
  }else
  {
    reading = true;
  }
}



void BALANCE()
{  
  float pitch = asin(ROLL)*rd+roll_offset;
  float roll = asin(PITCH)*rd+pitch_offset;
  
  THR = THR/10;
  THR = THR*10;

  A = THR;
  B = THR;
  C = THR;
  D = THR;
  
  //Pitch
  float setpoint_pitch = -(ELE-1475)/angle_step;
  float P_OUT_PITCH = (pitch-setpoint_pitch)*P_PITCH; 
        I_OUT_PITCH = I_OUT_PITCH+ (pitch-setpoint_pitch)*I_PITCH;
  float D_OUT_PITCH = GY*D_PITCH;

  A = A-P_OUT_PITCH/2+D_OUT_PITCH/2-I_OUT_PITCH/2;
  B = B+P_OUT_PITCH/2-D_OUT_PITCH/2+I_OUT_PITCH/2;
  C = C-P_OUT_PITCH/2+D_OUT_PITCH/2-I_OUT_PITCH/2;
  D = D+P_OUT_PITCH/2-D_OUT_PITCH/2+I_OUT_PITCH/2;

  //Roll
  float setpoint_roll = (AIL-1475)/angle_step;
  float P_OUT_ROLL = (roll-setpoint_roll)*P_ROLL;
        I_OUT_ROLL = I_OUT_ROLL+(roll-setpoint_roll)*I_ROLL;
  float D_OUT_ROLL = GX*D_ROLL;

  A = A-P_OUT_ROLL/2-D_OUT_ROLL/2-I_OUT_ROLL/2;
  B = B-P_OUT_ROLL/2-D_OUT_ROLL/2-I_OUT_ROLL/2;
  C = C+P_OUT_ROLL/2+D_OUT_ROLL/2+I_OUT_ROLL/2;
  D = D+P_OUT_ROLL/2+D_OUT_ROLL/2+I_OUT_ROLL/2;

  //Yaw
  float setpoint_yaw = (RUD-1475)/ang_speed_step;
  float P_OUT_YAW = (GZ-setpoint_yaw)*P_YAW;
        I_OUT_YAW = I_OUT_YAW + (GZ-setpoint_yaw)*I_YAW;
  float D_OUT_YAW = 0;

  if(THR<1075)
  {
    P_OUT_YAW = 0;
    I_OUT_YAW = 0;
    D_OUT_YAW = 0;  
  }
  
  A = A+P_OUT_YAW+I_OUT_YAW-D_OUT_YAW;
  B = B-P_OUT_YAW-I_OUT_YAW+D_OUT_YAW;
  C = C-P_OUT_YAW-I_OUT_YAW+D_OUT_YAW;
  D = D+P_OUT_YAW+I_OUT_YAW-D_OUT_YAW; 

  if(A>max_motor_speed)
  {
    A = max_motor_speed;
  }
  if(B>max_motor_speed)
  {
    B = max_motor_speed;
  }

  if(C>max_motor_speed)
  {
    C = max_motor_speed;
  }
  if(D>max_motor_speed)
  {
    D = max_motor_speed;
  }
}



