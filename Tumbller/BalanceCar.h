/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @Date: 2019-10-08 09:35:07
 * @LastEditTime: 2019-10-11 16:25:04
 * @LastEditors: Please set LastEditors
 */
#include "MsTimer2.h"
#include "KalmanFilter.h"
#include "I2Cdev.h"
//#include "MPU6050_6Axis_MotionApps20.h"

#include "MPU6050.h"
#include "Wire.h"
MPU6050 mpu;
KalmanFilter kalmanfilter;

//Setting PID parameters

double kp_balance = 55, kd_balance = 0.75;
double kp_speed = 10, ki_speed = 0.26;
double kp_turn = 2.5, kd_turn = 0.5;

//Setting MPU6050 calibration parameters
double angle_zero = 0;            //x axle angle calibration
double angular_velocity_zero = 0; //x axle angular velocity calibration

volatile unsigned long encoder_count_right_a = 0;
volatile unsigned long encoder_count_left_a = 0;
int16_t ax, ay, az, gx, gy, gz;
float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;

int encoder_left_pulse_num_speed = 0;
int encoder_right_pulse_num_speed = 0;
double speed_control_output = 0;
double rotation_control_output = 0;
double speed_filter = 0;
int speed_control_period_count = 0;
double car_speed_integeral = 0;
double speed_filter_old = 0;
int setting_car_speed = 0;
int setting_turn_speed = 0;
double pwm_left = 0;
double pwm_right = 0;
float kalmanfilter_angle;
// char balance_angle_min = -27;
// char balance_angle_max = 27;
char balance_angle_min = -22;
char balance_angle_max = 22;

void carStop()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(STBY_PIN, HIGH);
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
}

void carForward(unsigned char speed)
{
  digitalWrite(AIN1, 0);
  digitalWrite(BIN1, 0);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed);
}

void carBack(unsigned char speed)
{
  digitalWrite(AIN1, 1);
  digitalWrite(BIN1, 1);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed);
}

void balanceCar()
{
  sei();
  encoder_left_pulse_num_speed += pwm_left < 0 ? -encoder_count_left_a : encoder_count_left_a;
  encoder_right_pulse_num_speed += pwm_right < 0 ? -encoder_count_right_a : encoder_count_right_a;
  encoder_count_left_a = 0;
  encoder_count_right_a = 0;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//  kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
//  kalmanfilter_angle = kalmanfilter.angle;
//  static double angle; 
//  angle = kalmanfilter_angle;
  
  double accelYAngle = atan2(ay, az);
  double degAccelAngle = accelYAngle*57.29578;

  double scaledZGyro = ((gx*0.0000025)*57.29578);

  static double angle = degAccelAngle; 
  angle = (0.9934 * (angle + scaledZGyro)) + (0.0066 * degAccelAngle); // complimentary filter
  kalmanfilter_angle = angle;

  double pTerm, iTerm, dTerm, output, angleError;
  float targetAngle = -2.2;
  float Kp = 10, kpOutput = 1;
  float Ki = .9, kiOutput = .09;
  float Kd = 7;


  angleError = (angle - targetAngle); // subtract angle by offset
    
  // calculate the proportional component
  pTerm = angleError*Kp;
  // calculate the integral component (summation of past errors * i scalar)
  static double integral = 0;
  integral += angleError;
  integral =  constrain(integral, -100,100);
  iTerm = Ki * integral;
  // calculate the derivative component
  static double lastAngle = 0;
  dTerm = Kd * (angle-lastAngle);
  lastAngle = angle;

  
  output = (pTerm + iTerm - dTerm);

  // angle adjusting
  static double outputFilter = output;
  static double outputFilterOld, outputIntegral = 0;
  outputFilter = outputFilterOld*.7 + output*.03;
  outputFilterOld = outputFilter;
  outputIntegral += outputFilter;
  // car_speed_integeral += -setting_car_speed;
  outputIntegral = constrain(outputIntegral, -3000, 3000);
  
  double outputControlOutput = -kpOutput * outputFilter + kiOutput * outputIntegral;

  pwm_left  = output + outputControlOutput;
  pwm_right = output + outputControlOutput;

  Serial.print(" angle ");
  Serial.print(angle);//32768
//  Serial.print(" delta angle ");
//  Serial.print(scaledZGyro);

  Serial.println(" ");

  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);
  if (motion_mode != START && motion_mode != STOP && (kalmanfilter_angle < balance_angle_min || balance_angle_max < kalmanfilter_angle))
  {
    motion_mode = STOP;
    carStop();
  }

  if (motion_mode == STOP && key_flag != '4')
  {
    car_speed_integeral = 0;
    setting_car_speed = 0;
    pwm_left = 0;
    pwm_right = 0;
    carStop();
  }
  else if (motion_mode == STOP)
  {
    car_speed_integeral = 0;
    setting_car_speed = 0;
    pwm_left = 0;
    pwm_right = 0;
  }
  else
  {
    if (pwm_left < 0)
    {
      digitalWrite(AIN1, 1);
      analogWrite(PWMA_LEFT, -pwm_left);
    }
    else
    {
      digitalWrite(AIN1, 0);
      analogWrite(PWMA_LEFT, pwm_left);
    }
    if (pwm_right < 0)
    {
      digitalWrite(BIN1, 1);
      analogWrite(PWMB_RIGHT, -pwm_right);
    }
    else
    {
      digitalWrite(BIN1, 0);
      analogWrite(PWMB_RIGHT, pwm_right);
    }
  }
}

void encoderCountRightA()
{
  encoder_count_right_a++;
}

void encoderCountLeftA()
{
  encoder_count_left_a++;
}

void carInitialize()
{
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA_LEFT, OUTPUT);
  pinMode(PWMB_RIGHT, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  carStop();
  Wire.begin();
  mpu.initialize();
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), encoderCountLeftA, CHANGE);
  attachPinChangeInterrupt(ENCODER_RIGHT_A_PIN, encoderCountRightA, CHANGE);
  MsTimer2::set(5, balanceCar);
  MsTimer2::start();
}
