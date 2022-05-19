#include <MPU9250.h>

#define MAG_CAL_X (30.32)
#define MAG_CAL_Y (-21.52)
#define MAG_CAL_Z (-5.76)
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;
float yaw;
float pitch; 
float roll;
float reference_pos = 90.0;
int Kp = 3, Kd = 1;

// Pin assignments for motors
const int pwm = 3; //initializing pin 2 as pwm
const int in_1 = 8 ;
const int in_2 = 9 ;

void set_motor_pins() {
   pinMode(pwm,OUTPUT) ; //we have to set PWM pin as output
   pinMode(in_1,OUTPUT) ; //Logic pins are also set as output
   pinMode(in_2,OUTPUT) ;
}

void setup() {
    
  // Setup motor pins
  set_motor_pins();
  
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
}

void update_rollpitchyaw() {
  
  // read the sensor
  IMU.readSensor();
  // accelerometer readings
  float accX = IMU.getAccelX_mss();
  float accY = IMU.getAccelY_mss();
  float accZ = IMU.getAccelZ_mss();

  // calibrated magnetometer readings
  float magX = IMU.getMagX_uT() + MAG_CAL_X;
  float magY = IMU.getMagY_uT() + MAG_CAL_Y;
  float magZ = IMU.getMagZ_uT() + MAG_CAL_Z;

  // update global parameters
  yaw =  -180 * atan2(magY, magX)/M_PI; // pretty dubious formula
  pitch = 180 * atan (accX/sqrt(accY*accY + accZ*accZ))/M_PI;
  roll = 180 * atan (accY/sqrt(accX*accX + accZ*accZ))/M_PI;


  Serial.print(yaw);
  Serial.print("\n");
  
  //%-9.4f, pitch: %-9.4f, roll: %-9.4f", yaw, pitch, roll);
  
}

void motor_forward(int new_speed){
  // Clockwise motion!
   digitalWrite(in_1,HIGH) ;
   digitalWrite(in_2,LOW) ;
   analogWrite(pwm,new_speed);
   delay(100);
}

void motor_backward(int new_speed){
  // Anti-Clockwise motion!
   digitalWrite(in_1,LOW) ;
   digitalWrite(in_2,HIGH) ;
   analogWrite(pwm,new_speed);
   delay(100);
}

void motor_stop(){
  digitalWrite(in_1,HIGH) ;
  digitalWrite(in_2,HIGH) ;
  delay(100);
}

void set_motor_speed(int new_speed) {

  int MOTOR_SPEED_MAX = 255;
  int MOTOR_SPEED_MIN = 70;

  // max input
  if (new_speed > MOTOR_SPEED_MAX) {
    new_speed = MOTOR_SPEED_MAX;
  }
  else if (new_speed < -MOTOR_SPEED_MAX) {
    new_speed = -MOTOR_SPEED_MAX;
  }

  if (new_speed > 10 && new_speed < MOTOR_SPEED_MIN) {
    new_speed = MOTOR_SPEED_MIN;
  }
  else if (new_speed < -10 && new_speed > -MOTOR_SPEED_MIN) {
    new_speed = -MOTOR_SPEED_MIN;
  }

  // update the speed
  //motor.setSpeed(abs(newSpeed));

  // set the direction
  if (new_speed > 10) {
    motor_forward(new_speed);
  }
  else if (new_speed < -10) {
    new_speed = -new_speed; // convert back to 0-255
    motor_backward(new_speed);
  }else{
    motor_stop();
  }
}

int PD(int yaw){
  
  float yaw_err;
  yaw_err = reference_pos - yaw; 

  int omega_z = IMU.getGyroZ_rads(); 
  int ctrl_out = (Kp*yaw_err) + (Kd*omega_z); 
  Serial.print("Yaw Error: ");
  Serial.println(yaw_err);

  Serial.print("Control Output: ");
  Serial.println(ctrl_out);

  return ctrl_out;
  
}

void loop() {

  int pwm_out; 
  
  update_rollpitchyaw();
  pwm_out = PD(yaw);

//  pwm_out = 11;
//  set_motor_speed(pwm_out);
//
//  pwm_out = 0;
//  set_motor_speed(pwm_out);  
//
//  pwm_out = -11;
//  set_motor_speed(pwm_out);  
  // set motor speeds
 // set_motor_speed(pwm_out);
//  
 }
  
    

  /*
  // read the sensor
  IMU.readSensor();
  
  
  // accelerometer readings
  float accX = IMU.getAccelX_mss();
  float accY = IMU.getAccelY_mss();
  float accZ = IMU.getAccelZ_mss();

  // calibrated magnetometer readings
  float magX = IMU.getMagX_uT() + MAG_CAL_X;
  float magY = IMU.getMagY_uT() + MAG_CAL_Y;
  float magZ = IMU.getMagZ_uT() + MAG_CAL_Z;

  // update global parameters
  yaw =  -180 * atan2(magY, magX)/M_PI; // pretty dubious formula
  pitch = 180 * atan (accX/sqrt(accY*accY + accZ*accZ))/M_PI;
  roll = 180 * atan (accY/sqrt(accX*accX + accZ*accZ))/M_PI;


  Serial.print(yaw);
  Serial.print("\n");
  
  //%-9.4f, pitch: %-9.4f, roll: %-9.4f", yaw, pitch, roll);
 */
 /* 
  // display the data
  Serial.print(IMU.getAccelX_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroZ_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(),6);
  Serial.print("\t");
  Serial.println(IMU.getTemperature_C(),6);
  delay(100);*/
