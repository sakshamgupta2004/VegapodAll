//LIDAR PID Control BLDC Motor
#include <Servo.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();


Servo motor;
float elapsedTime, time, timePrev;
float PID, pwm, error, previous_error;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
/////////////////PID CONSTANTS/////////////////
double kp = 3.55; //3.55
double ki = 0.005; //0.003
double kd = 2.05; //2.05
double throttle = 1300; //initial value of throttle to the motor
float desired_distance = 15; //This is the distance to maintain in mm.
void setup() {
  Serial.begin(9600);
  motor.attach(9); ///ESC on pin 9
  time = millis();
  motor.writeMicroseconds(1000);
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  Serial.println("Waiting ro boot");
  delay(7000);
  Serial.println("Start");
}

void loop() {
  timePrev = time;
  time = millis();
  elapsedTime = (time - timePrev) / 1000;


  ///////READ DISTANCE HERE///////////

  VL53L0X_RangingMeasurementData_t measure;

  int curr_distance = 0;
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    curr_distance = measure.RangeMilliMeter;
  } else {
    Serial.println(" out of range "); //////error
  }

    ////////////////////////////////////


    
  if (curr_distance != 0) {
    error = curr_distance - desired_distance;
    pid_p = kp * error;
    if (-2 < error < 2)
    {
      pid_i = pid_i + (ki * error);
    }
    pid_d = kd * ((error - previous_error) / elapsedTime);
    PID = pid_p + pid_i + pid_d;
    pwm = throttle - PID;
    motor.writeMicroseconds(pwm);
  }
  else {
    motor.writeMicroseconds(1000);
  }
}
