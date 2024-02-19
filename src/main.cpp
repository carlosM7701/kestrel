#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//#include <PulsePosition.h>
#include <Servo.h>

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
// id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

Servo wingL;
Servo wingR;
Servo tail;

//receiver ppm signal setup
// PulsePositionInput receiver_input(RISING);
// float receiver_value[] = {0, 0, 0, 0, 0, 0, 0, 0};
// int channel_number = 0;

// void read_receiver(){

//   channel_number = receiver_input.available();
//   if(channel_number > 0){
//     for(int i = 1; i <= channel_number; i++){
//       receiver_value[i-1] = receiver_input.read(i);
//     }
//   }
// }

void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup(void)
{
  Serial.begin(9600);
  //receiver_input.begin(10);
  wingL.attach(2);
  wingR.attach(3);
  tail.attach(4);

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
   
  delay(1000);

  bno.setExtCrystalUse(true);
  displaySensorDetails();
}

void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  // read_receiver();
  // Serial.print("Channel Available: ");
  // Serial.println(receiver_input.available());
  // Serial.print("Receiver Yaw: ");
  // Serial.println(receiver_value[0]);
  // Serial.print("Ch1: ");
  // Serial.println(pulseIn(2, HIGH));

  // Serial.print("Ch2: ");
  // Serial.println(pulseIn(3, HIGH));

  // Serial.print("Ch3: ");
  // Serial.println(pulseIn(4, HIGH));

  // Serial.print("Ch4: ");
  // Serial.println(pulseIn(5, HIGH));

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> gyro_raw = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  /* Display the floating point data */
  // Serial.print("X: ");
  // Serial.print(gyro_raw.x());
  // Serial.print(" Y: ");
  // Serial.print(gyro_raw.y());
  // Serial.print(" Z: ");
  // Serial.print(gyro_raw.z());
  // Serial.print("\t\t");

  wingL.write(90 + 0.25 * gyro_raw.z());
  wingR.write(90 + 0.25 * gyro_raw.z());

  //Serial.println();

  // /* The WebSerial 3D Model Viewer expects data as heading, pitch, roll */
  // Serial.print(F("Orientation: "));
  // Serial.print((float)event.orientation.x);
  // Serial.print(F(", "));
  // Serial.print((float)event.orientation.y);
  // Serial.print(F(", "));
  // Serial.print((float)event.orientation.z);
  // Serial.println(F(""));

  // // /* The WebSerial 3D Model Viewer also expects data as roll, pitch, heading */
  // imu::Quaternion quat = bno.getQuat();
  
  // // Serial.print(F("Quaternion: "));
  // Serial.print((float)quat.w(), 4);
  // Serial.print(F(", "));
  // Serial.print((float)quat.x(), 4);
  // Serial.print(F(", "));
  // Serial.print((float)quat.y(), 4);
  // Serial.print(F(", "));
  // Serial.print((float)quat.z(), 4);
  // Serial.println(F(""));

  // // /* Also send calibration data for each sensor. */
  // uint8_t sys, gyro, accel, mag = 0;
  // bno.getCalibration(&sys, &gyro, &accel, &mag);
  // Serial.print(F("Calibration: "));
  // Serial.print(sys, DEC);
  // Serial.print(F(", "));
  // Serial.print(gyro, DEC);
  // Serial.print(F(", "));
  // Serial.print(accel, DEC);
  // Serial.print(F(", "));
  // Serial.print(mag, DEC);
  // Serial.println(F(""));

  delay(10);
}