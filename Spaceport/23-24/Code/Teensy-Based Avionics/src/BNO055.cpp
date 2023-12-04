include "BNO055.h"


void setupBno()
{
  if (!bno.begin())
  {
    while (1);
  }
  bno.setExtCrystalUse(true);
}

void calibrate()
{
  uint8_t system, gyro, accel, mag, i = 0;
  while ((system != 3) || (gyro != 3) || (accel != 3) || (mag != 3))
  {
    bno.getCalibration(&system, &gyro, &accel, &mag);
    i = i + 1;
    if (i == 10)
    { 
      i = 0;
    }
    delay(10);
  }
}



virtual imu::Quaternion get_orientation() {
return getQuat();
}

virtual imu::Vector<3> get_acceleration() {
acceleraton_vec = getVector(VECTOR_ACCELEROMETER);
return acceleration_vec
}

virtual imu::Vector<3> get_orientation_euler() {
euler_vec = getVector(EULER);
return euler_vec;
}

virtual String getdataString() {
return String(accleration_vec.x) +”,” + String(accleration_vec.y) +”,” +String(accleration_vec.z) +”,” + String(euler_vec.x) +”,” + String(euler_vec.y) +”,” String(euler_vec.z);
} 

virtual String getcsvHeader() {
String s = “acceleration_X (m/s/s), acceleration_Y (m/s/s), acceleration_Z (m/s/s) **** euler (orientation x), euler (orientation y), euler (orientation z)”;
}



