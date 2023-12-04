include "BNO055.h"





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



