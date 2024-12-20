/*void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}*/

#include <Marvelmind.h>
#include <MAVLink.h>

MarvelmindHedge hedge;
PositionValuePro positionVP;
RawIMUValue rIMUV;
FusionIMUValuePro fIMUVP;
char print_buffer[128];

const int res = 1; // Resolution to print values
float radiansToDegrees = 57.2957795f; // radians to degrees
float roll, pitch, yaw;

unsigned long t1;
const unsigned long period = 100;
long baudrate = 115200;

void setup() {
  Serial.begin(baudrate); // Monitor serial
  Serial2.begin(baudrate, SERIAL_8N1, 16, 17); // Configura Serial2 con los pines GPIO 16 (RX) y GPIO 17 (TX)
  Serial1.begin(baudrate, SERIAL_8N1, 18, 19); // Configura Serial1 para la comunicación con la Pixhawk

  hedge.begin(&Serial2);
  t1 = millis();
}

void loop() {
  hedge.read();
  if (millis() - t1 > period) {
    // Enviar datos a la Pixhawk
    sendToPixhawk();

    t1 = millis();
  }
}

void sendToPixhawk() {
  mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

  // Enviar datos de posición
  if (hedge.getPositionFromMarvelmindHedge(true, &positionVP)) {
    mavlink_msg_local_position_ned_pack(1, 200, &msg, millis(), positionVP.x / 1000.0, positionVP.y / 1000.0, positionVP.z / 1000.0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    Serial1.write(buffer, len);
  }

  // Enviar datos del IMU crudo
  if (hedge.getRawIMUFromMarvelmindHedge(true, &rIMUV)) {
    uint64_t time_usec = millis() * 1000; // Tiempo en microsegundos
    mavlink_msg_raw_imu_pack(1, 200, &msg, time_usec, 
                             rIMUV.acc_x, rIMUV.acc_y, rIMUV.acc_z, 
                             rIMUV.gyro_x, rIMUV.gyro_y, rIMUV.gyro_z, 
                             rIMUV.compass_x, rIMUV.compass_y, rIMUV.compass_z, 
                             0, 0); // Asumiendo valores predeterminados para temperatura y temperatura2
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    Serial1.write(buffer, len);
  }

  // Enviar datos de la IMU fusionada
  if (hedge.getFusionIMUFromMarvelmindHedge(true, &fIMUVP)) {
    roll = fIMUVP.roll * radiansToDegrees;
    pitch = fIMUVP.pitch * radiansToDegrees;
    yaw = fIMUVP.yaw * radiansToDegrees;

    mavlink_msg_attitude_pack(1, 200, &msg, millis(), roll, pitch, yaw, fIMUVP.vx, fIMUVP.vy, fIMUVP.vz);
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    Serial1.write(buffer, len);
  }
}

