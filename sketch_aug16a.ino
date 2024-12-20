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
long baudrate = 500000;

void setup() {
  Serial.begin(baudrate); // Monitor serial
  Serial3.begin(57600, SERIAL_8N1); // Configura Serial2 con los pines GPIO 16 (RX) y GPIO 17 (TX)
  Serial2.begin(baudrate, SERIAL_8N1); // Configura Serial1 para la comunicación con la Pixhawk

  hedge.begin(&Serial3);
  t1 = millis();
}

void loop() {
  hedge.read();
  if (millis() - t1 > period) {
    if (hedge.getPositionFromMarvelmindHedge(true, &positionVP)) {
      sendToPixhawk();

      Serial.print("Position Hedge: ");
      Serial.print(" X=");
      Serial.print(positionVP.x, res);
      Serial.print(", Y=");
      Serial.print(positionVP.y, res);
      Serial.print(", Z=");
      Serial.print(positionVP.z, res);
      Serial.print(", Angle=");
      Serial.print(positionVP.angle, res);
      Serial.println();
    }
    // Enviar datos a la Pixhawk
    t1 = millis();
  }
}

void sendToPixhawk() {
  mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

  float pos_x = positionVP.x;
  float pos_y = positionVP.y;
  float pos_z = positionVP.z;
  float or_angle = positionVP.angle*0.0174533;
  float covariance[21] = {0}; // Inicializar el arreglo de covarianza a ceros
  uint8_t reset_counter = 0;  // Inicializar el contador de reinicio

  mavlink_msg_vision_position_estimate_pack(1, 2, &msg, 1, pos_x, pos_y, pos_z, 3, 4, or_angle, covariance, 1);
  uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
  Serial2.write(buffer, len);
  Serial.write(buffer, len);
  Serial.println();
}