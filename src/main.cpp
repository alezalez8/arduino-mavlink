#include <Arduino.h>
//#include "..\.pio\libdeps\megaatmega2560\MAVLink v2 C library\common\mavlink.h"
#include <MAVLink.h>
#include <OneWire.h>



#define CubeOrangeSerial2 Serial1


unsigned long previousMillisMAVLink = 0;      // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 5;                       // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_count = num_hbs;                  // count of heartbeats. if num_hbs_count == num_hbs then num_hbs_count = 0;
unsigned long currentTime = 0;

unsigned long timeReceive = 0;
unsigned long currentTimeReceive = 0;


boolean onoff = false;
int leds_status = 0;
#define LEDS 13




void Mav_Request_Data() // сообщает ПК, какой набор данных и с какой скоростью я хочу получать
{
  mavlink_message_t msg;               // для формирования сообщений MAVLink.
  uint8_t buf[MAVLINK_MAX_PACKET_LEN]; // Буфер для хранения данных, которые будут отправлены по UART.
 // const int maxMessages = 3;           // кол-во моих сообщений
                                       // const uint16_t MAVMessages[maxMessages] = { MAVLINK_MSG_ID_SYS_STATUS, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, MAVLINK_MSG_ID_ATTITUDE };
                                       // const uint32_t MAVIntervals[maxMessages] = {0x02, 0x05, 0x02}; // Интервалы в микросекундах
                                       // mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
                             // mavlink_msg_command_long_pack(2, 200, &msg,
                             //                                      1, 0,                // system_id, component_id
                             //                                      MAV_CMD_SET_MESSAGE_INTERVAL, // MAVLink команда для установки интервала сообщения
                             //                                      0,                    // confirmation
                             //                                      MAVMessages[i],       // Тип сообщения, для которого устанавливаем интервал
                             //                                      MAVIntervals[i],      // Интервал передачи сообщения в микросекундах
                             //                                      0, 0, 0, 0            // Параметры команды (не используются)
  //                                    );

  // STREAMS that can be requested
  /*
   * Definitions are in common.h: enum MAV_DATA_STREAM
   *
   * MAV_DATA_STREAM_ALL=0,  Enable all data streams
   * MAV_DATA_STREAM_RAW_SENSORS=1  Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
   * MAV_DATA_STREAM_EXTENDED_STATUS=2 Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
   * MAV_DATA_STREAM_RC_CHANNELS=3  Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
   * MAV_DATA_STREAM_RAW_CONTROLLER=4  Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
   * MAV_DATA_STREAM_POSITION=6  Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
   * MAV_DATA_STREAM_EXTRA1=10  Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA2=11  Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA3=12  Dependent on the autopilot
   * MAV_DATA_STREAM_ENUM_END=13
   *
   * Data in PixHawk available in:
   *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
   *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
   */

  const int maxStreams = 4;

  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_RAW_SENSORS, MAV_DATA_STREAM_EXTRA3, MAV_DATA_STREAM_POSITION}; // Массив, определяющий запрашиваемые потоки данных
  const uint16_t MAVRates[maxStreams] = {0x05, 0x05, 0x04, 0x05};

  /*
   * @brief Pack a request_data_stream message
   * @param system_id ID of this system
   * @param component_id ID of this component (e.g. 200 for IMU)
   * @param msg The MAVLink message to compress the data into
   *
   * @param target_system  The target requested to send the message stream.
   * @param target_component  The target requested to send the message stream.
   * @param req_stream_id  The ID of the requested data stream
   * @param req_message_rate [Hz] The requested message rate
   * @param start_stop  1 to start sending, 0 to stop sending.
   * @return length of the message in bytes (excluding serial stream start sign)
                                                                        1                       2                         3
  static inline uint16_t mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                                              4                             5                 6                             7               8
                                 uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop)

    */

  for (int i = 0; i < maxStreams; i++)
  {                                                                                          // 1,  2,   3,  4, 5,       6,          7,        8
    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1); // 2 - это MAV_DATA_STREAM_EXTENDED_STATUS=2
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }
}



void comm_receive() {

  mavlink_message_t msg;    // используется для хранения разобранного MAVLink-сообщения.
  mavlink_status_t status;  // используется для отслеживания статуса процесса разбора сообщения.


  while (CubeOrangeSerial2.available() > 0) {
    uint8_t c = CubeOrangeSerial2.read();

    // Try to get a new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {  //  Попытка разбора полученного байта в сообщение MAVLink. Если сообщение успешно разобрано, функция вернет true.

      if (msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
        unsigned long timeNow = millis() - currentTime;
        currentTime = millis();
        mavlink_gps_raw_int_t gps_data;
        mavlink_msg_gps_raw_int_decode(&msg, &gps_data);
        Serial.println("              ++++++++++++++++++++++++++++++ GPS +++++++++++++++++++++++++++++++++");
        Serial.print("              Longtitude =  ");
        Serial.print(gps_data.lon);
        // Serial.print(mavlink_msg_gps_raw_int_get_lat(&msg));
        Serial.print("; Latitude =  ");
        Serial.print(gps_data.lat);
        Serial.print("; amount of satts = ");
        Serial.println(gps_data.satellites_visible);
        Serial.print("                                        time =  ");
        Serial.println(timeNow);
      }

      if (msg.msgid == MAVLINK_MSG_ID_SYS_STATUS) {
        mavlink_sys_status_t sys_status;
        mavlink_msg_sys_status_decode(&msg, &sys_status);
        float batV = sys_status.voltage_battery;
        Serial.println();
        Serial.println("              ----------------- STATUS FLIGHT CONTROLLER ------------------");
        Serial.print("                                     Battery :  ");
        Serial.print(batV / 1000);
        Serial.println(" V ");
        Serial.println();
      }

      if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        mavlink_global_position_int_t azimut;
        mavlink_msg_global_position_int_decode(&msg, &azimut);
        Serial.print(" Azimut=");
        Serial.print(azimut.hdg / 100);
        Serial.print("; alt=");
        Serial.print(azimut.alt);
        Serial.print("; relative alt=");
        Serial.println(azimut.relative_alt);
      }

      if (msg.msgid == MAVLINK_MSG_ID_SYS_STATUS) {
        mavlink_sys_status_t sys_status;
        mavlink_msg_sys_status_decode(&msg, &sys_status);

        Serial.print("                                        CPU load =  ");
        Serial.print(sys_status.load / 10);
        Serial.println("% ");
      }

      if (msg.msgid == MAVLINK_MSG_ID_DISTANCE_SENSOR) {
        Serial.println(" ********************************************************************* ");
        //mavlink_distance_sensor_t rangefnd;
        uint16_t distanceCurr = mavlink_msg_distance_sensor_get_current_distance(&msg);
        Serial.println(" ********************************************************************* ");
        Serial.print(" Range finder = ");
        Serial.println(distanceCurr);
      }

    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println("MAVLink starting.");
  pinMode(LEDS, OUTPUT);
}

void loop() {  
  // MAVLink
  /* The default UART header for your MCU */
  //int sysid = 1;                  ///< ID 20 for this airplane. 1 PX, 255 ground station
  //int compid = 205;               ///< The component sending the message, was 158
  int type = MAV_TYPE_HEXAROTOR;  ///< This system is an airplane / fixed wing
  //uint8_t system_type = MAV_TYPE_HEXAROTOR;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
  uint8_t system_mode = MAV_MODE_PREFLIGHT;  ///< Booting up
  uint32_t custom_mode = 0;                  ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY;  ///< System ready for flight
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];  //

  // Pack the message
  //mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  mavlink_msg_heartbeat_pack(1, 0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
    previousMillisMAVLink = currentMillisMAVLink;

    CubeOrangeSerial2.write(buf, len);
    num_hbs_count++;
    if (num_hbs_count >= num_hbs) {
      Mav_Request_Data();
      onoff = !onoff;
      digitalWrite(LEDS, onoff);
      num_hbs_count = 0;
    }
  }
  comm_receive();
  
}


