// Arduino MAVLink Simulation


#include <mavlink.h>
#include <SoftwareSerial.h>

// Connect the GPS RX/TX to arduino pins 8 and 9 for nano else check the SoftwareSerial docs for correct pin
SoftwareSerial altSerial(2,3);

// Parameter setup
// These parameters may be entered manually, parsed from another protocol or simulated locally.
// If your input protocol does not provide values for all of these, parameters may be left at their default values.
// All parameters set here are passed onward through serial output in MAVLink format.

//Basic UAV Parameters
uint8_t system_id = 200; 
uint8_t component_id = 1;     
uint8_t vehicle_type = MAV_TYPE_QUADROTOR;      // UAV type. 0 = generic, 1 = fixed wing, 2 = quadcopter, 3 = helicopter
uint8_t firmware_type = 3;   // 3 = Autopilot type. Usually set to 0 for generic autopilot with all capabilities
uint8_t base_mode = 0;      // 0 is stabilizze
uint32_t custom_mode = 1;     // Usually set to 0          
uint8_t system_state = 3;     // 0 = unknown, 3 = standby, 4 = active
uint32_t upTime = 0;          // System uptime, usually set to 0 for cases where it doesn't matter


uint8_t mvl_packet_received = 0;
mavlink_message_t mvl_tx_message; //A special MAVLink message data structure. 
mavlink_message_t mvl_rx_message;
const uint8_t mvl_chan = MAVLINK_COMM_0;  //MAVLink channel 1 appears to be required at least for Blue Robotics QGC
mavlink_status_t mvl_rx_status;

String params[5] = { 
  "WPNAV_COOR_NS",
  "WPNAV_ACCEL",
  "WPNAV_RADIUS",
  "WPNAV_SPEED",
  "WPNAV_COOR_WE"
};
template< typename T, size_t N > size_t ArraySize (T (&) [N]){ return N; }
// _____________GPS CODES______________****************___________________________
const unsigned char UBX_HEADER[] = {0xB5, 0x62, 0x01, 0x07};
static int fpos = 0;
struct NAV_PVT {
  unsigned char cls =0x01;
  unsigned char id=0x07;
  unsigned short len;
  
  unsigned long iTOW;           //1.u4 4 GPS time of week of the navigation epoch (ms)
  
  unsigned short year;          //2.u2 6 Year (UTC) 
  unsigned char month;          //3.u1 7 Month, range 1..12 (UTC)
  unsigned char day;            //4.u1 8 Day of month, range 1..31 (UTC)
  unsigned char hour;           //5.u1 9 Hour of day, range 0..23 (UTC)
  unsigned char minute;         //6.u1 10 Minute of hour, range 0..59 (UTC)
  unsigned char second;         //7.u1 11 Seconds of minute, range 0..60 (UTC)
  
  unsigned char valid;          //8.x1 12 alidity Flags (see graphic below)
  unsigned long tAcc;           //9.u4 16 Time accuracy estimate (UTC) (ns)
  long nano;                    //10.i4 20 Fraction of second, range -1e9 .. 1e9 (UTC) (ns)
  unsigned char fixType;        //11.u1 21 GNSSfix Type, range 0..5
  unsigned char flags;          //12.x1 22 Fix Status Flags
  unsigned char flaggs2;        //13.x1 23 reserved
  unsigned char numSV;          //14.u1 24 Number of satellites used in Nav Solution
  
  long lon;                     //15.i4 28 Longitude (deg)
  long lat;                     //16.i4 32 Latitude (deg)
  long height;                  //17.i4 36 Height above Ellipsoid (mm)
  long hMSL;                    //18.i4 40 Height above mean sea level (mm)
  unsigned long hAcc;           //19.u4 44 Horizontal Accuracy Estimate (mm)
  unsigned long vAcc;           //20.u4 48 Vertical Accuracy Estimate (mm)
  
  long velN;                    //21.i4 52 NED north velocity (mm/s)
  long velE;                    //22.i4 56 NED east velocity (mm/s)
  long velD;                    //23.i4 60 NED down velocity (mm/s)
  long gSpeed;                  //24.i4 64 Ground Speed (2-D) (mm/s)
  long headMot;                 //25.i4 68 Heading of motion 2-D (deg)

  unsigned long sAcc;           //26.u4 72 Speed Accuracy Estimate
  unsigned long headingAcc;     //27.u4 76 Heading Accuracy Estimate
  unsigned short pDOP;          //28.u2 78 Position dilution of precision
  unsigned short flage3;        //29.x2 80
  unsigned short  reserved1;    //30.u1/4 84 *****************
  long headVeh;                 //31.i4 88
  short magDec;                 //32.i2 90
  unsigned short magAcc;        //33.u2 92
};

bool newData = false;
bool send_param_state = false;
unsigned long time_micro_one_hz = 0;
unsigned long time_micro_two_hz = 0;
unsigned long time_micro_three_hz = 0;
int sending_position = 1;
int send_param_index = 0;
int times_send_param = 0;
unsigned long loop_timer = 0;
int params_length = ArraySize(params);
int loop_index = 1;

NAV_PVT pvt;

void calcChecksum(unsigned char* CK) {
    memset(CK, 0, 2);
    for (int i = 0; i < (int)sizeof(NAV_PVT); i++) {
        CK[0] += ( (unsigned char*)(&pvt) )[i];
        CK[1] += CK[0];
    }
}

void processGPS() {
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_PVT);
  if ( altSerial.available() > 0 ) {
    byte c = altSerial.read(); 
    if ( fpos <= 3) {
      if ( c == UBX_HEADER[fpos] ){
          fpos++; 
          // if(fpos == 4) newData = true;
      }else {
          fpos = 0;
          newData = false;
      }
    }
    else {      
      if ( fpos>=4 && fpos < payloadSize ){ // starting from fpos 4 meaning the we pas the id byte so now we are recording from len
        ((unsigned char*)(&pvt))[fpos-2] = c; // @ at pos=4 we record pvt @index 2 is len at 3rd position
      }
      
      fpos++; // need to be here!

      if ( fpos == (payloadSize+4) ) { // the fpos has increment over the payload 4 times
          // here the fpos is 98 and our pvt is 94
          calcChecksum(checksum); // we miss the checksum ?
          if(pvt.fixType ==3) newData = true;
      }
  
      else if ( fpos == (payloadSize+5) ) {
          if ( c == checksum[0] ) { // if we miss why this work?  
            // Serial.println('__0');
            
          }else{
            fpos = 0;   
          }
      }
      else if ( fpos > (payloadSize+5) ) {
          fpos = 0;
      }
    }
  }
}


void setup() {
  // Enable serial output. Default: 57600 baud, can be set higher if needed
  Serial.begin(57600);
  altSerial.begin(38400);
  time_micro_one_hz = micros();
  time_micro_two_hz = micros();
}



//Main loop: Send generated MAVLink data via serial output
void loop() {
  
  loop_timer = micros();
  while((micros()-loop_timer < 20000)){
    if (Serial.available()>0){
      uint8_t rxbyte = Serial.read();
      mvl_packet_received = mavlink_parse_char(mvl_chan,rxbyte, &mvl_rx_message, &mvl_rx_status);
    }
    processGPS();
    if ((micros() - time_micro_one_hz) > 1000000 ){
      runGPS();
      time_micro_one_hz = micros();
    }
  }

  loop_timer = micros();
  while((micros()-loop_timer < 2000)){
    if(micros() - time_micro_three_hz > 3000000){
        sending_position = 0;
    }

    if ((micros() - time_micro_two_hz) > 1000000 ){
      time_micro_two_hz = micros();
      command_heartbeat(system_id, component_id, firmware_type, vehicle_type, base_mode, custom_mode, system_state);
    }

    if ((mvl_packet_received) && (255==mvl_rx_message.sysid)){ 
      mvl_packet_received = 0;
      if(mvl_rx_message.msgid == 21){ //#21 https://mavlink.io/en/messages/common.html#PARAM_REQUEST_LIST
        send_param_state = true;
      } 
      
      if(mvl_rx_message.msgid == 20 && !send_param_state){
        // mavlink_param_request_read_t msgDecode;
        // mavlink_msg_param_request_read_decode(&mvl_rx_message,&msgDecode);
        // String paramTosend = params[msgDecode.param_index];
        // if(msgDecode.param_index > -1) send_param(msgDecode.param_index,&paramTosend,true);
      }
      if(mvl_rx_message.msgid == 43){
        mavlink_message_t msg;
        mavlink_msg_mission_count_pack(system_id,component_id,&msg,225,1,0);
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); // set buf and return length
        Serial.write(buf,len);
      }
    }
  
    if(send_param_index < params_length && send_param_state){
      String param = params[send_param_index];
      send_param(send_param_index,&param,false);
      send_param_index++; 
      times_send_param++;
      if(send_param_index == params_length) {
        time_micro_three_hz = micros();
        send_param_state = false; 
        send_param_index =0;
      } 
    }
  }

  loop_index = loop_index == 1 ? loop_index = 2 : 1;
}



void runGPS (){
  command_gps(system_id, component_id, micros(), pvt.fixType, pvt.lat, pvt.lon, 1, 1,0, pvt.gSpeed, pvt.pDOP, pvt.numSV);
}


void send_param(int idx, String *param, bool single){
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_param_value_t mvl_param;

  const char *charBuf = param->c_str();
  for(int i =0; i < strlen(charBuf); i++ ){
    char c = charBuf[i];
    mvl_param.param_id[i] = c;
  }
  mvl_param.param_id[strlen(charBuf)] = '\0'; // terminated_indicator

  mvl_param.param_type = 9; //https://mavlink.io/en/messages/common.html#MAV_PARAM_TYPE // Putting the incorrect type will cause QGC to shutdown
  mvl_param.param_value = times_send_param*1.0f; //the parameter value as a float
  mvl_param.param_count = single ? 1 : 5;//idx == 4 ? 5 : idx+1; //We have just one parameter to send. 
  mvl_param.param_index = single ? idx : idx; //index greater than -1 will cause QG shutdown.
  mavlink_msg_param_value_encode(system_id,component_id,&msg,&mvl_param);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg); // set buf and return length
  Serial.write(buf, len);

}


/************************************************************
* @brief Sends a MAVLink heartbeat message, needed for the system to be recognized
* @param Basic UAV parameters, as defined above
* @return void
*************************************************************/

void command_heartbeat(uint8_t system_id, uint8_t component_id, uint8_t vehicle_type , uint8_t firmware_type, uint8_t base_mode, uint32_t custom_mode, uint8_t system_state) {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  //  mavlink_msg_heartbeat_pack(system_id,component_id, &msg, system_type, autopilot_type, base_mode, custom_mode, system_state);
  // base_mode 1 no touching, custom_mode 0 stabilize, 1 acro ...
  // mavlink_msg_heartbeat_pack(system_id,component_id, &msg, 3, 3, 1, 5, 3); ardupilot
  mavlink_msg_heartbeat_pack(system_id,component_id, &msg, 3, 0, 80, 0, 3); //generic has no custom_mode set to 0, base_mode manual_stabilize
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
 
  // Send the message
  Serial.write(buf, len);
}

/************************************************************
* @brief Sends current geographical location (GPS position), altitude and heading
* @param lat: latitude in degrees, lon: longitude in degrees, alt: altitude, heading: heading
* @return void
*************************************************************/

void command_gps(int8_t system_id, int8_t component_id, int32_t upTime, int8_t fixType, float lat, float lon, float alt, float gps_alt, int16_t heading, float groundspeed, float gps_hdop, int16_t gps_sats) {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_gps_raw_int_pack(system_id, component_id, &msg, upTime, fixType, lat, lon, alt/1000.0f, gps_hdop , 0.5, groundspeed, heading, gps_sats);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  //Send globalgps command
  
  
  // Send the message (.write sends as bytes)
  Serial.write(buf, len);
}
/************************************************************
* @brief Sends Integer representation of location, only for ijnternal use (do not call directly)
* @param lat: latitude, lon: longitude, alt: altitude, gps_alt: altitude above MSL, heading: heading
* @return void
*************************************************************/
       
// void command_globalgps(int8_t system_id, int8_t component_id, int32_t upTime, float lat, float lon, float alt, float gps_alt, uint16_t heading) {
//   int16_t velx = 0; //x speed
//   int16_t vely = 0; //y speed
//   int16_t velz = 0; //z speed


//   // Initialize the required buffers
//   mavlink_message_t msg;
//   uint8_t buf[MAVLINK_MAX_PACKET_LEN];

//   // Pack the message
//   mavlink_msg_global_position_int_pack(system_id, component_id, &msg, upTime, lat , lon , gps_alt, alt , velx, vely, velz, heading);

//   // Copy the message to the send buffer
//   uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
//   // Send the message (.write sends as bytes)
//   Serial.write(buf, len);
// }

// /************************************************************
// * @brief Send some system data parameters (battery, etc)
// * @param 
// * @return void
// *************************************************************/
       
void command_status(uint8_t system_id, uint8_t component_id, float battery_remaining, float voltage_battery, float current_battery) {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
    mavlink_msg_sys_status_pack(system_id, component_id, &msg, 32767, 32767, 32767, 500, voltage_battery * 1000.0, current_battery * 100.0, battery_remaining, 0, 0, 0, 0, 0, 0);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message (.write sends as bytes)
  Serial.write(buf, len);
}
