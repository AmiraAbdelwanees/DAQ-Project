/*****************************OTA Version*******************************************/
#include "fota.h"


#include <ArduinoJson.h>
#include "FS.h"
#include "SD.h"
#include <SPI.h>

/******************************bluetooth libarary**********************************************/
#include "BluetoothSerial.h"

#include "esp_bt_main.h"
#include "esp_bt_device.h"


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif
/******************************bluetooth configuration global variables**********************************************/
const char* Sim_num = "01032050930";
int value2 = 50;
int percentage = 89;
bool plugged = true;

BluetoothSerial SerialBT;
boolean confirmRequestPending = true;
const char* BT_Password = "1234";
BTScanResults discoveredDevice;
BTAdvertisedDevice discoveredDeviceName;
#define REMOVE_BONDED_DEVICES 1  // <- set to 1 to remove
#define PAIR_MAX_DEVICES 3
uint8_t pairedDeviceBtAddr[PAIR_MAX_DEVICES][6];
char bda_str[18];
/******************************bluetooth login global variables**********************************************/
// User account structure
struct UserAccount {
  String username;
  String password;
  int userType;
};
// User account database
int Log_count=0;
UserAccount adminAccount;
UserAccount viewOnlyAccounts[2];
int numViewOnlyAccounts = 0;

bool isPairingRequested = false;
bool isLoggedIn = false;
bool riceived_configuration = false;
bool isadmin=false;
/**************gps libaray ********************************/
String NMEA;
int gps_n;
int gps_num;
char c;
float distance = 0;
SemaphoreHandle_t uart1TxMutex;  // Mutex for UART transmission
SemaphoreHandle_t uart1RxMutex;  // Mutex for UART reception
int count_var = 0;
int bufferIndexGPS;
char raduis_buffer[6];
char lat_buffer[7];
char lng_buffer[7];
double initialLatitude = 30.08032;
double initialLongitude = 31.24543;
float latitude = initialLatitude;
float longitude = initialLongitude;
double raduis_of_C1;


/**************LIBRARIES********************************/
#include <ArduinoQueue.h>
#include <ArduinoJson.h>
#include "time.h"
#include <ACAN_ESP32.h>
#include "RTClib.h"
RTC_DS1307 rtc;
#define USING_TINYGPS_LIBRARY \
https:  //github.com/mikalhart/TinyGPSPlus.git

// See all AT commands, if wanted

//String latitude = "";
//String longitude = "";

int gps_counter = 0;

//#define TINY_GSM_MODEM_HAS_BLUETOOTH
bool connected = 0;

#include <Ticker.h>
#include <SPI.h>
//#include <SD.h>

#ifdef USING_TINYGPS_LIBRARY
// Use TinyGPS NMEA math analysis library
#include <TinyGPS++.h>
TinyGPSPlus gps;

#endif

int mqtt_not_sent;

Ticker tick;

//TinyGsmClientBluetooth hello();

#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP 60           // Time ESP32 will go to sleep (in seconds)

/************************DEFINES***************************/
#define MSG_BUFFER_SIZE (50)

#define TXD2 17
/**************************Thresholds****************************/
#define INFLUXDB_NOTCONNECTED_THRESHOLD 30
#define WIFI_NOTCONNECTED_THRESHOLD 5
#define MSG_SIZE 31
//——————————————————————————————————————————————————————————————————————————————
//  ESP32 Desired Bit Rate
//——————————————————————————————————————————————————————————————————————————————

static const uint32_t DESIRED_BIT_RATE = 1000UL * 500UL;

//——————————————————————————————————————————————————————————————————————————————
//   SETUP
//——————————————————————————————————————————————————————————————————————————————

/************************************GLobal Variables***************************************************************/
//GPS
double lat;
double lon;

struct message {
  unsigned int ID;
  unsigned char payload[8];
} can_message1;
CANMessage can_message2;
ArduinoQueue<CANMessage> smallQueue(1000);
//ArduinoQueue<CANMessage> CopyQueue(10000);
bool can_isupdated =1 ;

String dataMessage;
String data_string;
String vehicle_data_string;
String data_influxdb;

CANMessage frames[MSG_SIZE];

DynamicJsonDocument doc(2048);
struct arraystruct {
  CANMessage frames[MSG_SIZE];
  unsigned long milliseconds;

} arraystruct1, arraystruct2;



ArduinoQueue<arraystruct> copyQueue(100);

//Temp
float temp1;
float temp2;
float temp3;
float temp4;
//float mainPlus_temp ;

uint16_t temp_max;
uint16_t temp_min;
uint16_t temp_av;
int temp_max_s;
int temp_min_s;
int temp_av_s;


//SOC-SOH
float soc_KF_Average;
float soc_KF_Min;
float soc_CC_Average;
float soc_CC_Min;
float prev_soc;
float soh_av;
float soh_min;
float soh_imp;
float soh_cap;

//Current-Voltage
float pack_current;
double pack_current_ma;
double instant_charge_current_limit;
double cont_charge_current_limit;
double instant_discharge_current_limit;
double cont_discharge_current_limit;
double cont_charge_current;
double cont_discharge_current;
double peak_charge_current;
double peak_discharge_current;
unsigned int debug_state;

bool main_plus_state;
bool brake_contactor_state;
bool bms_enable_switch;
bool MstStMch_bool_CANChargingEN;

int by7_25;

unsigned int v1;
unsigned int v2;
unsigned int v3;
unsigned int v4;
unsigned int v5;
unsigned int v6;
unsigned int v7;
unsigned int v8;
unsigned int v9;
unsigned int v10;
unsigned int v11;
unsigned int v12;
unsigned int v13;
unsigned int v14;
unsigned int v15;




double voltage_before_contactor;
double voltage_after_contactor;

unsigned int v16;
uint16_t max_v;
uint16_t min_v;
uint32_t pack_voltage;
unsigned int system_state;

//counters
char not_connected_counter;
unsigned int counter;
unsigned int oled_counter;
//Vehicle Speed
float vehicle_speed;

//IDs
unsigned int vehicle_id;
unsigned int drive_id;
uint32_t charging_id;

//TimeStamp
unsigned long Epoch_Time;
unsigned long queued_timestamp;
unsigned long loop_timer;

//CAN MSG
unsigned long lastMsg = 0;
char msg[MSG_BUFFER_SIZE];
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
unsigned char ext = 0;

//Booleans
bool NOT_CONNECTED_MODE;
bool CALIB_request;
bool CALIB_resuest_sent;
bool charge_done;
bool MSG_38;
bool MSG_33;
bool MSG_755;
bool MSG_37;
bool MSG_34;
bool MSG_36;
bool MSG_741;
bool MSG_191;
bool MSG_19;
bool MSG_112;
bool MSG_11;
bool MSG_47;
bool MSG_54;
bool MSG_53;
bool MSG_40;
bool MSG_25;
bool MSG_27;
bool MSG_17;
bool MSG_62;
bool MSG_56;
bool MSG_541;
// new variables   // need intialization

double charging_CC_current;
double charging_CC_voltage;

double charging_CV_current;
double charging_CV_voltage;

double charging_TK_current;
double charging_TK_voltage;

unsigned int charging_Enum_SysState;
unsigned int charg_u8_Debug_State_Charging;
uint16_t Debug_Charging_Substate;

unsigned int u8_1;
unsigned int u8_2;
unsigned int u8_3;
unsigned int u8_4;
unsigned int u8_5;
unsigned int u8_6;

unsigned int BMS_bl;
unsigned int BMS_bl2;

uint16_t u16_1;
uint16_t u16_2;
uint16_t u16_3;
uint16_t u16_4;

uint32_t u32_1;
uint32_t u32_2;

uint32_t u32_3;
uint32_t u32_4;

uint32_t DisCycles_NoOfCycles;
uint32_t DisCycles_Ah;

unsigned int OverVoltage_Counts;
unsigned int UnderVoltage_Counts;
unsigned int OverTemperature_Counts;
unsigned int UnderTemperature_Counts;
unsigned int OverCharge_Counts;
unsigned int OverDischarge_Counts;
unsigned int OverCurrent_Counts;
unsigned int OperationalState_Counts;


unsigned int BalancingErrors_Counts;
unsigned int BrakeConErr_Counts;
unsigned int BrakeTempErr_Counts;
unsigned int VoltageMeasErr_Counts;
unsigned int MainFB_Err_Counts;
unsigned int BrakeFB_Err_Counts;
//  62
float mainPlus_temp;
float Brake_temp;
int RlysHndlr_bl_byte4;
int RlysHndlr_bl_byte5;
float Shunt_Sensor_temp;

uint32_t SOH_Cap_Est_Q;


float SOH_Cap_Ratio;
int Charge_Warning_6;


// new frames
bool MSG_710;
bool MSG_711;
bool MSG_712;
bool MSG_145;
bool MSG_146;
bool MSG_147;
bool MSG_148;
bool MSG_171;
bool MSG_10;
bool MSG_13;

bool MSG_59;
bool MSG_58;
bool MSG_1806E5F4;

int by0_17;
int by1_17;
int by2_17;
int by3_17;
int by4_17;
int by5_17;
int by6_17;
int by7_17;
//extended
float Max_Voltage;
float Max_Current;
unsigned int control;

float output_Voltage;
float output_Current;
//Strings

bool SENT;

bool READY;
bool RECIEVED;
//Strings
//String timeStamp ;
String mqtt_message;

unsigned long milliseconds;

unsigned long start = 0, end = 0, elapsed = 0, start_loop = 0, end_loop = 0;

DateTime now;






void gps_setup(){
     modem.sendAT("+QGPSCFG=\"outport\",\"usbnmea\"");
     modem.waitResponse(1000);

    modem.sendAT("+QGPSCFG=\"nmeasrc\",1");
    modem.waitResponse(1000);

    modem.sendAT("+QGPSCFG=\"gpsnmeatype\",2"); //2
    modem.waitResponse(1000);

    modem.sendAT("+QGPSCFG=\"gnssnmeatype\",1");
    modem.waitResponse(1000);

    modem.sendAT("+QGPSCFG=\"gnssconfig\",0");
    modem.waitResponse(1000);

    modem.sendAT("+QGPSCFG=\"autogps\",0");
    modem.waitResponse(1000);

    modem.sendAT("+QGPS=1"); //gps is more power consumption 
    modem.waitResponse(1000);
     delay(5000);

     modem.sendAT("+QGPSLOC=1"); //
     modem.waitResponse(1000);//

//delay(10000);///fast mqtt debug

}


void init__global_variables() {
  instant_charge_current_limit = 0;
  cont_charge_current_limit = 0;
  cont_discharge_current_limit = 0;
  instant_discharge_current_limit = 0;
  cont_charge_current = 0;
  cont_discharge_current = 0;
  peak_charge_current = 0;
  peak_discharge_current = 0;
  debug_state = 0;
  main_plus_state = 0;
  brake_contactor_state = 0;
  bms_enable_switch = 0;
  pack_voltage = 0;
  charging_id = 0;


  NOT_CONNECTED_MODE = 0;  //
  loop_timer = 0;
  counter = 0;
  not_connected_counter = 0;
  MSG_38 = 0;
  MSG_33 = 0;
  MSG_755 = 0;
  MSG_37 = 0;
  MSG_34 = 0;
  MSG_36 = 0;
  MSG_741 = 0;
  MSG_191 = 0;
  MSG_19 = 0;
  MSG_112 = 0;
  MSG_11 = 0;
  MSG_47 = 0;
  MSG_541 = 0;
  MSG_54 = 0;
  MSG_53 = 0;
  MSG_40 = 0;
  MSG_25 = 0;
  MSG_27 = 0;
  MSG_17 = 0;
  MSG_62 = 0;
  MSG_56 = 0;

  CALIB_request = 0;
  CALIB_resuest_sent = 0;
  data_string = "";
  vehicle_data_string = "";
  data_influxdb = "";
  oled_counter = 0;

  MSG_710 = 0;
  MSG_711 = 0;
  MSG_712 = 0;
  MSG_145 = 0;
  MSG_146 = 0;
  MSG_147 = 0;
  MSG_148 = 0;
  MSG_171 = 0;
  MSG_10 = 0;
  MSG_13 = 0;
  MSG_59 = 0;
  MSG_58 = 0;

  //new

  charging_CC_current = 0;
  charging_CC_voltage = 0;

  charging_CV_current = 0;
  charging_CV_voltage = 0;

  charging_TK_current = 0;
  charging_TK_voltage = 0;

  charging_Enum_SysState = 0;
  charg_u8_Debug_State_Charging = 0;
  Debug_Charging_Substate = 0;

  u8_1 = 0;
  u8_2 = 0;
  u8_3 = 0;
  u8_4 = 0;
  u8_5 = 0;
  u8_6 = 0;

  BMS_bl = 0;

  u16_1 = 0;
  u16_2 = 0;
  u16_3 = 0;
  u16_4 = 0;

  u32_1 = 0;
  u32_2 = 0;

  u32_3 = 0;
  u32_4 = 0;

  DisCycles_NoOfCycles = 0;
  DisCycles_Ah = 0;

  OverVoltage_Counts = 0;
  UnderVoltage_Counts = 0;
  OverTemperature_Counts = 0;
  UnderTemperature_Counts = 0;
  OverCharge_Counts = 0;
  OverDischarge_Counts = 0;
  OverCurrent_Counts = 0;
  OperationalState_Counts = 0;
  BalancingErrors_Counts = 0;
  BrakeConErr_Counts = 0;
  BrakeTempErr_Counts = 0;
  VoltageMeasErr_Counts = 0;
  MainFB_Err_Counts = 0;
  BrakeFB_Err_Counts = 0;

  mainPlus_temp = 0;
  Brake_temp = 0;
  RlysHndlr_bl_byte4 = 0;
  RlysHndlr_bl_byte5 = 0;
  Shunt_Sensor_temp = 0;

  SOH_Cap_Est_Q = 0;

  MstStMch_bool_CANChargingEN = 0;

  SOH_Cap_Ratio = 0;
  Charge_Warning_6 = 0;

  //17
  by0_17 = 0;
  by1_17 = 0;
  by2_17 = 0;
  by3_17 = 0;
  by4_17 = 0;
  by5_17 = 0;
  by6_17 = 0;
  by7_17 = 0;
  by7_25 = 0;
  //extended
  MSG_1806E5F4 = 0;
  Max_Voltage = 0;
  Max_Current = 0;
  control = 0;

  output_Voltage = 0;
  output_Current = 0;
}

void modem_reset() {
  pinMode(MODEM_PWRKEY, OUTPUT);

  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(3500);  //Need delay
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(3500);  //Need delay
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(3500);  //Need delay
  Serial.println("resetting");
}
void setup() {

  Serial.begin(115200);
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  SerialAT.setRxBufferSize(500); ///500 ////for debug
  Serial.println("Start modem...");

  modem_reset();
  delay(10000);
  return_uart();



  Serial.println("Modem Response Started.");
  fota(false);
  uart1TxMutex = xSemaphoreCreateMutex();
  uart1RxMutex = xSemaphoreCreateMutex();
  init__global_variables();
  ACAN_ESP32_Settings settings(DESIRED_BIT_RATE);
  settings.mTxPin = GPIO_NUM_25;
  settings.mRxPin = GPIO_NUM_26;
  settings.mRequestedCANMode = ACAN_ESP32_Settings::NormalMode;
  const uint32_t errorCode = ACAN_ESP32::can.begin(settings);
  if (errorCode == 0) {
    Serial.print("Bit Rate prescaler: ");
    Serial.println(settings.mBitRatePrescaler);
    Serial.print("Time Segment 1:     ");
    Serial.println(settings.mTimeSegment1);
    Serial.print("Time Segment 2:     ");
    Serial.println(settings.mTimeSegment2);
    Serial.print("RJW:                ");
    Serial.println(settings.mRJW);
    Serial.print("Triple Sampling:    ");
    Serial.println(settings.mTripleSampling ? "yes" : "no");
    Serial.print("Actual bit rate:    ");
    Serial.print(settings.actualBitRate());
    Serial.println(" bit/s");
    Serial.print("Exact bit rate ?    ");
    Serial.println(settings.exactBitRate() ? "yes" : "no");
    Serial.print("Sample point:       ");
    Serial.print(settings.samplePointFromBitStart());
    Serial.println("%");
    Serial.println("Configuration OK!");
  } else {
    Serial.print("Configuration error 0x");
    Serial.println(errorCode, HEX);
  }

  Serial.println("Trying to connect!");
  while (modem.getRegistrationStatus() != 1) {
    Serial.print(".");
    delay(100);
    not_connected_counter++;

    if (not_connected_counter >= 300) {

      ESP.restart();
    }
  }

  Serial.println(" connected !");

  modem.sendAT("+QIDEACT=1");
  modem.waitResponse(100);

  modem.sendAT("+qicsgp=1,1,\"internet\",\"\",\"\",0");
  modem.waitResponse(100);

  modem.sendAT("+QIACT=1");
  modem.waitResponse(100);

  modem.sendAT("+CREG?");
  modem.waitResponse(100);

  gps_setup();
  Bluetooth_Setup(true);
  

  modem.sendAT("+cgpaddr=1");
  modem.waitResponse(100);

  // broker.emqx.io     3.77.81.209
  modem.sendAT("+QMTOPEN=1,\"3.77.81.209\",1883");
  modem.waitResponse(100);
  delay(500);

  modem.sendAT("+qmtconn = 1, \"hello\"");
  modem.waitResponse(1000);
  delay(500);

  modem.sendAT("+QMTPUBEX=1,0,0,0,\"simcomtest\",15");
  modem.waitResponse(100);
  SerialAT.println("mqtt_testone\x1A");
  delay(10);

  xTaskCreatePinnedToCore(
    TaskCANcode, /* Task function. */
    "TaskCANcode",   /* name of task. */
    5000,      /* Stack size of task */
    NULL,      /* parameter of the task */
    2,         /* priority of the task */
    NULL,    /* Task handle to keep track of created task */
    0);        /* pin task to core 0 */

  xTaskCreatePinnedToCore(taskGPScode, "taskGPScode", 2000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskMQTTcode, "TaskMQTTcode", 10000, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskBluetoothcode,"TaskBluetoothcode",5000,NULL,0,NULL,0);
}


//Task1code: blinks an LED every 1000 ms
void TaskCANcode(void* pvParameters) {
  for (;;) {
    CANMessage frame;

    start = millis();
    READY = 0;
    int i = 0;  ///

    MSG_38 = 0;   // done
    MSG_33 = 0;   // done
    MSG_37 = 0;   // done
    MSG_34 = 0;   // done
    MSG_36 = 0;   // done
    MSG_741 = 0;  // done
    MSG_191 = 0;  // done
    MSG_19 = 0;   // done
    MSG_112 = 0;  // done
    MSG_11 = 0;   // done
    MSG_47 = 0;   // done
    MSG_54 = 0;   // done
    MSG_53 = 0;   // done
    MSG_40 = 0;   // done
    MSG_25 = 0;   // done
    MSG_27 = 0;   //done
    MSG_17 = 0;   // done
    MSG_62 = 0;   // done
    MSG_56 = 0;   // done

    MSG_541 = 0;  // done

    MSG_710 = 0;  //not

    MSG_711 = 0;  // not

    MSG_145 = 0;  // not

    MSG_146 = 0;  // not

    MSG_147 = 0;  // not

    MSG_148 = 0;  // not

    MSG_171 = 0;  //not

    MSG_10 = 0;  // not

    MSG_13 = 0;  // not

    MSG_59 = 0;  // not

    MSG_58 = 0;  // not done


    //vTaskDelay(100);
         can_isupdated=1;
         char no_can_itrr = 0;
    for (int i = 0; i < MSG_SIZE;) {
      if (ACAN_ESP32::can.receive(frame)) {
        switch (frame.id) {
          case 0x34:  //found
            if (!MSG_34) {
              MSG_34 = 1;
              arraystruct1.frames[0] = frame;
              i = i + 1;
            }
            break;

          case 0x112:  //found
            if (!MSG_112) {
              MSG_112 = 1;
              arraystruct1.frames[1] = frame;
              i = i + 1;
            }
            break;

          case 0x11:  //  found
            if (!MSG_11) {
              MSG_11 = 1;
              arraystruct1.frames[2] = frame;
              i = i + 1;
            }
            break;
            /****************************************************************************************************/

          case 0x741:  // found
            if (!MSG_741) {
              MSG_741 = 1;
              arraystruct1.frames[3] = frame;
              i = i + 1;
            }
            break;

          case 0x38:  //  found
            if (!MSG_38) {
              MSG_38 = 1;
              arraystruct1.frames[4] = frame;
              i = i + 1;
            }
            break;

          case 0x25:  // found
            if (!MSG_25) {
              MSG_25 = 1;
              arraystruct1.frames[5] = frame;
              i = i + 1;
            }
            break;
            /****************************************************************************************************/
          case 0x37:  // found
            if (!MSG_37) {
              MSG_37 = 1;
              arraystruct1.frames[6] = frame;
              i = i + 1;
            }
            break;

          case 0x36:  // found
            if (!MSG_36) {
              MSG_36 = 1;
              arraystruct1.frames[7] = frame;
              i = i + 1;
            }
            break;

          case 0x27:  //  found
            if (!MSG_27) {
              MSG_27 = 1;
              arraystruct1.frames[8] = frame;
              i = i + 1;
            }
            break;


            /****************************************************************************************************/
          case 0x17:  //  found
            if (!MSG_17) {
              MSG_17 = 1;
              arraystruct1.frames[9] = frame;
              i = i + 1;
            }
            break;
            /****************************************************************************************************/

          case 0x53:  // found
            if (!MSG_53) {
              MSG_53 = 1;
              arraystruct1.frames[10] = frame;
              i = i + 1;
            }
            break;

          case 0x40:  // found
            if (!MSG_40) {
              MSG_40 = 1;
              arraystruct1.frames[11] = frame;
              i = i + 1;
            }
            break;

          case 0x56:  // found
            if (!MSG_56) {
              MSG_56 = 1;
              arraystruct1.frames[12] = frame;
              i = i + 1;
            }
            break;

          case 0x62:  // found
            if (!MSG_62) {
              MSG_62 = 1;
              arraystruct1.frames[13] = frame;
              i = i + 1;
            }
            break;
            /****************************************************************************************************/

          case 0x47:  // found
            if (!MSG_47) {
              MSG_47 = 1;
              arraystruct1.frames[14] = frame;
              i = i + 1;
            }
            break;

          case 0x191:  //found
            if (!MSG_191) {
              MSG_191 = 1;
              arraystruct1.frames[15] = frame;
              i = i + 1;
            }
            break;
            /****************************************************************************************************/
          case 0x33:  //  found
            if (!MSG_33) {
              MSG_33 = 1;
              arraystruct1.frames[16] = frame;
              i = i + 1;
            }
            break;

          case 0x54:  // found
            if (!MSG_54) {
              MSG_54 = 1;
              arraystruct1.frames[17] = frame;
              i = i + 1;
            }
            break;

          case 0x19:  // found
            if (!MSG_19) {
              MSG_19 = 1;
              arraystruct1.frames[18] = frame;
              i = i + 1;
            }
            break;
          case 0x541:  // found
            if (!MSG_541) {
              MSG_541 = 1;
              arraystruct1.frames[19] = frame;
              i = i + 1;
            }
            break;
          case 0x710:  // found
            if (!MSG_710) {
              MSG_710 = 1;
              arraystruct1.frames[20] = frame;
              i = i + 1;
            }
            break;

          case 0x711:  // found
            if (!MSG_711) {
              MSG_711 = 1;
              arraystruct1.frames[21] = frame;
              i = i + 1;
            }
            break;

          case 0x145:  // found
            if (!MSG_145) {
              MSG_145 = 1;
              arraystruct1.frames[22] = frame;
              i = i + 1;
            }
            break;

          case 0x146:  // found
            if (!MSG_146) {
              MSG_146 = 1;
              arraystruct1.frames[23] = frame;
              i = i + 1;
            }
            break;

          case 0x147:  // found
            if (!MSG_147) {
              MSG_147 = 1;
              arraystruct1.frames[24] = frame;
              i = i + 1;
            }
            break;
          case 0x148:  // found
            if (!MSG_148) {
              MSG_148 = 1;
              arraystruct1.frames[25] = frame;
              i = i + 1;
            }
            break;

          case 0x171:  // found
            if (!MSG_171) {
              MSG_171 = 1;
              arraystruct1.frames[26] = frame;
              i = i + 1;
            }
            break;

          case 0x10:  // found
            if (!MSG_10) {
              MSG_10 = 1;
              arraystruct1.frames[27] = frame;
              i = i + 1;
            }
            break;

          case 0x13:  // found
            if (!MSG_13) {
              MSG_13 = 1;
              arraystruct1.frames[28] = frame;
              i = i + 1;
            }
            break;

          case 0x59:  // found
            if (!MSG_59) {
              MSG_59 = 1;
              arraystruct1.frames[29] = frame;
              i = i + 1;
            }
            break;

          case 0x58:  // found
            if (!MSG_58) {
              MSG_58 = 1;
              arraystruct1.frames[30] = frame;
              i = i + 1;
            }
            break;


          default:
            break;
        }
      }
      else{
        can_isupdated=0;
        ++no_can_itrr;
        if(no_can_itrr > MSG_SIZE )
             break;
        
        }
    }

    vTaskDelay(80); //100
    arraystruct1.milliseconds = millis();
    copyQueue.enqueue(arraystruct1);
    READY = 1;
    end = millis();
  }
}

void loop() {
  
}

void TaskMQTTcode(void* pvParameters){
    test_uart() ;
    modem.sendAT("E0");
    modem.waitResponse(100);
  while(1){
  start_loop = micros();
  bool UPDATED = 0;
  CANMessage frames_copy[MSG_SIZE];  // revise
  if (copyQueue.itemCount()) {
    arraystruct2 = copyQueue.dequeue();

    for (int i = 0; i < MSG_SIZE; i++) {
      frames_copy[i] = arraystruct2.frames[i];
    }
    UPDATED = 1;
  }


if (UPDATED) {
  if (can_isupdated) {
    uint32_t start_update = millis();
    //34
    soh_av = (frames_copy[0].data[1] * 256 + frames_copy[0].data[0]);
    soh_min = (frames_copy[0].data[3] * 256 + frames_copy[0].data[2]);
    // 112
    v5 = (frames_copy[1].data[1] * 256 + frames_copy[1].data[0]);
    v6 = (frames_copy[1].data[3] * 256 + frames_copy[1].data[2]);
    v7 = (frames_copy[1].data[5] * 256 + frames_copy[1].data[4]);
    v8 = (frames_copy[1].data[7] * 256 + frames_copy[1].data[6]);
    //11
    v1 = (frames_copy[2].data[1] * 256 + frames_copy[2].data[0]);
    v2 = (frames_copy[2].data[3] * 256 + frames_copy[2].data[2]);
    v3 = (frames_copy[2].data[5] * 256 + frames_copy[2].data[4]);
    v4 = (frames_copy[2].data[7] * 256 + frames_copy[2].data[6]);
    //741

    pack_current_ma = (frames_copy[3].data[3] * 16777216 + frames_copy[3].data[2] * 65536 + frames_copy[3].data[1] * 256 + frames_copy[3].data[0]);
    if (pack_current_ma > 2147483647) {
      pack_current_ma = -2147483648 - (2147483648 - pack_current_ma);
    }
    temp1 = frames_copy[3].data[4];
    temp2 = frames_copy[3].data[5];
    temp3 = frames_copy[3].data[6];
    temp4 = frames_copy[3].data[7];
    if (temp1 > 127) { temp1 = -128 - (128 - temp1); }
    if (temp2 > 127) { temp2 = -128 - (128 - temp2); }
    if (temp3 > 127) { temp3 = -128 - (128 - temp3); }
    if (temp4 > 127) { temp4 = -128 - (128 - temp4); }
    //38

    max_v = (frames_copy[4].data[1] * 256 + frames_copy[4].data[0]);
    min_v = (frames_copy[4].data[3] * 256 + frames_copy[4].data[2]);

    //25

    debug_state = (frames_copy[5].data[4]);
    by7_25 = frames_copy[5].data[7];


    //37
    pack_voltage = (frames_copy[6].data[3] * 16777216 + frames_copy[6].data[2] * 65536 + frames_copy[6].data[1] * 256 + frames_copy[6].data[0]);

    //36

    temp_max = (frames_copy[7].data[1] * 256 + frames_copy[7].data[0]);
    temp_av = (frames_copy[7].data[3] * 256 + frames_copy[7].data[2]);
    temp_min = (frames_copy[7].data[5] * 256 + frames_copy[7].data[4]);

    if (temp_max > 32767) {
      temp_max = -32768 - (32768 - temp_max);
    } else {
      temp_max = temp_max;
    }
    if (temp_min > 32767) {
      temp_min_s = -32768 - (32768 - temp_min);
    } else {
      temp_min_s = temp_min;
    }

    if (temp_av > 32767) {
      temp_av = -32768 - (32768 - temp_av);
    } else {
      temp_av = temp_av;
    }

    //27
    system_state = (frames_copy[8].data[0]);
    charge_done = (frames_copy[8].data[3] & 0x10) / 0x10;

    //17

    by0_17 = frames_copy[9].data[0];  // 3        // done
    by1_17 = frames_copy[9].data[1];  // 8
    by2_17 = frames_copy[9].data[2];
    by3_17 = frames_copy[9].data[3];
    by4_17 = frames_copy[9].data[4];
    by5_17 = frames_copy[9].data[5];
    by6_17 = frames_copy[9].data[6];
    by7_17 = frames_copy[9].data[7];

    //53
    instant_discharge_current_limit = (frames_copy[10].data[3] * 16777216 + frames_copy[10].data[2] * 65536 + frames_copy[10].data[1] * 256 + frames_copy[10].data[0]);
    cont_discharge_current_limit = (frames_copy[10].data[7] * 16777216 + frames_copy[10].data[6] * 65536 + frames_copy[10].data[5] * 256 + frames_copy[10].data[4]);
    if (instant_discharge_current_limit > 2147483647) {
      instant_discharge_current_limit = -2147483648 - (2147483648 - instant_discharge_current_limit);
    }
    if (cont_discharge_current_limit > 2147483647) {
      cont_discharge_current_limit = -2147483648 - (2147483648 - cont_discharge_current_limit);
    }

    //40
    charging_Enum_SysState = frames_copy[11].data[0];
    charg_u8_Debug_State_Charging = frames_copy[11].data[1];
    Debug_Charging_Substate = frames_copy[11].data[3] * 256 + frames_copy[11].data[2];

    //56

    soh_cap = (frames_copy[12].data[1] * 256 + frames_copy[12].data[0]);
    soh_imp = (frames_copy[12].data[5] * 256 + frames_copy[12].data[4]);

    //62

    mainPlus_temp = (frames_copy[13].data[1] * 256 + frames_copy[13].data[0]);
    Brake_temp = (frames_copy[13].data[3] * 256 + frames_copy[13].data[2]);

    RlysHndlr_bl_byte4 = frames_copy[13].data[4];                                   // 32
    RlysHndlr_bl_byte5 = frames_copy[13].data[5];                                   // 32
    Shunt_Sensor_temp = (frames_copy[13].data[7] * 256 + frames_copy[13].data[6]);  //48

    if (mainPlus_temp > 32767) {
      mainPlus_temp = -32768 - (32768 - mainPlus_temp);
    } else {
      mainPlus_temp = mainPlus_temp;
    }
    if (Brake_temp > 32767) {
      Brake_temp = -32768 - (32768 - Brake_temp);
    } else {
      Brake_temp = Brake_temp;
    }

    if (Shunt_Sensor_temp > 32767) {
      Shunt_Sensor_temp = -32768 - (32768 - Shunt_Sensor_temp);
    } else {
      Shunt_Sensor_temp = Shunt_Sensor_temp;
    }


    //47
    charging_id = (frames_copy[14].data[3] * 16777216 + frames_copy[14].data[2] * 65536 + frames_copy[14].data[1] * 256 + frames_copy[14].data[0]);


    //191

    v13 = (frames_copy[15].data[1] * 256 + frames_copy[15].data[0]);
    v14 = (frames_copy[15].data[3] * 256 + frames_copy[15].data[2]);
    v15 = (frames_copy[15].data[5] * 256 + frames_copy[15].data[4]);
    v16 = (frames_copy[15].data[7] * 256 + frames_copy[15].data[6]);

    //33
    soc_KF_Average = ((frames_copy[16].data[1] * 256 + frames_copy[16].data[0]));
    soc_KF_Min = ((frames_copy[16].data[3] * 256 + frames_copy[16].data[2]));
    soc_CC_Average = ((frames_copy[16].data[5] * 256 + frames_copy[16].data[4]));
    soc_CC_Min = ((frames_copy[16].data[7] * 256 + frames_copy[16].data[6]));


    //54
    voltage_before_contactor = (frames_copy[17].data[3] * 16777216 + frames_copy[17].data[2] * 65536 + frames_copy[17].data[1] * 256 + frames_copy[17].data[0]);
    voltage_after_contactor = (frames_copy[17].data[7] * 16777216 + frames_copy[17].data[6] * 65536 + frames_copy[17].data[5] * 256 + frames_copy[17].data[4]);

    //19

    v9 = (frames_copy[18].data[1] * 256 + frames_copy[18].data[0]);
    v10 = (frames_copy[18].data[3] * 256 + frames_copy[18].data[2]);
    v11 = (frames_copy[18].data[5] * 256 + frames_copy[18].data[4]);
    v12 = (frames_copy[18].data[7] * 256 + frames_copy[18].data[6]);
    //541
    voltage_before_contactor = (frames_copy[19].data[3] * 16777216 + frames_copy[19].data[2] * 65536 + frames_copy[19].data[1] * 256 + frames_copy[19].data[0]);
    voltage_after_contactor = (frames_copy[19].data[7] * 16777216 + frames_copy[19].data[6] * 65536 + frames_copy[19].data[5] * 256 + frames_copy[19].data[4]);
    //710

    charging_CC_current = (frames_copy[20].data[3] * 16777216 + frames_copy[20].data[2] * 65536 + frames_copy[20].data[1] * 256 + frames_copy[20].data[0]);
    charging_CC_voltage = (frames_copy[20].data[7] * 16777216 + frames_copy[20].data[6] * 65536 + frames_copy[20].data[5] * 256 + frames_copy[20].data[4]);

    if (charging_CC_current > 2147483647) {
      charging_CC_current = -2147483648 - (2147483648 - charging_CC_current);
    }
    if (charging_CC_voltage > 2147483647) {
      charging_CC_voltage = -2147483648 - (2147483648 - charging_CC_voltage);
    }
    //711
    charging_CV_current = (frames_copy[21].data[3] * 16777216 + frames_copy[21].data[2] * 65536 + frames_copy[21].data[1] * 256 + frames_copy[21].data[0]);
    charging_CV_voltage = (frames_copy[21].data[7] * 16777216 + frames_copy[21].data[6] * 65536 + frames_copy[21].data[5] * 256 + frames_copy[21].data[4]);

    if (charging_CV_current > 2147483647) {
      charging_CV_current = -2147483648 - (2147483648 - charging_CV_current);
    }
    if (charging_CV_voltage > 2147483647) {
      charging_CV_voltage = -2147483648 - (2147483648 - charging_CV_voltage);
    }
    //145
    u8_1 = frames_copy[22].data[0];
    u8_2 = frames_copy[22].data[1];
    u8_3 = frames_copy[22].data[2];
    u8_4 = frames_copy[22].data[3];
    u8_5 = frames_copy[22].data[4];
    u8_6 = frames_copy[22].data[5];
    BMS_bl = frames_copy[22].data[6];
    BMS_bl2 = frames_copy[22].data[7];
    //146

    u16_1 = (frames_copy[23].data[1] * 256 + frames_copy[23].data[0]);
    u16_2 = (frames_copy[23].data[3] * 256 + frames_copy[23].data[2]);
    u16_3 = (frames_copy[23].data[5] * 256 + frames_copy[23].data[4]);
    u16_4 = (frames_copy[23].data[7] * 256 + frames_copy[23].data[6]);
    //147
    u32_1 = (frames_copy[24].data[3] * 16777216 + frames_copy[24].data[2] * 65536 + frames_copy[24].data[1] * 256 + frames_copy[24].data[0]);
    u32_2 = (frames_copy[24].data[7] * 16777216 + frames_copy[24].data[6] * 65536 + frames_copy[24].data[5] * 256 + frames_copy[24].data[4]);
    //148
    u32_3 = (frames_copy[25].data[3] * 16777216 + frames_copy[25].data[2] * 65536 + frames_copy[25].data[1] * 256 + frames_copy[25].data[0]);
    u32_4 = (frames_copy[25].data[7] * 16777216 + frames_copy[25].data[6] * 65536 + frames_copy[25].data[5] * 256 + frames_copy[25].data[4]);
    //171
    DisCycles_NoOfCycles = (frames_copy[26].data[3] * 16777216 + frames_copy[26].data[2] * 65536 + frames_copy[26].data[1] * 256 + frames_copy[26].data[0]);
    DisCycles_Ah = (frames_copy[26].data[7] * 16777216 + frames_copy[26].data[6] * 65536 + frames_copy[26].data[5] * 256 + frames_copy[26].data[4]);

    //10
    OverVoltage_Counts = frames_copy[27].data[0];
    UnderVoltage_Counts = frames_copy[27].data[1];
    OverTemperature_Counts = frames_copy[27].data[2];
    UnderTemperature_Counts = frames_copy[27].data[3];
    OverCharge_Counts = frames_copy[27].data[4];
    OverDischarge_Counts = frames_copy[27].data[5];
    OverCurrent_Counts = frames_copy[27].data[6];
    OperationalState_Counts = frames_copy[27].data[7];

    //13

    BalancingErrors_Counts = frames_copy[28].data[0];
    BrakeConErr_Counts = frames_copy[28].data[1];
    BrakeTempErr_Counts = frames_copy[28].data[2];
    VoltageMeasErr_Counts = frames_copy[28].data[3];
    MainFB_Err_Counts = frames_copy[28].data[4];
    BrakeFB_Err_Counts = frames_copy[28].data[5];

    //59
    SOH_Cap_Est_Q = (frames_copy[29].data[3] * 16777216 + frames_copy[29].data[2] * 65536 + frames_copy[29].data[1] * 256 + frames_copy[29].data[0]);

    //58

    SOH_Cap_Ratio = (frames_copy[30].data[1] * 256 + frames_copy[30].data[0]);
    Charge_Warning_6 = frames_copy[30].data[6];  //48

    counter = counter + 1;

    doc.clear();
    doc["ts"] = arraystruct2.milliseconds;

    doc["lat"] = latitude;
    doc["lon"] = longitude;

    doc["cc"] = charging_CC_current;
    doc["ccv"] = charging_CC_voltage;

    doc["CVC"] = charging_CV_current;
    doc["CVV"] = charging_CV_voltage;
    // if (vehicle_id ==999)
    // {
    doc["veh_id"] = "R5";

    // }

    // else
    // {
    // doc["veh_id"] = String(vehicle_id);
    // }
    // // 145

    //  doc["8_1"] = u8_1;
    //  doc["8_2"] = u8_2;
    //  doc["8_3"] = u8_3;
    //  doc["8_4"] = u8_4;
    //  doc["8_5"] = u8_5;
    //  doc["8_6"] = u8_6;

    doc["bl"] = BMS_bl;
    doc["bl2"] = BMS_bl2;

    // 146

    //   doc["16_1"]= u16_1;
    //   doc["16_2"]= u16_2;
    //   doc["16_3"]= u16_3;
    //   doc["16_4"]= u16_4;
    // //147
    //   doc["32_1"]= u32_1;
    //   doc["32_2"]= u32_2;
    // // 148
    //   doc["32_3"]= u32_3;
    //   doc["32_4"]= u32_4;
    //171
    doc["C_D"] = DisCycles_NoOfCycles;
    doc["DAh"] = DisCycles_Ah;
    //10

    doc["*"] = OverVoltage_Counts;
    doc["~"] = UnderVoltage_Counts;
    doc["@"] = OverTemperature_Counts;
    doc["#"] = UnderTemperature_Counts;
    doc["$"] = OverCharge_Counts;
    doc["^"] = OverDischarge_Counts;
    doc["&"] = OverCurrent_Counts;
    doc["-"] = OperationalState_Counts;
    //13
    doc["+"] = BalancingErrors_Counts;
    doc["/"] = BrakeConErr_Counts;
    doc["B"] = BrakeTempErr_Counts;
    doc["m"] = VoltageMeasErr_Counts;
    doc["fb"] = MainFB_Err_Counts;
    doc["BFB"] = BrakeFB_Err_Counts;


    //62
    doc["MPT"] = mainPlus_temp;
    doc["BT"] = Brake_temp;
    doc["BL4"] = RlysHndlr_bl_byte4;
    doc["BL5"] = RlysHndlr_bl_byte5;

    doc["ST"] = Shunt_Sensor_temp;
    // 59
    doc["hcap"] = SOH_Cap_Est_Q;
    // 58


    doc["hcap_r"] = SOH_Cap_Ratio;
    doc["cw6"] = Charge_Warning_6;


    doc["vbc"] = voltage_before_contactor;
    doc["vac"] = voltage_after_contactor;

    doc["0_17"] = by0_17;
    doc["1_17"] = by1_17;
    doc["2_17"] = by2_17;
    doc["3_17"] = by3_17;
    doc["4_17"] = by4_17;
    doc["5_17"] = by5_17;
    doc["6_17"] = by6_17;
    doc["7_17"] = by7_17;

    doc["icl"] = instant_charge_current_limit;
    doc["ccl"] = cont_charge_current_limit;

    doc["idl"] = instant_discharge_current_limit;
    doc["cdl"] = cont_discharge_current_limit;

    doc["SS"] = charging_Enum_SysState;
    doc["SC"] = charg_u8_Debug_State_Charging;
    doc["CS"] = Debug_Charging_Substate;

    doc["ds"] = debug_state;
    doc["7_25"] = by7_25;

    doc["ss"] = system_state;
    doc["ChD"] = int(charge_done);

    doc["xv"] = max_v / 10000.0;
    doc["nv"] = min_v / 10000.0;
    doc["I"] = pack_current_ma;  //pack_current;

    doc["%kfv"] = soc_KF_Average / 100;
    doc["%kfn"] = soc_KF_Min / 100;

    doc["%"] = soc_CC_Average / 100;
    doc["%_n"] = soc_CC_Min / 100;

    doc["V"] = pack_voltage / 10000.0;

    doc["hv"] = soh_av / 100.0;
    doc["hn"] = soh_min / 100.0;
    doc["himp"] = soh_imp / 100.0;
    doc["hcyc"] = soh_cap / 100.0;


    doc["tx"] = temp_max / 100.0;
    doc["tn"] = temp_min / 100.0;
    doc["tv"] = temp_av / 100.0;

    doc["t1"] = temp1 * 119 / 100.0;
    doc["t2"] = temp2 * 119 / 100.0;
    doc["t3"] = temp3 * 119 / 100.0;
    doc["t4"] = temp4 * 119 / 100.0;

    doc["ch_id"] = charging_id;
    // //17
    doc["1"] = v1;
    doc["2"] = v2;
    doc["3"] = v3;
    doc["4"] = v4;
    doc["5"] = v5;
    doc["6"] = v6;
    doc["7"] = v7;
    doc["8"] = v8;
    doc["9"] = v9;

    doc["10"] = v10;
    doc["11"] = v11;
    doc["12"] = v12;
    doc["13"] = v13;
    doc["14"] = v14;
    doc["15"] = v15;
    doc["16"] = v16;
    //extended
    //   doc["mv"] =  Max_Voltage;
    //   doc["mc"] =  Max_Current;
    //   doc["c"] = control;

    // doc["ov"] =  output_Voltage;
    //   doc["oc"] =output_Current;
    data_string = "";
    //vehicle_data_string="";
    serializeJson(doc, data_string);
    data_influxdb = data_influxdb + data_string;
    data_influxdb = data_influxdb + "\x1A";
    Serial.println(millis() - start_update);
    //  dataMessage =  data_influxdb +"\r\n";
    //  Serial.println(dataMessage);
    if (counter >= 1) {


      if (mqtt_not_sent <= 60) {
        if (xSemaphoreTake(uart1TxMutex, portMAX_DELAY) == pdTRUE) {
          // Receive data from UART
          xSemaphoreTake(uart1RxMutex, portMAX_DELAY);
          modem.sendAT("+QMTPUBEX=1,0,0,0,\"simcomtest\"");
          if (modem.waitResponse(1) == 2) {
            mqtt_not_sent++;
          }
          SerialAT.println(data_influxdb.c_str());
          vTaskDelay(45);  //50

          xSemaphoreGive(uart1TxMutex);
          xSemaphoreGive(uart1RxMutex);  // Release RX mutex

          vTaskDelay(3);
        }

      } else {
        Serial.println("ESP RESTARTING!");
        ESP.restart();
      }
      data_influxdb = "";
      counter = 0;
    }
    // Serial.println(millis()-start_update);
  } else {

    vTaskDelay(1);
  }
}
  

}
}



/*---------------------------------------------------- gps function --------------------------------------------------------------*/

float getDistance(float flat1, float flon1, float flat2, float flon2) {

  // Variables haversine
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;

  // Calculations
  diflat  = radians(flat2-flat1);
  flat1 = radians(flat1);
  flat2 = radians(flat2);
  diflon = radians((flon2)-(flon1));

  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2 = cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;

  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
  
  dist_calc*=6371000.0; //Converting to meters

  return dist_calc;
}

void convertToGPGGA(char* qgpsloc ) {
    char time[11], latitude[12], longitude[13], date[7];
    float hdop ,altitude;
    int numSatellites;
   
    sscanf(qgpsloc, "+QGPSLOC: %10s,%11s,%*c,%12s,%*c,%f,%f,%*f,%*f,%*f,%*f,%6s,%d",
           time, latitude, longitude, &hdop, &altitude, date, &numSatellites);

    sprintf(qgpsloc ,"GPGGA,%s,%.9s,N,%.10s,E,3,%.2d,%.1f,%.1f,M,-22.5,M,,0000",
           time, latitude, longitude, numSatellites, hdop, altitude);
               unsigned char checksum = 0;
    for (int i = 0; i < strlen(qgpsloc); i++) {
        checksum ^= qgpsloc[i];
    }

    // Append the checksum to the GPGGA sentence
    sprintf(qgpsloc + strlen(qgpsloc), "*%02X\r\n", checksum);
    sprintf(qgpsloc, "$%s", qgpsloc);
    qgpsloc[1]='G';

       
           
}

  bool disableGPS() {
    modem.sendAT("+QGPSEND");
    if (modem.waitResponse(300) != 1) { return false; }
    return true;
  }

 bool enableGPS() {

    modem.sendAT("+QGPS=1");
    if (modem.waitResponse(300) != 1) { return false; }
    return true;
  }



void taskGPScode(void* pvParameters) {
  //test_uart() ;  //remove for debug only
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (1) {
    if ((xTaskGetTickCount() - lastWakeTime) >= pdMS_TO_TICKS(5000)) {
      distance = getDistance(latitude, longitude, initialLatitude, initialLongitude);
      lastWakeTime = xTaskGetTickCount();  // Update the wake time
      Serial.println(distance);
      Serial.println(gps_n);
      Serial.println(gps_num);
      if (distance > raduis_of_C1) {

        Serial.println("out of range ");
      }
    }

    ///gps_askForLocationRTOS(latitude, longitude);
    String res1;
    

    if (xSemaphoreTake(uart1TxMutex, portMAX_DELAY) == pdTRUE) {
      // Receive data from UART
      vTaskPrioritySet(NULL, 3);
      // vTaskSuspend(NULL);

      xSemaphoreTake(uart1RxMutex, portMAX_DELAY);
      // SerialAT.flush();

      unsigned long startTime = millis();
      while (SerialAT.available() && (millis() - startTime < 50)) {
        SerialAT.read();
      }
      //
      modem.sendAT("+QGPSLOC=1");
      SerialAT.flush();
      // modem.waitResponse(50);

      vTaskDelay(pdMS_TO_TICKS(40));  //50




      if (SerialAT.available()) {
        SerialAT.readStringUntil('\n');
        res1 = SerialAT.readStringUntil('\n');
        // SerialAT.readStringUntil('\n'); //for ok it for dubugg only
        //SerialAT.readStringUntil('\n');
      }

      // modem.waitResponse();
      res1.trim();

      // Example: data = uart.read();
      xSemaphoreGive(uart1TxMutex);
      xSemaphoreGive(uart1RxMutex);  // Release RX mutex
      vTaskPrioritySet(NULL, 1);


      //vTaskResume(NULL);

      vTaskDelay(pdMS_TO_TICKS(600));  //200
                                       //for debug

      if (res1.startsWith("+QGPSLOC")) {
        char res[100];
        res1.toCharArray(res, sizeof(res));
        ++gps_num;

        convertToGPGGA(res);

        for (int i = 0; i < sizeof(res); ++i) {
          if (gps.encode(res[i])) {

            if (gps.location.isValid()) {
              /*for debug only*/  // displayInfo();
              latitude = round(gps.location.lat() * 1000000000.0) / 1000000000.0;
              longitude = round(gps.location.lng() * 1000000000.0) / 1000000000.0;
              ++gps_n;
              break;

            } else {
              //    latitude = initialLatitude;
              //  longitude = initialLongitude;
              Serial.println("not valid");
              break;
            }
          }
        }
      }
      Serial.println(res1);
      //Serial.println(longitude);
      vTaskDelay(pdMS_TO_TICKS(2000));  // Wait for 1 second
    }
  }
}

void test_uart() {


  SerialAT.write("AT+IPR=921600\r\n");
  delay(1000);
  SerialAT.flush();
  SerialAT.begin(921600, SERIAL_8N1, MODEM_RX, MODEM_TX);



  delay(500);
  modem.sendAT("+IPR?");
  modem.waitResponse(1000);
} 

void return_uart() {
  SerialAT.end();
  delay(100);
  SerialAT.begin(921600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(100);

  SerialAT.write("AT+IPR=115200\r\n");
  SerialAT.flush();
  delay(1000);
 SerialAT.end();
  delay(100);
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(500);
  modem.sendAT("+IPR?");
  modem.waitResponse(1000);
}

/******************************bluetooth  function **********************************************/

bool initBluetooth() {
  if (!btStart()) {
    Serial.println("Failed to initialize controller");
    return false;
  }

  if (esp_bluedroid_init() != ESP_OK) {
    Serial.println("Failed to initialize bluedroid");
    return false;
  }

  if (esp_bluedroid_enable() != ESP_OK) {
    Serial.println("Failed to enable bluedroid");
    return false;
  }
  return true;
}

char* bda2str(const uint8_t* bda, char* str, size_t size) {
  if (bda == NULL || str == NULL || size < 18) {
    return NULL;
  }
  sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
          bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  return str;
}

void BTConfirmRequestCallback(uint32_t numVa) {

  confirmRequestPending = true;
  Serial.println(numVa);
}

void BTAuthCompleteCallback(boolean success) {
  confirmRequestPending = false;
  if (success) {
    Serial.println("Pairing success!!");
  } else {
    Serial.println("Pairing failed, rejected by user!!");
  }
}
/*
void GetBonbedBTDevices() {
  Serial.print("ESP32 bluetooth address: ");
  Serial.println(bda2str(esp_bt_dev_get_address(), bda_str, 18));
  int count = esp_bt_gap_get_bond_device_num();
  if (!count) {
    Serial.println("No bonded device found.");
  } else {
    Serial.print("Bonded device count: ");
    Serial.println(count);
    if (PAIR_MAX_DEVICES < count) {
      count = PAIR_MAX_DEVICES;
      Serial.print("Reset bonded device count: ");
      Serial.println(count);
    }
    esp_err_t tError = esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
    if (ESP_OK == tError) {
      for (int i = 0; i < count; i++) {
        Serial.print("Found bonded device # ");
        Serial.print(i);
        Serial.print(" -> ");
        Serial.println(bda2str(pairedDeviceBtAddr[i], bda_str, 18));
      }
    }
  }
}
*/
void RemoveBonbedBTDevices() {
  int count = esp_bt_gap_get_bond_device_num();
  esp_err_t tError = esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
  if (ESP_OK == tError) {
    for (int i = 0; i < count; i++) {

      if (REMOVE_BONDED_DEVICES) {
        esp_err_t tError = esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
        if (ESP_OK == tError) {
          Serial.print("Removed bonded device # ");
        } else {
          Serial.print("Failed to remove bonded device # ");
        }
        Serial.println(i);
      }
    }
  }
}
void Bluetooth_Setup(bool on_off) {
  if (on_off) {
    initBluetooth();
    // GetBonbedBTDevices();
    //RemoveBonbedBTDevices();
    SerialBT.enableSSP();
    SerialBT.onConfirmRequest(BTConfirmRequestCallback);
    SerialBT.onAuthComplete(BTAuthCompleteCallback);
    // Initialize admin and view-only accounts
    adminAccount.username = "admin";
    adminAccount.password = "123";
    adminAccount.userType = 0;
    //sys2_Req_061
    viewOnlyAccounts[0].username = "user1";
    viewOnlyAccounts[0].password = "123";
    viewOnlyAccounts[0].userType = 1;
    numViewOnlyAccounts++;

    viewOnlyAccounts[1].username = "user2";
    viewOnlyAccounts[1].password = "123";
    viewOnlyAccounts[1].userType = 1;
    numViewOnlyAccounts++;
    SerialBT.begin("ESP32test");  //Bluetooth device name
    // SerialBT.disconnect();
    Serial.println("The device started, now you can pair it with bluetooth!");
    // Bluetooth_pairing();
    // Bluetooth_Login();
  }
}
// Function to check the pairing requests
void Bluetooth_pairing() {
  if (confirmRequestPending) {
    if (Serial.available()) {
      int dat = Serial.read();
      if (dat == 'Y' || dat == 'y') {
        SerialBT.confirmReply(true);
        Serial.println("Bluetooth Connected ");
        isPairingRequested = true;
        //  vTaskDelay(1000 / portTICK_PERIOD_MS);

      } else {
        SerialBT.confirmReply(false);
      }
    }

    // continue;
  } else {
    if (Serial.available()) {
      SerialBT.write(Serial.read());
    }
    if (SerialBT.available()) {
      Serial.write(SerialBT.read());
    }
    delay(20);
  }
}
/*****************************Login functions*********************************/
// Function to Login as an admin or user and you can change the password of admin and the viewer also
void Bluetooth_Login() {
  if (SerialBT.available()) {

    
    //check if this is the first time for pairing?
    if (isPairingRequested) {
      String receivedData = SerialBT.readStringUntil('\n');  //receive the logged account username and password
      receivedData.trim();
      // if the pairing happens for the first time it gives the admin the ability to change the password
     // Serial.println("Received pairing password:" + receivedData);
      adminAccount.password = receivedData;
      isPairingRequested = false;
      Log_count = 1;

    } else if (!isLoggedIn) {
      String receivedData = SerialBT.readStringUntil('\n');  //receive the logged account username and password
      receivedData.trim();
      // during communication it will the the account(admin or viewer) then send the type to the app
      int delimiterIndex = receivedData.indexOf(':');  //amira:123
      //Serial.print(delimiterIndex);
      if (delimiterIndex != -1) {
        String username = receivedData.substring(0, delimiterIndex);
        String password = receivedData.substring(delimiterIndex + 1);
        if (isValidAccount(username, password)) {
          // Successful login
          Serial.println("Login successful");
          isLoggedIn = true;
          Log_count = 2;
          if (username == adminAccount.username) {
            SerialBT.write(adminAccount.userType);
            send_Type_Data(adminAccount.userType);
          } else {
            for (int i = 0; i < numViewOnlyAccounts; i++) {
              if (username == viewOnlyAccounts[i].username) {
                SerialBT.write(viewOnlyAccounts[i].userType);
                send_Type_Data(viewOnlyAccounts[i].userType);
              }
            }
          }
        } else {
          RemoveBonbedBTDevices();
        }
      }
    }
  }
}
// Function to send the type of the account to the app
void send_Type_Data(int num) {
  char buffer[128];
  snprintf(buffer, sizeof(buffer), "%d", num);
  SerialBT.println(buffer);
}

// Function to check the validity of a username and password
bool isValidAccount(const String& username1, const String& password1) {
  if (username1 == adminAccount.username && password1 == adminAccount.password) {
    isadmin = true;
    return true;
  }

  for (int i = 0; i < numViewOnlyAccounts; i++) {
    if (username1 == viewOnlyAccounts[i].username && password1 == viewOnlyAccounts[i].password) {
      return true;
    }
  }

  return false;
}

/*****************************mobile app functions*********************************/
/*void send_CAN_Data(int num, const char* SIM_NUM, int value1, int value2, int value3, int percentage, float float1, float float2, float float3, bool plugged) {
  char buffer[128];
  snprintf(buffer, sizeof(buffer), "%d %s %d %d %d %d%% %.1f %.1f %.1f %s",
           num, SIM_NUM, value1, value2, value3, percentage, float1, float2, float3, plugged ? "plugged" : "");

  SerialBT.println(buffer);
}
*/
void send_CAN_Data() {
  char buffer[128];
  Serial.println(isadmin);
  if (isadmin) {
    snprintf(buffer, sizeof(buffer), "%d %d %.2f %d %.2f %d %d %d %d", vehicle_id, soc_CC_Average, temp_av, pack_current_ma, value2, system_state, debug_state, min_v, max_v);
  } else {
    snprintf(buffer, sizeof(buffer), "%d %d %.2f %d %.2f", vehicle_id, soc_CC_Average, temp_av, pack_current_ma, value2);
  }
  SerialBT.println(buffer);
}
double stringToDouble(char* str) {
  return atof(str);
}

void Receive_GPS_Configuration() {

  if (isadmin) {
    int lng;
    int lat;
    int raduis;
    while (SerialBT.available() && count_var < 8) {

       lng = SerialBT.read();
      if (bufferIndexGPS < 8) {
        lng_buffer[bufferIndexGPS++] = lng;
      }
     
      count_var++;
    }
    if (count_var == 8) {
      char* str_ptr = &lng_buffer[0];
      initialLongitude = stringToDouble(str_ptr);
      // Serial.print("lng value = ");
      // Serial.println(initialLongitude, 5);
    }
    bufferIndexGPS = 0;
    if (count_var == 8) {
      while (SerialBT.available() && count_var < 17) {
        if (count_var == 8) {
          count_var++;
          int garbage_val = SerialBT.read();
        }
         lat = SerialBT.read();
        if (bufferIndexGPS < 8) {
          lat_buffer[bufferIndexGPS++] = lat;
        }
        Serial.write(lat);
        count_var++;
      }
      if (count_var == 17) {
        char* str_ptr = &lat_buffer[0];
        initialLatitude = stringToDouble(str_ptr);
        // Serial.print("lat value = ");
        // Serial.println(initialLatitude, 5);
      }
      bufferIndexGPS = 0;
    }
    if (count_var == 17) {
      while (SerialBT.available() && count_var <= 25) {
        if (count_var == 17) {
          count_var++;
          int garbage_val = SerialBT.read();
        }
         raduis = SerialBT.read();
        if (bufferIndexGPS < 5) {
          raduis_buffer[bufferIndexGPS++] = raduis;
        }
        Serial.write(raduis);
        count_var++;
      }
      if (count_var == 25) {
        char* str_ptr = &raduis_buffer[0];
        raduis_of_C1 = stringToDouble(str_ptr);
         Serial.print("raduis value = ");
         Serial.println(raduis_of_C1);
      }
      bufferIndexGPS = 0;
    }
    
  
  }
}




/******************************bluetooth tasks******************************************************/
void TaskBluetoothcode(void* parameters) {
  for (;;) {
    Bluetooth_pairing();
    if (Log_count == 0) {
      Bluetooth_Login();
    }
    if (isLoggedIn) {
        send_CAN_Data();
      if (!riceived_configuration) {
        Receive_GPS_Configuration();
      }

      float internal_distance = getDistance(latitude, longitude, initialLatitude, initialLongitude);
      Serial.print("distance = ");
      Serial.println(internal_distance);

      //delay(10000);
      count_var = 0;
    }
    vTaskDelay(1000);
    //delay(1000);
  }
}