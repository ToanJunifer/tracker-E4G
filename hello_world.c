//  LUCKY CHARM
/*
                       _oo0oo_
                      o8888888o
                      88" . "88
                      (| -_- |)
                      0\  =  /0
                    ___/`---'\___
                  .' \\|     |// '.
                 / \\|||  :  |||// \
                / _||||| -:- |||||- \
               |   | \\\  -  /// |   |
               | \_|  ''\---/''  |_/ |
               \  .-\__  '-'  ___/-. /
             ___'. .'  /--.--\  `. .'___
          ."" '<  `.___\_<|>_/___.' >' "".
         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
         \  \ `_.   \_ __\ /__ _/   .-` /  /
     =====`-.____`.___ \_____/___.-`___.-'=====
                       `=---='

             God bless:      NO ANY BUG
*/
//     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#ifndef INCLUDE
// Standard lib
#include "nwy_osi_api.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "nwy_gpio_open.h"
#include "Hardware.h"
#include "osi_log.h"
#include "InitSystem.h"
#include "math.h"
#include "stdbool.h"
#include "CommandHandler.h"
// Include for PPP
#include "DataDefine.h"
#include "nwy_socket.h"
#include "nwy_network.h"
#include "nwy_data.h"
// Include for GPS
#include "nwy_loc.h"
#include "nwy_vir_at.h"
// Include for SDCard
#include "nwy_file.h"
// Include for call, message
#include "nwy_voice.h"
#include "nwy_sms.h"
#include "nwy_adc.h"
// FOTA include 
#include "nwy_fota.h"
#include "nwy_fota_api.h"
#include "nwy_file.h"

#include "nwy_uart.h"
#include "nwy_sim.h"
#include "nwy_pm.h"

#include "nwy_i2c.h"
/*
Template NMEA message
$GPGGA,085946.000,2102.91090,N,10544.43369,E,1,05,3.8,35.8,M,-28.2,M,,*40
$GPGSV,3,1,09,02,,,11,12,22,221,29,13,60,141,33,15,45,200,38,0*59
$GPGSV,3,2,09,20,,,24,25,19,257,28,29,,,23,195,57,122,28,0*52
$GPGSV,3,3,09,199,,,28,0*57
$GPRMC,085945.000,A,2102.91067,N,10544.43370,E,0.00,0.00,280621,,,A,V*11
$GPZDA,085945.000,28,06,2021,00,00*5E
$GPGSA,A,3,12,13,15,25,195,,,,,,,,7.9,3.8,6.9,1*1A
$GPVTG,0.00,T,,M,0.00,N,0.00,K,A*3D
$GPGLL,2102.91090,N,10544.43369,E,085946.000,A,A*50
$GPTXT,01,01,01,ANTENNA OPEN*25
*/

/*
06/10/2021: TG102E-4G-V0.6
@author: ToanDao
+ Add power mode: alway on and power saving 
+ Low voltage alert and roll in sleep mode then
*/

/*
07/10/2021: TG102E-4G-V0.7
@author: ToanDao
+ Fix bug: send double sms response when read or write a command
+ Add update firmware time.
*/

/*
08/10/2021: TG102E-4G-V0.8
@author: ToanDao
+ Add cut off led after 5 min from acc is off.
*/

/*
11/10/2021: TG102E-4G-V0.9
@author: ToanDao
+ Add timer min and max, is the duration between update gps messages
*/

/*
12/10/2021: TG102E-4G-V0.9
@author: ToanDao
+ Add configuration for motion sensitivity
*/

/*
13/10/2021: TG102E-4G-V0.10
@author: ToanDao
+ Add alarm configure: alarm01, alarm02, alarm03 and alarm05
+ Add handle for Alarm01
*/

/*
14/10/2021: TG102E-4G-V0.11
@author: ToanDao
+ Add battery and charger mode
+ Add handle for Alarm02, alarm03 and alarm05
*/

/*
15/10/2021: TG102E-4G-V0.11
@author: ToanDao
+ Test watchdog and battery charger - but fail
*/

/*
22/10/2021: TG102E-4G-V0.12
@author: ToanDao
+ Add motion LIS3DSH
*/

/*
25/10/2021: TG102E[4G]-v1.0-013
@author: ToanDao
+ Add command URL to get current location
*/

/*
28/10/2021: TG102E[4G]-v1.0-013
@author: ToanDao
+ Modify host number configuration part.
+ Change all default number to 0123456789.
+ Add read working time and sleeping time.
*/

/*
29/10/2021: TG102E[4G]-v1.0-014
@author: ToanDao
+ Add MOTIONALARM command to set alarm when motion detected in manual mode
*/
char FIRMWARE_VERSION[]         = "TG102E[4G]-v1.0-014";

#define VnET_echo nwy_ext_echo
#define bool _Bool
#endif

// Section define
#define PPP                     1
#define GPS                     1
#define TMR                     1
#define SIM_Service             1
#define CFG                     1
#define FOTA                    1
#define iQueue                  1
#define iUART                   1
#define Peripheral              1
#define I2C                     1
// API Timer
#if TMR
// define section
// variable section
static uint32_t tickCount100MS = 0;
nwy_osiTimer_t  *g_timer = NULL;
nwy_osiThread_t *g_app_thread = NULL;
nwy_osiThread_t *tcp_recv_thread = NULL;
nwy_osiThread_t *fota_app_thread = NULL;
nwy_osiThread_t *tcp_recv_fota_thread = NULL;
nwy_osiThread_t *cfg_app_thread = NULL;
//API section
void nwy_app_timer_cb(void);
#endif

// API Peripheral
#if Peripheral
/*
0   - placeholder for unknown reason
1   - boot by power key
2   - boot by pin reset
4   - boot by alarm
8   - boot by charge in
16  - boot by watchdog
32  - boot by wakeup
64  - boot from PSM wakeup
*/
// define section
#define LED_GPS_E           28
#define LED_GSM_E           27
#define OUTPUT2             26
#define OUTPUT1             25
#define ACC                 24
#define CHARGER             21
#define PSM_WAKEUP          10
#define PARKING             0
#define RUNNING             1
#define CHARGING_TIME       900000  // 15 mins
#define NO_CHARGING_TIME    180000  // 3 mins
// variable section
static char * Battery_voltage;
int led_gps_state = 0;
int led_gsm_state = 0;
int old_led_gps_state = 0;
int old_led_gsm_state = 0;
int counter_led_gsm = 0;
int counter_led_gps = 0;
int light_on_ms = 0;
int light_off_ms= 0;
static int accValue;
static int sleepMode = -1;
static int cutOffLED = 0;
static int LEDHandle = -1;
static int exceptionReboot = 0;
static char * PowerSaving;
static char * voltageThres;
int sendAlertLowPower = 0;
int64 timeAccOn;
int informToOwnerAlarm01 = 0;
int chargerState = 0;
float THRESH_TO_CHARGER = 12.6;
int64 chargerMoment = 0;
int64 lastChargeMoment = 0;
// API section
int init_4led();
void led_gsm_control(int old_state, int pres_state);
void led_gps_control(int old_state, int pres_state);
void read_Battery(char ** Voltage_input);
static void read_ACC(int param);
void check_reboot_time();
int deinit_led();
int init_charger_pin();
int enable_charger();
int disable_charger();
// Stack store power voltage
#define STACKSIZE       5
int idxTop = 0;
float stackVoltage[STACKSIZE];
void push(float voltage);
float average_voltage();
#endif

// API PPP
#if PPP
//====== Define section
#define NWY_UART_RECV_SINGLE_MAX   560
typedef enum{
  NWY_CUSTOM_IP_TYPE_OR_DNS_NONE = -1,
  NWY_CUSTOM_IP_TYPE_OR_DNS_IPV4 = 0,
  NWY_CUSTOM_IP_TYPE_OR_DNS_IPV6 = 1,
  NWY_CUSTOM_IP_TYPE_OR_DNS_DNS = 2
}nwy_ip_type_or_dns_enum;
// ====== Variable section
static int hndl_ppp = 0;
static int tcp_connect_flag = 0;
int ret = 0;
int rssi = 0;
int send_len = 0;
int call_PPP = 0;
int init_TCP = 0;
static int ppp_state[10] = {0};
int sentLoginFlag = -1;
static int sock = 0;
int numRetryCallPPP = 0;
static profile_index = 0;
int PPPConnection = 0;
int PPPLostConnectCounter = 0;
int recv_login_package = 0;
int waiting_login_gps = 0;
// ====== API section
int vnet_tcp_tracking_init(char * url_or_ip, int port);
int nwy_ext_check_data_connect();
static void nwy_data_cb_fun(int hndl,nwy_data_call_state_t ind_state);
static void nwy_tcp_recv_tracking(void *param);
nwy_ip_type_or_dns_enum nwy_judge_ip_or_dns(char *str);
static int nwy_hostname_check(char *hostname, char *ip_buf);
int vnet_call_ppp(int waitingFlag);
int sendMessage(void *socketID, char * hexString);
int stop_data_call(void * handler);
#endif

//  API FOTA
#if FOTA
#define NWY_EXT_APPIMG_FOTA_DATA_REC_END	(NWY_APP_EVENT_ID_BASE + 3)
#define NWY_APPIMG_FOTA_BLOCK_SIZE          (2*1024)
#define FOTA_UPDATE         	            NWY_APP_EVENT_ID_BASE + 7
#define TIMEOUT_RECV_LOGIN_FOTA             5
#define FOTA_LOGIN_CYCLE                    3600
#define MIN5SEC20                           320
#define MIN5SEC50                           350     
#define LEN_LOGIN_FOTA_MSG                  66   
#define LEN_PACKET_FOTA_MSG                 22
unsigned int fota_global_counter = 0;
int recv_fota_login_package = 0;
int init_fota_TCP = 0;
static int tcp_connect_fota_flag = 0;
uint8_t first_byte_hex;
static int fota_sock = 0;
char fota_SOF[]="2424";
char fota_Login_DataType[]="01FF";
char fota_Code[]="1D00";
char fota_Login_Length[]="3000";
char fota_DateTime[13];
// char fota_ID[]="8618810507619320";
char * fota_FirmwareVersion;
char fota_missing22Byte[]="0000000000000000000000";
char fota_Reserved[]="0000000000000000";
char fota_Firmware_DataType[]="03FF";
char fota_SerialNumber[]="0000";
char fota_Login_Checksum[5];
char fota_Firmware_Checksum[5];
char fota_Packet_Checksum[5];
char fota_EOF[] = "2323";
char fota_Firmware_Length[]="0200";
char fota_Packet_ID[]="0000";
char fota_Packet_Length[]="0400";
char fota_Packet_DataType[]="05FF";
char fota_LoginPackage[133];
char fota_FirmwarePackage[41];
char fota_PacketPackage[45];
char fota_timezone =7;
nwy_time_t fota_vnet_time = {0};
nwy_time_t vnet_time = {0};
char * fota_LoginMessage;
char * fota_FirmwareMessage;
char * fota_PacketMessage;
int FOTA_login_flag = 0;
char Header_RecvMessage[5];
char DataType_RecvMessage[5];
char PackageSize_RecvMessage[9];
char NumPackage_RecvMessage[5];
long fota_package_size = 0;
int fota_number_package = 0;
int fota_id_package_req = 0;
char fota_packageID[5];
int fota_id_package_recv = 0;
char fota_data_bin_file[1024];
uint32_t fota_bytes_recv = 0;
uint8_t fota_store_firm[204800];
long fota_pos_firm = 0;
int init_fota_moment = 0;
int time_retry_fota = 0;
int waiting_login_fota = 0;
int vnet_tcp_fota_init(char * url_or_ip, int port);
static void vnet_tcp_recv_fota(void *param);
int fota_sendMessage(void *socketID, char * hexString);
char *Read_FOTA_Firmware_Version();
char *Get_FOTA_Login_Message();
char *Get_FOTA_Firmware_Message();
char *Get_FOTA_Package_Message(int id);
char * hexstr_to_char(char* hexstr);
void nwy_appimg_fota();
#endif

// API GPS
#if GPS
// ====== Define section
struct mesg_GPRMC { 
    int Time; 
    float Latitude;
    float Longitude;
    float oldLatitude;
    float oldLongitude;
    char Flag[2];
    int Date; 
    int Speed;
    int oldSpeed;
    int HalfCourse;
    int bit_1;
    int bit_2;
    int bit_3;
} GPRMC; 

typedef struct{
    float prev_Lat;
    float prev_Lng;
    float prev_HoC;
} prev_gps_data;

#define LOC_EQUR               ((float)6378137)
#define LOC_POLR               ((float)6356752.314245)
#define LOC_INVF               ((float)0.0033528106647475)
#define LOC_PIVAL              3.14159265358979323846
#define LOC_PI                 ((float)LOC_PIVAL)
#define LOC_PID                ((double)LOC_PIVAL)
#define LOC_DEGTORAD(VDEG)     ((VDEG) * LOC_PI / ((float)180))
#define LOC_ABSSUB(FNA, FNB) (((FNA)>(FNB))? ((FNA)-(FNB)):((FNB)-(FNA)))
#define LEN_TRACKING_MSG        36
#define VOLTAGE_BIAS            360 // mV
        
// ====== Variable section
int INIT_GPS = 0;
int optionCase = 1;
char package_GPS[73] = {0};
int NoOfSatellite = 0;
char mLenOfContent[5], mDate[7], mTime[7], mLatitude[9], mLongitude[9], mLocationInfo[3], mNoOfSatellite[3], mSpeed[3], mDistance[9], mHalfOfCourse[3], mSerialNo[5],  mChecksum[5];
char mSOF[] = "2424";
char mEOF[] = "2323";
char mDataType[] = "3300";
char mReserved[] = "00";
float fDistance;
uint16_t biasX, biasY;
float currentAngular;
float oldAngular;
uint16_t NSendPackageSuccess = 0;
char package_Heartbeat[79];
char hSOF[] = "2424";
char hDataType[] = "0000";
char hLenOfContent[5], hDate[7], hTime[7], hFirewareVersion[17], hCode[5], hSerialNo[5], hChecksum[5];
char hDeviceFamily[] = "0f"; 
char hEOF[] = "2323";
char hStatus[] = "01";
char package_Realtime[61];
char rSOF[] = "2424";
char rDataType[] = "3400";
char rLenOfContent[] = "0c00";
char rBattery[] = "00";
char rMainPower[] = "00";
char rNumberDoorOpened[] = "0100";
char rNumberOverSpeed[] = "0000";
char rDate[7], rTime[7], rStaticState[13], rSerialNumber[5], rCheckSum[5];
char rEOF[] = "2323";
char rStaticState_byte1[]="01";
char rStaticState_byte2[]="00";
char rStaticState_byte3[]="00";
char rStaticState_byte4[]="EF";
char rStaticState_byte5[3];
char rStaticState_byte6[]="00";
char * GPSPackage;
char * LoginPackage;
char * Realtime_Message;
int VehicleSpeedMAX = 0;
static const uint16_t crctab16[] = {
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
static float ValL;
static float xxA;
static float xxB;
static float xxC;
static float sinU1, sinU2, cosU1, cosU2;
static float cosSqAlpha;
static float sinSigma;
static float cos2SigmaM;
static float cosSigma;
static float sigma;
static float sinLambda, cosLambda, sinAlpha, TmpC;
prev_gps_data prev_data_gps;
float distanceGPS = 0.0;
float delta_GPS = 0.0;
int prev_rssi = 0;
int prev_satellite = 0;
int last_time_sent_status = 0;
int send_loop = 0;
char nmea_data[2048] = {0};
char linedata[32][128];
unsigned int global_counter_1s = 0;
unsigned int acc_off_counter = 0;
int Tmin = 5;
int Tmax = 30;
// ====== API section
int VnET_init_gps();
uint16_t CalCRC16Hex(unsigned char  * Data);
uint16_t GetCrc16(const uint8_t* pData, uint32_t nLength);
uint32_t swap32bit(uint32_t number);
uint16_t swap16bit(int number);
char * Get_Heartbeat_Message();
char * Get_GPS_Message();
char * Get_Realtime_Message();
float Loc_DeflectionAngle(float anga, float angb);
float  Loc_GetDistance(float latA, float lonA, float latB, float lonB);
prev_gps_data get_prev_gps_data(char * GPS_Pkg);
int deinit_GPS();
#endif

// API Queue
#if iQueue
#define QUEUESIZE 256
#define PACKAGESIZE 78
struct Queue {
	int front, rear, size;
	int capacity;
	char array[QUEUESIZE][PACKAGESIZE];
};
char * getPackageFromQueue;
struct Queue* queue;
struct Queue* createQueue(int capacity);
int isFull(struct Queue* queue);
char * front(struct Queue* queue);
int isEmpty(struct Queue* queue);
int queueValid();
void enqueue(struct Queue* queue, char * item);
void checkOverFlowQueue(struct Queue* queue);
void simpleDequeue(struct Queue* queue);
#endif

#if CFG
#define CONFIGS_SIZE                512
#define CONFIGS_TYPE_POS            0       // 1    +1
#define CONFIGS_IP_GPS_POS          2       // 15   +1
#define CONFIGS_PORT_GPS_POS        18      // 5    +1
#define CONFIGS_IP_FOTA_POS         24      // 15   +1
#define CONFIGS_PORT_FOTA_POS       40      // 5    +1
#define CONFIGS_HOST_NUMBER_POS     46      // 12   +1
#define CONFIGS_TIMEZONE_POS        59      // 4    +1
#define CONFIGS_NTP_POS             64      // 15   +1
#define CONFIGS_APN_POS             80      // 4    +1
#define CONFIGS_IMEI_POS            85      // 16   +1
#define CONFIGS_REBOOT_POS          102     // 4    +1
#define CONFIGS_POWER_MODE          107     // 3    +1
#define CONFIGS_VOLTAGE_THRES       111     // 5    +1
#define CONFIGS_TIMER_MIN           117     // 5    +1
#define CONFIGS_TIMER_MAX           123     // 5    +1
#define CONFIGS_MOTION              129     // 10   +1
#define CONFIGS_ALARM_01            140     // 26   +1
#define CONFIGS_ALARM_02            167     // 26   +1
#define CONFIGS_ALARM_03            194     // 26   +1
#define CONFIGS_ALARM_05            221     // 26   +1

#define URL_CFG2URL                 0
#define URL_URL2CFG                 1

char path_cfgs_file[] = "configs.txt";
enum CMD_SRC{
    SRC_NONE,
    SRC_UART,
    SRC_SMS,
    SRC_FOTA
};
typedef struct{
    int uart_connection;
    void (*uart_send)(uint8_t hd, uint8_t *data_ptr, uint32_t length);
    int (*uart_init)(uint32_t name, nwy_uart_mode_t mode);
}UART;
UART UART_CFG;

char config_type[2];
static char * tracking_url_cfg;
static char tracking_port_cfg[6];
static char * fota_url_cfg;
static char fota_port_cfg[6];
char host_number[13];
char sender_number[13];
char timezone[4];
char ntpserver[16];
char APN[5];
char APN_name[11];
char APN_user[5];
char APN_pass[5];
char * IMEI_NUM;
static char reboot_time[5];
static char hour_reboot[3];
static char minute_reboot[3];
static char updateFirmwareTime[32];
typedef struct{
    char relayChannel[2];
    char ctrlType[2];
    char operationMode[2];
    char informType[2];
    char phoneNumber[12];
}alarm;
alarm alarm01;
alarm alarm02;
alarm alarm03;
alarm alarm05;
int64 uptime_ms;
int64 prevTime      = 0;
int sleepingTimeM   = 0;
int workingTimeM    = 0;
void getImei(char ** IMEI_);
int isNumber(const char *str);
void check_configs_file();
char * read_configs();
void url_transform(char * url, int mode, char ** url_output);
void update_configs();
int load_default_cfg2flash();
int delete_configs();
int findSpecialChar(unsigned char *buff, int strLength, char c);
int findTotalSpecialChar(unsigned char *buff, unsigned char leng, char c);
int Response_Command(char * header, int src, char * content_type);
int GetaLinkOfGoogleMap(char **ContentPtr);
int APN_update(char apn[]);
int Center_Handle_Command(unsigned char *cmd, int len, int src );
int checkAlarmMessage(char * ptr1, char * ptr2, char * ptr3, char * ptr4, char * ptr5);
#endif
 
// API UART
#if iUART
void uart_recv_handle (unsigned char *str,uint32_t length);
void uart_main(void);
int deinit_uart(void * hdlr);
#endif

// API SIM_Service
#if SIM_Service
void checkSIMSlot();
// SMS
nwy_sim_status sim_status;
int ret_report_mode = -1;
int set_report_mode_sms();
int vnet_send_sms(char phoneNumber[], char message[]);
void nwy_recv_sms();

// CALL
static int call_alert = 0;
int counter_call_alert = 0;
static void nwy_voice_ind();
int vnet_voice_call(char phoneNumber[]);
void nwy_test_cli_set_sim_slot();
int  nwy_test_cli_get_sim_slot();
#endif

// API I2C
#if I2C
// Register LIS3DSH
#define LIS3DSH_DEV_ADDR            0x1E    // (0x68 >> 1 = 0x34)
#define LIS3DSH_WHO_AM_I            0x0F
#define LIS3DSH_WHO_AM_I_VALUE      0x3F
#define LIS3DSH_CTRL_REG4_ADDR      0x20
#define LIS3DSH_CTRL_REG5_ADDR      0x24
#define LIS3DSH_STATUS_ADDR         0x27
#define LIS3DSH_OUT_X_L      	    0x28
#define LIS3DSH_OUT_X_H      	    0x29
#define LIS3DSH_OUT_Y_L      	    0x2A
#define LIS3DSH_OUT_Y_H      	    0x2B
#define LIS3DSH_OUT_Z_L      	    0x2C
#define LIS3DSH_OUT_Z_H      	    0x2D

// Register lis3dh
#define LIS3DH_DEV_ADDR             0x18    // (0x68 >> 1 = 0x34)
#define LIS3DH_WHO_AM_I             0x0F
#define LIS3DH_WHO_AM_I_VALUE       0x33
#define LIS3DH_CTRL_REG1_ADDR       0x20
#define LIS3DH_CTRL_REG4_ADDR       0x23
#define LIS3DH_CTRL_REG1_ADDR       0x20
#define LIS3DH_CTRL_REG4_ADDR       0x23
#define LIS3DH_OUT_X_L      	    0x28
#define LIS3DH_OUT_X_H      	    0x29
#define LIS3DH_OUT_Y_L      	    0x2A
#define LIS3DH_OUT_Y_H      	    0x2B
#define LIS3DH_OUT_Z_L      	    0x2C
#define LIS3DH_OUT_Z_H      	    0x2D

typedef struct{
    float biasAccelX;
    float biasAccelY;
    float biasAccelZ;
    float prevAccelX;
    float prevAccelY;
    float prevAccelZ;
    float AccelX;
    float AccelY;
    float AccelZ;
    int motionDetect;
} motionAxisValue;
motionAxisValue lis3dh;
motionAxisValue lis3dsh;
uint8_t ACC_X_L = 0;
uint8_t ACC_X_H = 0;
uint8_t ACC_Y_L = 0;
uint8_t ACC_Y_H = 0;
uint8_t ACC_Z_L = 0;
uint8_t ACC_Z_H = 0;
static int      motionAlarm         = 0;
int             motionActive        = 1;
int             motionAlert         = 0;
static int      i2c_init            = -1;
static float    sensitivity;
float           motionSensitive     = 0.125;
int             alertMotionTime     = 150; // 0.2 second
int             counterAlertMotion  = 0;
int             i2cDeviceAvailable  = 0;
void            configs_LIS3DSH();
int             motion_LIS3DSH();
void            configs_LIS3DH();
int             motion_LIS3DH();
int             check_i2c_device();
typedef void    configs_i2c_device_def();
typedef int     motion_detect_def();
configs_i2c_device_def  * configs_i2c_device;
motion_detect_def       * motion_detect;
static bool     nwy_i2c_set_addr(int bus, uint8_t dev, uint16_t reg, bool read);
static bool     nwy_i2c_read_dev_reg(int bus, uint8_t dev, uint16_t reg, uint8_t *buf, uint32_t length);
static bool     nwy_i2c_write_dev_reg(int bus, uint8_t dev, uint16_t reg, uint8_t *buf, uint32_t length);
#endif
//END OF API

static void CfgThreadEntry(){
    nwy_sleep(5000);
    unsigned int cfg_global_counter = 0;
    nwy_sdk_at_parameter_init();
    nwy_sdk_at_unsolicited_cb_reg("+CMT", nwy_recv_sms);
    nwy_sdk_at_unsolicited_cb_reg("+ATD", nwy_voice_ind);
    nwy_init_sms_option();
    nwy_osiEvent_t cfg_event;
    VnET_echo("\r\nFirmware version: %s",FIRMWARE_VERSION);
    while(1){
        // INIT SMS mode
        cfg_global_counter++;
        if ((cfg_global_counter>=5)&&(ret_report_mode==-1)){
            ret_report_mode = set_report_mode_sms();
            (ret_report_mode==0) ? VnET_echo("\r\n Set report SMS successfully") : VnET_echo("\r\n Set report SMS failed. Please check again");
            char content[42];
            sprintf(content,"Boot up |  Cause: %d | Firmware version: %s",nwy_get_boot_causes(),FIRMWARE_VERSION);
            vnet_send_sms((char *)host_number,(char *)content);
        }

        if(!ret_report_mode){
            nwy_wait_thead_event(cfg_app_thread, &cfg_event, 0);
        }
        nwy_sleep(2000);
    }
}

static void FotaThreadEntry(){
    nwy_sleep(5000);
    nwy_osiEvent_t fota_event; 
    int fota_login_time = 0;
    while(1){
        // FOTA
        nwy_wait_thead_event(fota_app_thread, &fota_event, 0);
        switch(fota_event.id){
            case MAIN_TICK_10MS:
                fota_global_counter++;
                // Quá trình login diễn ra từ 5p20 đến 5p50
                if ((fota_global_counter<=MIN5SEC50)&&(fota_global_counter>=MIN5SEC20)&&((accValue == nwy_low)||(!strncmp(strupr(PowerSaving),"PS0",strlen(PowerSaving))))){
                    // Có 4G + chưa kết nối socket + số lần thử kết nối nhỏ hơn hoặc bằng 3
                    if ((!init_fota_TCP)&&(call_PPP)&&(time_retry_fota<3)){
                        time_retry_fota++;
                        init_fota_TCP = vnet_tcp_fota_init(fota_url_cfg,atoi(fota_port_cfg));
                        if (init_fota_TCP){
                            init_fota_moment = fota_global_counter;
                            VnET_echo("\r\n Init FOTA connection successfully");
                            time_retry_fota = 0;
                        }
                        else{
                            VnET_echo("\r\n Init FOTA connection failed");
                        }
                    }
                    // Socket đã phản hồi + chưa gửi gói login
                    if ((FOTA_login_flag==0)&&(init_fota_TCP==1)){
                        // kể từ giây thứ 3, sau khi init socket --> gửi login
                        if ((fota_global_counter-init_fota_moment)>=3){
                            fota_LoginMessage = Get_FOTA_Login_Message();
                            VnET_echo("\r\nFOTALoginMessage: %s\n",fota_LoginMessage);
                            send_len = fota_sendMessage((void*)&fota_sock,fota_LoginMessage);
                            if (send_len==LEN_LOGIN_FOTA_MSG){
                                VnET_echo("\r\nSent login FOTA message successfully\n");
                                FOTA_login_flag = 1;
                            }
                            else{
                                VnET_echo("\r\nSent login FOTA failed.\n");
                            }
                            free(fota_LoginMessage);
                        }
                    }
                    // Gửi login mà sau 30s ko nhận đc phản hồi --> reset cờ login
                    if ((recv_fota_login_package==0)&&(FOTA_login_flag==1)&&(fota_login_time <= 3)){
                        waiting_login_fota++;
                        VnET_echo("\r\n waiting_login_fota: %d",waiting_login_fota);
                        if (waiting_login_fota>=30){
                            FOTA_login_flag=0;
                            waiting_login_fota = 0;
                            fota_login_time ++;
                            VnET_echo("\r\n FOTA server not response. Resending...");
                        }
                    }
                }
                // 10s trước khi hết chu kì 60 phút --> reset all
                if (fota_global_counter >= (FOTA_LOGIN_CYCLE-5)){
                    fota_login_time = 0;
                    init_fota_TCP = 0;
                    FOTA_login_flag = 0;
                    fota_global_counter = 0;
                    init_fota_moment  = 0;
                    recv_fota_login_package = 0;
                    waiting_login_fota = 0;
                    time_retry_fota = 0;
                }
                break;
            case FOTA_UPDATE:
                if (fota_id_package_req < fota_number_package){
                    fota_PacketMessage = Get_FOTA_Package_Message(fota_id_package_req);
                    send_len = fota_sendMessage((void*)&fota_sock,fota_PacketMessage);
                    if (send_len==LEN_PACKET_FOTA_MSG){
                        VnET_echo("\r\nSent package FOTA message successfully\n");
                        fota_id_package_req++;
                    }
                    else{
                        VnET_echo("\r\nSent package FOTA failed. Retrying...\n");
                        fota_id_package_req = 0;
                    }
                    free(fota_PacketMessage);
                }
                
                break;
            case NWY_EXT_APPIMG_FOTA_DATA_REC_END:
                nwy_appimg_fota();
                break;
            default:
                break;
        }
    }
}

static void prvThreadEntry(void *param){
    nwy_sleep(2000);
    VnET_echo("\r\n==> ==> ==> START SYSTEM ==> ==> ==>");
    nwy_osiEvent_t event; 
    checkSIMSlot();
    nwy_subpower_switch(NWY_POWER_SD , true, false);
    nwy_set_pmu_power_level(NWY_POWER_SD,3000);
    check_configs_file();
    nwy_sleep(500);
    // delete_configs();
    update_configs();
    // uart_main();
    LEDHandle = init_4led();
    // Enable interrupt on ACC pin
    nwy_open_gpio_irq_config(ACC,nwy_irq_rising_falling,read_ACC);
    nwy_sleep(100);
    read_Battery(&Battery_voltage);
    sprintf(rMainPower,"%02X",(int)(round(atof(Battery_voltage))));
    VnET_echo("\r\nMainPower: %s",rMainPower);
    INIT_GPS = VnET_init_gps();
    (INIT_GPS) ? VnET_echo("\r\nInit GPS complete") : VnET_echo("\r\nInit GPS fail");
    sim_status = nwy_sim_get_card_status();
    if (sim_status == NWY_SIM_STATUS_READY){
        VnET_echo("\r\nSIMCard is ready");
        call_PPP = vnet_call_ppp(1);
        (call_PPP) ? VnET_echo("\r\nInit PPP complete") : VnET_echo("\r\nInit PPP fail");
    }
    
    queue = createQueue(QUEUESIZE);
    (init_charger_pin()) ? VnET_echo("\r\n Initialize charger pin success") : VnET_echo("\r\n Initialize charger pin fail");

    // Init i2c
    i2c_init = nwy_i2c_init(NAME_I2C_BUS_2,NWY_I2C_BPS_100K);
    (i2c_init == NWY_ERROR) ? VnET_echo("\r\n I2c init Error:   %d",i2c_init) : VnET_echo("\r\n I2c init Success: %d",i2c_init);
    // Setup for i2c
    i2cDeviceAvailable = check_i2c_device();
    if ((i2cDeviceAvailable)&&(motionActive)){
        configs_i2c_device();
    }
    // Set the first value of motion alarm which is used on manual mode in ALARM01.
    (accValue) ? (motionAlarm=1) : (motionAlarm=0);
    while (1){
        nwy_wait_thead_event(g_app_thread, &event, 0);
        switch(event.id){
            case MAIN_TICK_1000MS:
                global_counter_1s++;
                if ((i2cDeviceAvailable)&&(motionActive)&&(accValue == nwy_high)){
                    if (motion_detect()==1){    // Detect motion in sensor
                        if((alarm01.operationMode==1) || ((alarm01.operationMode==0) && (motionAlarm ==1))){    // Active on automatic mode and manual mode also. In manual mode, motionAlarm must be on
                            counterAlertMotion = alertMotionTime;
                            VnET_echo("\r\n[MOTION]Start moving---------------->");
                        }
                    }
                }
                checkOverFlowQueue(queue);
                // PPP
                if ((call_PPP <= 0)&&((accValue == nwy_low)||(!strncmp(strupr(PowerSaving),"PS0",strlen(PowerSaving))))){
                    PPPLostConnectCounter ++;
                    if (PPPLostConnectCounter >= 10){
                        sim_status = nwy_sim_get_card_status();
                        VnET_echo("\r\nPPP connection is not established. SIM status: %d\n",sim_status);
                        if (sim_status == NWY_SIM_STATUS_READY){
                            call_PPP = vnet_call_ppp(1);
                            if (call_PPP){
                                VnET_echo("\r\nInit PPP complete");
                            }
                            else{
                                numRetryCallPPP ++;
                                VnET_echo("\r\nInit PPP fail");
                                if (numRetryCallPPP >= 6){
                                    VnET_echo("\r\nRebooting...Due to can't connect PPP");
                                    nwy_power_off(2);
                                }
                            } 
                        }
                        else if (sim_status == NWY_SIM_STATUS_GET_ERROR){
                            VnET_echo("\r\nRebooting. Due to SIM card get error");
                            nwy_power_off(2);
                        }
                        PPPLostConnectCounter = 0;
                        sentLoginFlag = -1;
                    }
                }
                // if (chargerState == 0){
                //     if (average_voltage() >= THRESH_TO_CHARGER){
                //         if ((lastChargeMoment==0) || ((nwy_get_ms()-lastChargeMoment) >= NO_CHARGING_TIME)){
                //             int ret = enable_charger();
                //             if (ret){
                //                 chargerState = 1;
                //                 VnET_echo("\r\n Enable charger mode");
                //                 chargerMoment = nwy_get_ms();
                //             }
                //         }
                //     }   
                // }
                // else if (chargerState == 1){
                //     if ((average_voltage() < THRESH_TO_CHARGER) || ((nwy_get_ms()-chargerMoment) >= CHARGING_TIME)){
                //         int ret = disable_charger();
                //         if (ret){
                //             chargerState = 0;
                //             VnET_echo("\r\n Disable charger mode");
                //             lastChargeMoment = nwy_get_ms();
                //         }
                //     }
                // }
                 
                if ((global_counter_1s % 45) == 0){     // each 45 sec
                    check_reboot_time();
                    read_Battery(&Battery_voltage);
                    // VnET_echo("\r\nCurrent Voltage: %s",Battery_voltage);
                    push(atof(Battery_voltage));
                    // VnET_echo("\r\nAverage Voltage: %02.02f",average_voltage());

                    if ((sleepMode!=0)&&(cutOffLED!=1)){
                        uptime_ms = nwy_get_ms();
                        if (uptime_ms-timeAccOn > (long)300000){    // after 5 mins - cut off led
                            cutOffLED = 1;
                            if (deinit_led()){
                                LEDHandle = -1;
                            }
                        }
                    }
                    if (average_voltage() < atof(voltageThres)){
                        if (!sendAlertLowPower){
                            char content[42];
                            sprintf(content,"[ALERT] Battery voltage(%s) is lower than %s",Battery_voltage,voltageThres);
                            vnet_send_sms((char *)host_number,(char *)content);
                            sendAlertLowPower = 1;
                        }
                    }
                    if (((accValue == nwy_high)||(atof(Battery_voltage) < atof(voltageThres)))&&(sleepMode != 0)&&(!strncmp(strupr(PowerSaving),"PS1",strlen(PowerSaving)))){
                        acc_off_counter++;
                        if (acc_off_counter >= 6){ // after 4.5 mins - close GPS and GSM
                            acc_off_counter = 0;
                            int deinit_gps = deinit_GPS();
                            int deinit_gsm = stop_data_call((void*)&hndl_ppp);
                            int deinitLED = -1;
                            if (cutOffLED==1){
                                deinitLED = 1;
                            }
                            else{
                                deinitLED = deinit_led();
                            }
                            if ((deinit_gps==1)&&(deinit_gsm==1)&&(deinitLED==1)){
                                int retSetMode = nwy_pm_state_set(NWY_ENTRY_SLEEP);
                                if (retSetMode==0){
                                    VnET_echo("\r\n Rolled in sleep mode");
                                    sleepMode = 0;
                                    INIT_GPS = 0;
                                    call_PPP = 0;
                                    init_TCP = 0;
                                    strcpy(GPRMC.Flag,"V");
                                    sentLoginFlag = 1;
                                    recv_login_package=0;
                                    LEDHandle = -1;
                                    uptime_ms = nwy_get_ms();
                                    workingTimeM = workingTimeM + (uptime_ms-prevTime)/60000;
                                    vnet_send_sms((char *)host_number,"Device rolled in sleep mode");
                                    prevTime = uptime_ms;
                                } 
                                else{
                                    VnET_echo("\r\n Power set failed");
                                }
                            }
                            else{
                                VnET_echo("\r\n: [EXCEPTION] GPS: %d | GSM: %d | LED: %d",deinit_gps,deinit_gsm,deinitLED);
                                char reportContent[32];
                                sprintf(reportContent,"GPS:%d | GSM:%d | LED:%d",deinit_gps,deinit_gsm,deinitLED);
                                vnet_send_sms((char *)host_number,reportContent);

                                int retSetMode = nwy_pm_state_set(NWY_ENTRY_SLEEP);
                                if (retSetMode==0){
                                    VnET_echo("\r\n [EXCEPTION] Rolled in sleep mode");
                                    sleepMode = 0;
                                    INIT_GPS = 0;
                                    call_PPP = 0;
                                    init_TCP = 0;
                                    strcpy(GPRMC.Flag,"V");
                                    sentLoginFlag = 1;
                                    recv_login_package=0;
                                    LEDHandle = -1;
                                    uptime_ms = nwy_get_ms();
                                    workingTimeM = workingTimeM + (uptime_ms-prevTime)/60000;
                                    vnet_send_sms((char *)host_number,"[EXCEPTION] Device rolled in sleep mode");
                                    prevTime = uptime_ms;
                                }
                                exceptionReboot = 1;
                            }
                        }
                    }
                    else if (accValue == nwy_low){ 
                        acc_off_counter = 0;
                    }
                }
                
                if ((INIT_GPS)&&(sleepMode != 0)){
                    int result = nwy_loc_get_nmea_data(nmea_data);
                    if (!result) {
                        VnET_echo("\r\n get nmea data fail");
                        INIT_GPS = 0;
                    } 
                    else {
                        char * token = strtok(nmea_data, "\n");
                        int c=0; 
                        while( token != NULL ) {
                            strcpy(linedata[c],token);
                            token = strtok(NULL, "\n");
                            c += 1;
                        }
                        for (int i = 0; i < c ; i ++){
                            char * part = strtok(linedata[i], ",");
                            char data[32][32];
                            int u = 0;
                            while( part != NULL ) {
                                strcpy(data[u],part);
                                part = strtok(NULL, ",");
                                u += 1;
                            }
                            if ((!strncmp(data[0] , "$GPGGA",6)) || (!strncmp(data[0] , "$GNGGA",6))){
                                NoOfSatellite = atoi(data[7]);
                            }
                            
                            if ((!strncmp(data[0] , "$GPRMC",6)) || (!strncmp(data[0] , "$GNRMC",6))){
                                strcpy(GPRMC.Flag,data[2]);
                                if (strncmp(GPRMC.Flag,"A",1)==0){
                                    GPRMC.Time = atoi(data[1]);
                                    GPRMC.Date = atoi(data[9]);
                                    GPRMC.Latitude = atof(data[3]);
                                    GPRMC.Longitude = atof(data[5]);
                                    GPRMC.Speed = (int)(atof(data[7])*1.852);
                                    GPRMC.HalfCourse = (int)(atof(data[8])/2);
                                    (strncmp(data[2],"A",1) == 0) ? (GPRMC.bit_3 = 1) : (GPRMC.bit_3 = 0);
                                    (strncmp(data[4],"N",1) == 0) ? (GPRMC.bit_1 = 1) : (GPRMC.bit_1 = 0);
                                    (strncmp(data[6],"E",1) == 0) ? (GPRMC.bit_2 = 1) : (GPRMC.bit_2 = 0);
                                }
                            }
                        }
                    }
                    if (strncmp(GPRMC.Flag,"A",1)==0){
                        // Init tracking server
                        if ((call_PPP)&&(!init_TCP)){
                            init_TCP = vnet_tcp_tracking_init(tracking_url_cfg,atoi(tracking_port_cfg));
                        }
                        // Send login message
                        if ((sentLoginFlag == -1)&&(init_TCP==1)){
                            LoginPackage = Get_Heartbeat_Message();
                            VnET_echo("\r\nLoginPackage: %s",LoginPackage);
                            send_len = sendMessage((void*)&sock,LoginPackage);
                            VnET_echo("\r\nsend_len login: %d",send_len);
                            if(send_len != 38){
                                VnET_echo("\r\nLogin package invalid\n");
                            }
                            else{
                                sentLoginFlag = 1;
                            }
                        }
                        // Check server response
                        if((sentLoginFlag==1)&&(recv_login_package==0)){	
                            waiting_login_gps++;
                            if (waiting_login_gps>=30){
                                sentLoginFlag=-1;
                                waiting_login_gps = 0;
                                VnET_echo("\r\n GPS server not response. Resending...");
                            }
                        }
                        // End a loop 10 mins
                        if (global_counter_1s>=600){
                            sentLoginFlag = -1;
                            recv_login_package = 0;
                            waiting_login_gps = 0;
                            last_time_sent_status = 0;
                            global_counter_1s = 0;
                            read_Battery(&Battery_voltage);
                            sprintf(rMainPower,"%02X",(int)(round(atof(Battery_voltage))));
                            VnET_echo("\r\n###################### SEND LOGIN ###############");
                        }
                    }
                    if (global_counter_1s%25==5){   // each 20 s
                        nwy_nw_get_signal_rssi(&rssi);
                        VnET_echo("\r\nGPS: %d | PPP: %d - RSSI: %d | Server: %d ",!strncmp(GPRMC.Flag,"A",1),call_PPP,(112-rssi)/2,init_TCP);
                    }
                    // Situations is gotten tracking message
                    if (strncmp(GPRMC.Flag,"A",1)==0){
                        distanceGPS = Loc_GetDistance(prev_data_gps.prev_Lat,prev_data_gps.prev_Lng,GPRMC.Latitude/100,GPRMC.Longitude/100)- (GPRMC.Speed/10);
                        delta_GPS = Loc_DeflectionAngle(prev_data_gps.prev_HoC,(float)GPRMC.HalfCourse*2);
                        if (abs(delta_GPS) > (float)360){
                            delta_GPS = 0;
                        }

                        if(global_counter_1s%10==0){
                            VnET_echo("\r\nSpeed: %d | Distance: %.02f | Angular: %.02f ",GPRMC.Speed,distanceGPS,delta_GPS);
                            VnET_echo("\r\nNSendPackageSuccess: %d",NSendPackageSuccess);
                        }
                        
                        if ((!NSendPackageSuccess)&&(sentLoginFlag)&&(recv_login_package)){
                            GPSPackage = Get_GPS_Message();
                            VnET_echo("\r\n[1st package]: %s",GPSPackage);
                            enqueue(queue,GPSPackage);
                            free(GPSPackage);
                            Realtime_Message = Get_Realtime_Message();
                            enqueue(queue,Realtime_Message);
                            free(Realtime_Message);
                            last_time_sent_status = global_counter_1s;
                        }
                        if (GPRMC.Speed < 10){
                            if (GPRMC.Speed > 5){
                                if (global_counter_1s%5==0){
                                    GPSPackage = Get_GPS_Message();
                                    VnET_echo("\r\n[Case 1-tracking]: %s",GPSPackage);
                                    enqueue(queue,GPSPackage);
                                    free(GPSPackage);
                                    VehicleSpeedMAX = 0;
                                }
                            }
                            else{
                                if (global_counter_1s%Tmax==0){
                                    GPSPackage = Get_GPS_Message();
                                    VnET_echo("\r\n[Case 0-tracking]: %s",GPSPackage);
                                    enqueue(queue,GPSPackage);
                                    free(GPSPackage);
                                    VehicleSpeedMAX = 0;
                                }
                            }
                        }
                        else{
                            if (distanceGPS > (float)5){
                                if (distanceGPS > (float)100){  // 100 meters
                                    GPSPackage = Get_GPS_Message();
                                    VnET_echo("\r\n[Case 2-tracking]: %s",GPSPackage);
                                    enqueue(queue,GPSPackage);
                                    free(GPSPackage);
                                }
                                else if ((distanceGPS>(float)40) && (delta_GPS>(float)8)){
                                    GPSPackage = Get_GPS_Message();
                                    VnET_echo("\r\n[Case 3-tracking]: %s",GPSPackage);
                                    enqueue(queue,GPSPackage);
                                    free(GPSPackage);
                                }
                                else if ((distanceGPS > (float)50)&&(GPRMC.Speed > VehicleSpeedMAX)){
                                    VehicleSpeedMAX = GPRMC.Speed;
                                    GPSPackage = Get_GPS_Message();
                                    VnET_echo("\r\n[Case 4-tracking]: %s",GPSPackage);
                                    enqueue(queue,GPSPackage);    
                                    free(GPSPackage);
                                }
                                else if (delta_GPS>=(float)12){
                                    GPSPackage = Get_GPS_Message();
                                    VnET_echo("\r\n[Case 5-tracking]: %s",GPSPackage);
                                    enqueue(queue,GPSPackage);
                                    free(GPSPackage);
                                }
                            }
                        }
                    }
                    // Situations is gotten state message
                    if (strncmp(GPRMC.Flag,"A",1)==0){
                        if (abs(prev_rssi-(112-rssi)/2)>=3){
                            Realtime_Message = Get_Realtime_Message();
                            VnET_echo("\r\n[Case 1-state]: %s",Realtime_Message);
                            enqueue(queue,Realtime_Message);
                            free(Realtime_Message);
                            prev_rssi = (112-rssi)/2;
                            last_time_sent_status = global_counter_1s;
                        }
                        else if (abs(prev_satellite-NoOfSatellite)>=3){
                            Realtime_Message = Get_Realtime_Message();
                            VnET_echo("\r\n[Case 2-state]: %s",Realtime_Message);
                            enqueue(queue,Realtime_Message);
                            free(Realtime_Message);
                            prev_satellite = NoOfSatellite;
                            last_time_sent_status = global_counter_1s;
                        }
                        else if (abs(global_counter_1s-last_time_sent_status)>=350){    // if no any changing--> each 350s, send realtime mesage
                            Realtime_Message = Get_Realtime_Message();
                            VnET_echo("\r\n[Case 0-state]: %s",Realtime_Message);
                            enqueue(queue,Realtime_Message);
                            free(Realtime_Message);
                            last_time_sent_status = global_counter_1s;
                        }
                    }
                    // Send tracking and state message
                    if ((init_TCP == 1)&&(recv_login_package)){
                        if (isEmpty(queue)==0){
                            if(queue->size < 2) send_loop = 1;
                            else if ((queue->size == 2)|| (queue->size == 3))   send_loop = queue->size;
                            else if (queue->size > 3) send_loop = 3;
                            for (int j = 0; j < send_loop; j++){
                                VnET_echo("\r\nQueue state: %d/%d",queue->size,queue->capacity);
                                getPackageFromQueue = front(queue);
                                send_len = sendMessage((void*)&sock,getPackageFromQueue);
                                if (send_len > 0){
                                    NSendPackageSuccess++;
                                }
                                VnET_echo("\r\nsend_len: %d",send_len);
                                if (send_len <= 0){ // Retry to connect is not effective==> add close sock and retry
                                    VnET_echo("\r\n[ERROR] send_len < 0. Check server connection\n");
                                    nwy_socket_close(sock);
                                    init_TCP = 0;
                                    sentLoginFlag = -1;
                                    recv_login_package = -1;
                                }
                                else if (send_len>0){
                                    VnET_echo("\r\nSent: %s",getPackageFromQueue);
                                    simpleDequeue(queue);
                                    if (send_len==LEN_TRACKING_MSG){
                                        prev_data_gps = get_prev_gps_data(getPackageFromQueue);
                                    }
                                }
                                free(getPackageFromQueue);
                                nwy_sleep(150);
                            }
                        }
                    }
                }
                else{
                    if (accValue == nwy_low){
                        INIT_GPS = VnET_init_gps();
                        (INIT_GPS) ? VnET_echo("\r\nInit GPS complete") : VnET_echo("\r\nInit GPS fail");
                    }
                }
                nwy_ext_send_sig(fota_app_thread, MAIN_TICK_10MS);
                break;
            case MAIN_TICK_100MS:
                if ((sleepMode!=0)||(cutOffLED==0)){
                    if (call_PPP){
                        if (recv_login_package){
                            //Sang 200ms - Tat 1800ms
                            led_gsm_state = 1;
                        }
                        else{
                            //Sang full
                            led_gsm_state = 2;
                        }
                    }
                    else if (call_PPP==0){
                        //Sang 1800ms - Tat 200ms
                        led_gsm_state = 3;
                    }

                    if ((INIT_GPS)&&(strncmp(GPRMC.Flag,"A",1)==0)){
                        // Sang 200ms - Tat 1800ms
                        led_gps_state = 1;
                    }
                    else if ((INIT_GPS)&&(strncmp(GPRMC.Flag,"A",1)!=0)){
                        // Sang 100ms - Tat 400
                        led_gps_state = 2;
                    }
                    else{
                        // Tat full
                        led_gps_state = 3;
                    }
                    led_gsm_control(old_led_gsm_state,led_gsm_state);
                    led_gps_control(old_led_gps_state,led_gps_state);
                    old_led_gsm_state = led_gsm_state;
                    old_led_gps_state = led_gps_state;
                }

                // Canh bao tim xe - Alarm02
                if(call_alert==1){
                    counter_call_alert++;
                    if (counter_call_alert>=150){   // alert in 30 second
                        counter_call_alert = 0;
                        call_alert = 0;
                    }
                    if((counter_call_alert%10) < 5){
                        if (atoi(alarm02.relayChannel)==0){
                            nwy_gpio_set_value(OUTPUT1, nwy_high);
                        }
                        else if (atoi(alarm02.relayChannel)==1){
                            nwy_gpio_set_value(OUTPUT2, nwy_high);
                        }
                    }
                    else if((counter_call_alert%10) >= 5){
                        if (atoi(alarm02.relayChannel)==0){
                            nwy_gpio_set_value(OUTPUT1, nwy_low);
                        }
                        else if (atoi(alarm02.relayChannel)==1){
                            nwy_gpio_set_value(OUTPUT2, nwy_low);
                        }
                    }
                }
                
                // Canh bao chuyen dong - Alarm01 - Alarm03 - Alarm05
                if (counterAlertMotion > 0){
                    counterAlertMotion--;
                    if ((atoi(alarm01.informType) == 0) && (informToOwnerAlarm01==0)){
                        // make a call
                        vnet_voice_call(alarm01.phoneNumber);
                        informToOwnerAlarm01 = 1;
                    }
                    else if ((atoi(alarm01.informType) == 1) && (informToOwnerAlarm01==0)){
                    //else if ((atoi(alarm01.operationMode) == 1) && (informToOwnerAlarm01==0)){
                        // send a message
                        vnet_send_sms(alarm01.phoneNumber,"Xe dang di chuyen. Can than mat xe.");
                        informToOwnerAlarm01 = 1;
                    }
                    if (counterAlertMotion%2 == 0){
                        // Alarm 01
                        if (!strncmp(alarm01.relayChannel,"0",strlen(alarm01.relayChannel))){
                            nwy_gpio_set_value(OUTPUT1, nwy_low);
                        }
                        else if (!strncmp(alarm01.relayChannel,"1",strlen(alarm01.relayChannel))){
                            nwy_gpio_set_value(OUTPUT2, nwy_low);
                        }

                        // Alarm 03
                        if (!strncmp(alarm03.relayChannel,"0",strlen(alarm03.relayChannel))){
                            nwy_gpio_set_value(OUTPUT1, nwy_low);
                        }
                        else if (!strncmp(alarm03.relayChannel,"1",strlen(alarm03.relayChannel))){
                            nwy_gpio_set_value(OUTPUT2, nwy_low);
                        }

                        // Alarm 05
                        if (!strncmp(alarm05.relayChannel,"0",strlen(alarm05.relayChannel))){
                            nwy_gpio_set_value(OUTPUT1, nwy_low);
                        }
                        else if (!strncmp(alarm05.relayChannel,"1",strlen(alarm05.relayChannel))){
                            nwy_gpio_set_value(OUTPUT2, nwy_low);
                        }
                    }
                    else if (counterAlertMotion%2 == 1){
                        // Alarm 01
                        if (!strncmp(alarm01.relayChannel,"0",strlen(alarm01.relayChannel))){
                            nwy_gpio_set_value(OUTPUT1, nwy_high);
                        }
                        else if (!strncmp(alarm01.relayChannel,"1",strlen(alarm01.relayChannel))){
                            nwy_gpio_set_value(OUTPUT2, nwy_high);
                        }
                        // Alarm 03
                        if (!strncmp(alarm03.relayChannel,"0",strlen(alarm03.relayChannel))){
                            nwy_gpio_set_value(OUTPUT1, nwy_high);
                        }
                        else if (!strncmp(alarm03.relayChannel,"1",strlen(alarm03.relayChannel))){
                            nwy_gpio_set_value(OUTPUT2, nwy_high);
                        }

                        // Alarm 05
                        if (!strncmp(alarm05.relayChannel,"0",strlen(alarm05.relayChannel))){
                            nwy_gpio_set_value(OUTPUT1, nwy_high);
                        }
                        else if (!strncmp(alarm05.relayChannel,"1",strlen(alarm05.relayChannel))){
                            nwy_gpio_set_value(OUTPUT2, nwy_high);
                        }
                    }
                    if (counterAlertMotion==0){  // alert in 30 second
                        informToOwnerAlarm01 = 0;
                        VnET_echo("\r\n[MOTION] End of alert <---------------------");
                    }
                }


                break;
            default:
                break;
        }
    }
    nwy_exit_thread();
}

int appimg_enter(void *param){
    g_app_thread = nwy_create_thread("mythread", prvThreadEntry, NULL, NWY_OSI_PRIORITY_NORMAL, 1024*64, 64);
    cfg_app_thread = nwy_create_thread("cfgthread", CfgThreadEntry, NULL, NWY_OSI_PRIORITY_NORMAL, 1024*32, 32);
    fota_app_thread = nwy_create_thread("fotathread", FotaThreadEntry, NULL, NWY_OSI_PRIORITY_NORMAL, 1024*32, 32);
    g_timer = nwy_timer_init(g_app_thread, nwy_app_timer_cb, NULL);
    nwy_start_timer_periodic(g_timer, 200); //200ms
    return 0;
}

void appimg_exit(void){
    VnET_echo("Application exit");
}

// =========================== API source
#if GPS
int VnET_init_gps(){
    int ret = -1;
    int result = -1;
    int nmea_freq = 1000;
    int msg_type = 255;
    
    nwy_loc_position_mode_t pos_mode = NWY_LOC_AUX_GNSS_GPS;
    nwy_loc_startup_mode startup = NWY_LOC_HOT_START;

    ret = nwy_loc_start_navigation();   
    if (!ret){
        VnET_echo("\r\n open location fail");
    }
    while(optionCase!=5){
        nwy_sleep(1000);
        switch(optionCase){
        case 1:
            // set position(1-gps 3-gps+bd 5-gps+glo 7-gps+bd+glo)
            // VnET_echo("\r\n set position mode:%d", pos_mode);
            result = nwy_loc_set_position_mode(pos_mode);
            if (result) {
                // VnET_echo("\r\n set position mode success");
                optionCase = 2;
            } 
            else {
                VnET_echo("\r\n set position mode fail");
            }
            break;
        case 2:   	  
            // set location update rate(1000-500-250)ms
            VnET_echo("\r\n#############################################Initing GPS...\n");
            result = nwy_loc_nmea_format_mode(2,nmea_freq);
            if (result) {
                // VnET_echo("\r\n set location update rate success");
                optionCase = 3;
            } 
            else {
                VnET_echo("\r\n set location update rate fail");
            }
            break;
        case 3:       
            // set nmea statements output format(255-all)
            // VnET_echo("\r\n output format:%d", msg_type);
            result = nwy_loc_nmea_format_mode(3,msg_type);
            if (result) {
                // VnET_echo("\r\n set nmea statements output format success");
                    optionCase = 4;
            } else {
                VnET_echo("\r\n set nmea statements output format fail");
            }   
            break;
        case 4: 
            // set startup mode(0-hot 1-warm 2-cold 4-factory)
            startup = 0;
            // VnET_echo("\r\n startup mode:%d",startup);
            result = nwy_loc_set_startup_mode(startup);
            if (result) {
                // VnET_echo("\r\n set nmea statements output format success");
                optionCase = 5;
            } 
            else {
                VnET_echo("\r\n set nmea statements output format fail");
            }  
            break;
            
        default:
            break;
        }
    }
    if (optionCase==5)   return 1;
    else return 0;
}

char * Get_GPS_Message(){
    char *fpackage_GPS;
    fpackage_GPS = (char*)calloc( 72,sizeof(char) );
    // SOF
    memset(package_GPS, 0, sizeof(package_GPS));
    strcat(package_GPS, mSOF);
    // Data type
    strcat(package_GPS, mDataType);
    // Length of Content
    sprintf(mLenOfContent, "%04x", swap16bit(18)); // 18 is length
    strcat(package_GPS, mLenOfContent);
    // Date
    if (GPRMC.Time + 70000 >= 240000){
        sprintf(mDate,"%06d",GPRMC.Date + 10000);
        strcat(package_GPS, mDate);
        sprintf(mTime,"%06d",GPRMC.Time + 70000-240000);
        strcat(package_GPS, mTime);
    }
    else if (GPRMC.Time + 70000 < 240000){
        sprintf(mDate,"%06d",GPRMC.Date);
        strcat(package_GPS, mDate);
        sprintf(mTime,"%06d",GPRMC.Time + 70000);
        strcat(package_GPS, mTime);
    }
    
    if ((GPRMC.oldLatitude != 0) && (GPRMC.oldLongitude != 0) && ((int)GPRMC.Latitude > 200) && ((int)GPRMC.Longitude > 1050)){
        biasX = (uint16_t)((GPRMC.Latitude - GPRMC.oldLatitude)*10000);
        biasY = (uint16_t)((GPRMC.Longitude - GPRMC.oldLongitude)*10000);
        currentAngular = atan2(biasY,biasX)*180/3.1415;
        if (oldAngular == 0) (oldAngular = currentAngular); 
        oldAngular = currentAngular; 
    }
    
    if ( ((GPRMC.Speed >= 5 )) || ((GPRMC.oldLatitude == 0) && (GPRMC.oldLongitude == 0)) ){
        sprintf(mLatitude, "%08d", (int)(GPRMC.Latitude*10000));
        strcat(package_GPS, mLatitude);
        sprintf(mLongitude, "%010d", (int)(GPRMC.Longitude*100000));
        strcat(package_GPS, mLongitude);
        
        GPRMC.oldLatitude = GPRMC.Latitude;
        GPRMC.oldLongitude = GPRMC.Longitude;
    }
    else{
        sprintf(mLatitude, "%08d", (int)(GPRMC.oldLatitude*10000));
        strcat(package_GPS, mLatitude);
        sprintf(mLongitude, "%010d", (int)(GPRMC.oldLongitude*100000));
        strcat(package_GPS, mLongitude);
        
    }
    
    // Location info 
    int calc = !GPRMC.bit_3*2*2*2 + GPRMC.bit_2*2*2 + GPRMC.bit_1*2 + GPRMC.bit_3;
    sprintf(mLocationInfo,"%02x",calc);
    strcat(package_GPS, mLocationInfo);
    // Number of satellite
    sprintf(mNoOfSatellite,"%02d",NoOfSatellite);
    strcat(package_GPS, mNoOfSatellite);
    // Speed
    (GPRMC.Speed < 5) ? sprintf(mSpeed, "%02x", 0) : sprintf(mSpeed, "%02x", GPRMC.Speed);
    strcat(package_GPS, mSpeed);
    // Distance
    float cDistance = (float)(GPRMC.Speed/3600);
    fDistance =  fDistance + cDistance;
    sprintf(mDistance,"%08lx",swap32bit((unsigned int)(fDistance)));
    strcat(package_GPS, mDistance);
    // Half of Course
    sprintf(mHalfOfCourse, "%02x", GPRMC.HalfCourse);
    strcat(package_GPS, mHalfOfCourse);
    // Reserved
    strcat(package_GPS, mReserved);
    //Serial Number
    sprintf(mSerialNo,"%04x",swap16bit(NSendPackageSuccess));
    strcat(package_GPS, mSerialNo);
    // Checksum
    uint16_t CRC = CalCRC16Hex((uint8_t*)package_GPS);
    sprintf(mChecksum,"%04x",swap16bit(CRC));
    strcat(package_GPS, mChecksum);
    // EOF
    strcat(package_GPS, mEOF);
    // GPS package
    sprintf(fpackage_GPS, "%s",package_GPS);
    return fpackage_GPS;
}
char * Get_Realtime_Message(){
    char *fpackage_Realtime;
    fpackage_Realtime = (char*)calloc(61,sizeof(char));
    memset(package_Realtime, 0, sizeof(package_Realtime));
    memset(rStaticState, 0, sizeof(rStaticState));    
    strcat(package_Realtime,rSOF);
    strcat(package_Realtime,rDataType);
    strcat(package_Realtime,rLenOfContent);

    if (GPRMC.Time + 70000 >= 240000){
        sprintf(rDate,"%06d",GPRMC.Date + 10000);
        strcat(package_Realtime, rDate);
        sprintf(rTime,"%06d",GPRMC.Time + 70000 -240000);
        strcat(package_Realtime, rTime);
    }
    else if (GPRMC.Time + 70000 < 240000){
        sprintf(rDate,"%06d",GPRMC.Date);
        strcat(package_Realtime, rDate);
        sprintf(rTime,"%06d",GPRMC.Time + 70000);
        strcat(package_Realtime, rTime);
    }
    strcat(package_Realtime,rBattery);
    strcat(package_Realtime,rMainPower);
    strcat(package_Realtime,rNumberDoorOpened);
    strcat(package_Realtime,rNumberOverSpeed);
    strcat(rStaticState,rStaticState_byte1);
    strcat(rStaticState,rStaticState_byte2);
    strcat(rStaticState,rStaticState_byte3);
    strcat(rStaticState,rStaticState_byte4);
    sprintf(rStaticState_byte5,"%02x",(int)((112-rssi)/2)+64);
    strcat(rStaticState,rStaticState_byte5);
    strcat(rStaticState,rStaticState_byte6);
    strcat(package_Realtime,rStaticState);
    sprintf(rSerialNumber,"%04x",swap16bit(NSendPackageSuccess));
    strcat(package_Realtime, rSerialNumber);               
    uint16_t CRC = CalCRC16Hex((uint8_t*)package_Realtime); 
    sprintf(rCheckSum,"%04x",swap16bit(CRC));             
    strcat(package_Realtime, rCheckSum);                 
    strcat(package_Realtime, hEOF);                      
    sprintf(fpackage_Realtime, "%s",package_Realtime);    
    return fpackage_Realtime;
}
char * Get_Heartbeat_Message(){
    char *fpackage_Heartbeat;
    fpackage_Heartbeat = (char *)calloc(78,sizeof(char)); 
    memset(package_Heartbeat, 0, sizeof(package_Heartbeat));
    strcat(package_Heartbeat,hSOF);
    strcat(package_Heartbeat,hDataType);
    sprintf(hLenOfContent, "%04x", swap16bit(20)); // 20 is length
    strcat(package_Heartbeat,hLenOfContent);
    if (GPRMC.Time + 70000 >= 240000){
        sprintf(hDate,"%06d",GPRMC.Date + 10000);
        strcat(package_Heartbeat, hDate);
        sprintf(hTime,"%06d",GPRMC.Time + 70000-240000);
        strcat(package_Heartbeat, hTime);
    }
    else if (GPRMC.Time + 70000 < 240000){
        sprintf(hDate,"%06d",GPRMC.Date);
        strcat(package_Heartbeat, hDate);
        sprintf(hTime,"%06d",GPRMC.Time + 70000);
        strcat(package_Heartbeat, hTime);
    }
        

    // sprintf(hImei,"8618810507619320");
    strcat(package_Heartbeat, IMEI_NUM); //hImei
    sprintf(hFirewareVersion,"4c64321e30504030");
    strcat(package_Heartbeat, hFirewareVersion);
    char string_Code[58] = {0};
    sprintf(string_Code,"%s%d%s%s%s",tracking_url_cfg,atoi(tracking_port_cfg),hFirewareVersion,hDate,IMEI_NUM);//hImei
    uint16_t CRC_code = GetCrc16((const uint8_t*)string_Code,strlen(string_Code));
    sprintf(hCode,"%04x",swap16bit(CRC_code));
    strcat(package_Heartbeat, hCode);

    strcat(package_Heartbeat,hStatus);
    strcat(package_Heartbeat, hDeviceFamily);
    sprintf(hSerialNo,"%04x",swap16bit(NSendPackageSuccess));
    strcat(package_Heartbeat, hSerialNo);

    uint16_t CRC = CalCRC16Hex((uint8_t*)package_Heartbeat);
    sprintf(hChecksum,"%04x",swap16bit(CRC));
    strcat(package_Heartbeat, hChecksum);

    strcat(package_Heartbeat, hEOF);

    sprintf(fpackage_Heartbeat, "%s",package_Heartbeat);
    return fpackage_Heartbeat;
}

uint16_t swap16bit(int number){
    return ((number >> 8) | (number << 8));
}

uint32_t swap32bit(uint32_t number){
    return (((number>>24)&0xff) | // move byte 3 to byte 0
            ((number<<8)&0xff0000) | // move byte 1 to byte 2
            ((number>>8)&0xff00) | // move byte 2 to byte 1
            ((number<<24)&0xff000000)); // byte 0 to byte 3
}

uint16_t GetCrc16(const uint8_t* pData, uint32_t nLength){
    uint16_t fcs = 0xffff; // Initialize
    while(nLength>0){
        fcs = (fcs >> 8) ^ crctab16[(fcs ^ *pData) & 0xff];
        nLength--;
        pData++;
    }
    return ~fcs; 
}

uint16_t CalCRC16Hex(unsigned char * Data){
    uint8_t mesg[256]={0};
    for (int i = 0; i< strlen(Data); i++){
        if (i % 2 == 0){
            char attemp[2]= {0};
            sprintf(attemp,"%c%c",Data[i],Data[i+1]);
            mesg[i/2] = (int)strtol(attemp, NULL, 16);
        }
    }
    uint16_t crc2 = GetCrc16(mesg,strlen(Data)/2);
    return crc2;
}

float Loc_DeflectionAngle(float anga, float angb){
	float defa;
	if(anga>angb){
		defa = anga - angb;
	}
	else{
		defa = angb - anga;
	}
	if(defa>180) {
		defa = 360-defa;
	}
	return defa;
}

float  Loc_GetDistance(float latA, float lonA, float latB, float lonB){
	xxA = LOC_DEGTORAD(latA);
	xxB = atanf((1 - LOC_INVF) * tanf(xxA));
	sinU1 = sin(xxB); 
	cosU1 = cos(xxB);
	
	xxA = LOC_DEGTORAD(latB);
	xxB = atanf((1 - LOC_INVF) * tanf(xxA));
	sinU2 = sinf(xxB); 
	cosU2 = cosf(xxB);
	
	ValL = LOC_DEGTORAD(lonB - lonA);
	xxA = ValL;
	xxC = 100;
	
	do{
		sinLambda = sinf(xxA); 
		cosLambda = cosf(xxA);
		sinSigma = sqrtf((cosU2 * sinLambda) * (cosU2 * sinLambda) + (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda) * (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda));
		if (sinSigma == 0) return 0;
		cosSigma = sinU1 * sinU2 + cosU1 * cosU2 * cosLambda;
		sigma = atan2(sinSigma, cosSigma);
		sinAlpha = cosU1 * cosU2 * sinLambda / sinSigma;
		cosSqAlpha = 1 - sinAlpha * sinAlpha;
		cos2SigmaM = cosSigma - 2 * sinU1 * sinU2 / cosSqAlpha;

		TmpC = LOC_INVF / 16 * cosSqAlpha * (4 + LOC_INVF * (4 - 3 * cosSqAlpha));
		xxB = xxA;
		xxA = ValL + (1 - TmpC) * LOC_INVF * sinAlpha * (sigma + TmpC * sinSigma * (cos2SigmaM + TmpC * cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM)));
		xxC--;
	} while ((LOC_ABSSUB(xxA, xxB) > ((float)1e-12)) && (xxC > 0));

	if (xxC == 0) return 0;
	xxC = cosSqAlpha * (LOC_EQUR * LOC_EQUR - LOC_POLR * LOC_POLR) / (LOC_POLR * LOC_POLR);
	xxA = 1 + xxC / 16384 * (4096 + xxC * (-768 + xxC * (320 - 175 * xxC)));
	xxB = xxC / 1024 * (256 + xxC * (-128 + xxC * (74 - 47 * xxC)));
	xxC = xxB * sinSigma * (cos2SigmaM + xxB / 4
		* (cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM) - xxB / 6 * cos2SigmaM
				* (-3 + 4 * sinSigma * sinSigma) * (-3 + 4 * cos2SigmaM * cos2SigmaM)));

	return LOC_POLR * xxA * (sigma - xxC);
}

prev_gps_data get_prev_gps_data(char * GPS_Pkg){
    char prev_GPS[72] = {0};
    char str_prev_Lat[10] = {0};
    char str_prev_Lng[10] = {0};
    char str_prev_HoC[2] = {0};
    prev_gps_data prev_data_GPS;
    sprintf(prev_GPS,"%s",GPS_Pkg);
    memcpy( str_prev_Lat, &prev_GPS[24], 8 );   str_prev_Lat[8]= '\0';
    memcpy( str_prev_Lng, &prev_GPS[32], 10 );   str_prev_Lng[10]= '\0';
    prev_data_GPS.prev_Lat = (float)atoi(str_prev_Lat)/1000000;
    prev_data_GPS.prev_Lng = (float)atoi(str_prev_Lng)/10000000;
    memcpy( str_prev_HoC, &prev_GPS[56], 2 );   str_prev_HoC[2]= '\0';
    prev_data_GPS.prev_HoC = (float)strtol(str_prev_HoC, NULL, 16)*2;
    return prev_data_GPS;
}

int deinit_GPS(){
    int deinitGPS = nwy_loc_stop_navigation();
    if (deinitGPS) {
        VnET_echo("\r\n close location position success");
        return 1;
    } else {
        VnET_echo("\r\n close location position fail");
        return -1;
    }
}
#endif

#if PPP
int vnet_call_ppp(int waitingFlag){
    profile_index ++;
    hndl_ppp = nwy_data_get_srv_handle(nwy_data_cb_fun);
    nwy_data_profile_info_t profile_info;
    memset(&profile_info,0,sizeof(nwy_data_profile_info_t));
    profile_info.auth_proto = NWY_DATA_AUTH_PROTO_NONE;
    profile_info.pdp_type = NWY_DATA_PDP_TYPE_PPP;
    memcpy(profile_info.apn,"v-internet",sizeof(profile_info.apn));
    memcpy(profile_info.user_name,"",sizeof(profile_info.user_name));
    memcpy(profile_info.pwd,"",sizeof(profile_info.pwd));
    int ret = nwy_data_set_profile(1,NWY_DATA_PROFILE_3GPP,&profile_info);
    memset(&profile_info,0,sizeof(nwy_data_profile_info_t));
    ret = nwy_data_get_profile(1,NWY_DATA_PROFILE_3GPP,&profile_info);
    nwy_data_start_call_v02_t param_t;
    param_t.profile_idx = profile_index;
    param_t.is_auto_recon = 1;
    param_t.auto_re_max = 0;
    param_t.auto_interval_ms = 420000;
    ret = nwy_data_start_call(hndl_ppp,&param_t);
    if (ret != NWY_RES_OK){
        VnET_echo("\r\nStart data call fail, result<%d>",ret);
        PPPConnection = 0;
    }
    else{
        VnET_echo("\r\nStart data call success...");
        PPPConnection = 1;
    }
    return PPPConnection;
}

static void nwy_data_cb_fun(int hndl,nwy_data_call_state_t ind_state){
  if (hndl > 0 && hndl <= 8){
    ppp_state[hndl-1] = ind_state;
  }
}

int nwy_ext_check_data_connect(){
    int i = 0;
    for (i = 0; i< NWY_DATA_CALL_MAX_NUM; i++) {
        if (ppp_state[i] == NWY_DATA_CALL_CONNECTED) {
            return 1;
        }
    }
    return 0;
}

// This function is using for tracking only
int vnet_tcp_tracking_init(char * url_or_ip, int port){
    nwy_ip_type_or_dns_enum ip_dns_type =-1;
    char *ip_str =NULL;
    char ip_buf[64] = {0};
    int isipv6 =0;
    int on = 1;
    int opt = 1,ret = 0;
    uint64_t start ;
    ip_addr_t addr = {0};

    struct sockaddr_in sa;
    struct sockaddr_in *to4 = (struct sockaddr_in *)&sa;
    while (!tcp_connect_flag){
        nwy_sleep(1000);
        if (0 == nwy_ext_check_data_connect()) {
            VnET_echo("\r\nTracking: data call not setup");
            break;
        }
        if(tcp_connect_flag){
            VnET_echo("\r\ntcp alreay connect");
            break;
        }
        VnET_echo("\r\nServer info:%s:%d", url_or_ip, port);
        ip_dns_type =nwy_judge_ip_or_dns(url_or_ip);
        // VnET_echo("\r\nip_dns_type: %d\n",ip_dns_type);
        if(ip_dns_type == 2){
            ip_str = nwy_gethostbyname1(url_or_ip, &isipv6);
            if(ip_str == NULL || 0==strlen(ip_str)){
                VnET_echo("\r\n%s get ip fail", url_or_ip);
                break;
            }
            VnET_echo("\r\n%s get ip:%s\r\n", url_or_ip, ip_str);
            ret = nwy_hostname_check(ip_str, ip_buf);
            if (ret != NWY_SUCESS) {
                VnET_echo("\r\ntcp ip error:");
                break;
            }
            if (ipaddr_aton(ip_buf, &addr) == 0){
                VnET_echo("\r\ntcp test fail1:");
                break;
            }
        } 
        else {
            ret = nwy_hostname_check(url_or_ip, ip_buf);
            if (ret != NWY_SUCESS) {
                VnET_echo("\r\ntcp ip error:");
                break;
            }
            addr.u_addr.ip4.addr = ipaddr_addr(ip_buf);
        }
        if (sock == 0) {
            sock= nwy_socket_open(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        }
        to4->sin_len = sizeof(struct sockaddr_in);
        to4->sin_family = AF_INET;
        to4->sin_port = htons(port);
        inet_addr_from_ip4addr(&to4->sin_addr, ip_2_ip4(&addr));
        nwy_socket_setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (void *)&on, sizeof(on));
        nwy_socket_setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (void *)&opt, sizeof(opt));
        if (0 != nwy_socket_set_nonblock(sock)){
            VnET_echo("\r\nsocket set err");
            break;
        }
        start = nwy_get_ms();
        do
        {
            ret = nwy_socket_connect(sock, (struct sockaddr *)&sa, sizeof(sa));
            if((nwy_get_ms()-start)>= 10000 ){
                VnET_echo("\r\nsocket connect timeout");
                nwy_socket_shutdown(sock, SHUT_RD);
                nwy_socket_close(sock);
                sock = 0;
                break;
            }
            if(ret == -1){
                if( EALREADY == nwy_socket_errno() || EISCONN == nwy_socket_errno() ){                        
                    VnET_echo( "\r\nnwy_net_connect_tcp connect ok..");
                    tcp_connect_flag = 1;
                    break;
                }
                if (EINPROGRESS != nwy_socket_errno() ){
                    VnET_echo("\r\nconnect errno = %d", nwy_socket_errno());
                    nwy_socket_close(sock);
                    sock = 0;
                    tcp_connect_flag = 0;
                    break;
                }
            }
            nwy_sleep(100);
        }while(1);
        if(tcp_connect_flag){
            tcp_recv_thread = nwy_create_thread("tcp_recv_thread",nwy_tcp_recv_tracking, (void*)&sock, NWY_OSI_PRIORITY_NORMAL, 1024*16, 16);
            if(tcp_recv_thread ==NULL){
                VnET_echo("\r\ncreate tcp recv thread failed, close connect");
                nwy_socket_close(sock);
                sock = 0;
                tcp_connect_flag = 0;
            }
        }
    }
    return tcp_connect_flag; 
}

static void nwy_tcp_recv_tracking(void *param){
    char MesgRecv[512] = {0};
    char recv_buff[NWY_UART_RECV_SINGLE_MAX +1] = {0};
    int recv_len = -1, result = 0;
    int s = *(int *)param;
    fd_set rd_fd;
    fd_set ex_fd;
    FD_ZERO(&rd_fd);
    FD_ZERO(&ex_fd);
    FD_SET(s,&rd_fd);
    FD_SET(s,&ex_fd);
    struct timeval tv = {0};
    tv.tv_sec = 20;
    tv.tv_usec = 0;
    while(1){
        FD_ZERO(&rd_fd);
        FD_ZERO(&ex_fd);
        FD_SET(s,&rd_fd);
        FD_SET(s,&ex_fd);
        result = nwy_socket_select(s+1, &rd_fd, NULL,&ex_fd, &tv);
        if (result < 0){
          VnET_echo("\r\ntcp select error:\r\n");
          nwy_socket_close(s);
          tcp_connect_flag =0;
          nwy_exit_thread();
        }
        else if(result > 0){
            if (FD_ISSET(s, &rd_fd)){
                memset(recv_buff, 0, NWY_UART_RECV_SINGLE_MAX +1);
                recv_len = nwy_socket_recv(s, recv_buff, NWY_UART_RECV_SINGLE_MAX, 0);
                if(recv_len >0){ 
                    size_t count = 0;
                    for(count = 0; count < recv_len; count++){
                        if (count==0){
                            sprintf(MesgRecv,"%02x",recv_buff[count]);
                        }
                        else{
                            sprintf(MesgRecv,"%s%02x",MesgRecv,recv_buff[count]);
                        }
                    }
                    MesgRecv[recv_len*2]= '\0';
                    VnET_echo("\r\n--> Receive %d: %s ",recv_len,MesgRecv);
                    if(!strncmp(LoginPackage,MesgRecv,76)){
                        recv_login_package = 1;
                    }
                    free(LoginPackage);
                }
                else if((recv_len == 0)||(sleepMode==0)){
                    VnET_echo("\r\ntracking tcp close by server\r\n");
                    init_TCP = 0;
                    sentLoginFlag = -1;
                    recv_login_package = 0;
                    FD_SET(s,&ex_fd);
                }  
                else{
                    VnET_echo("\r\ntracking tcp connection error\r\n");
                    nwy_socket_close(s);
                    init_TCP = 0;
                    tcp_connect_flag =0;
                    nwy_exit_thread();
                }
            }
            if (FD_ISSET(s, &ex_fd)){
                VnET_echo("\r\ntracking tcp select ex_fd:\r\n");
                nwy_socket_close(s);
                tcp_connect_flag =0;
                nwy_exit_thread();
            }
        }
        else    
            OSI_LOGI(0,"\r\ntcp select timeout:\r\n");
        nwy_sleep(1000);
    }
}

static int nwy_hostname_check(char *hostname, char *ip_buf){
    int a,b,c,d;
    char temp[32] = {0};
    if(strlen(hostname) > 15)
    return NWY_GEN_E_UNKNOWN;

    if((sscanf(hostname,"%d.%d.%d.%d",&a,&b,&c,&d))!=4)
        return NWY_GEN_E_UNKNOWN;

    if(!((a <= 255 && a >= 0)&&(b <= 255 && b >= 0)&&(c <= 255 && c >= 0)))
        return NWY_GEN_E_UNKNOWN;

    sprintf(temp,"%d.%d.%d.%d",a,b,c,d);
    strcpy(ip_buf, temp);
    return NWY_SUCESS;
}

nwy_ip_type_or_dns_enum nwy_judge_ip_or_dns(char *str){
	int len= 0;
	int strLen = 0;
	nwy_ip_type_or_dns_enum retValue = NWY_CUSTOM_IP_TYPE_OR_DNS_DNS;
	if(str == NULL ){
		return NWY_CUSTOM_IP_TYPE_OR_DNS_NONE;
	}
	else{
		if(strlen(str) <= 0 ){
			return NWY_CUSTOM_IP_TYPE_OR_DNS_NONE;
		}
	}

	strLen = strlen(str);

	for(len = 0;len < strLen; len++){
		if( ((*(str+len) >= '0') && (*(str+len) <= '9')) || (*(str+len) == '.') ){
			continue;
		}
		else{
			break;
		}
	}
	if(len == strLen){
		retValue = NWY_CUSTOM_IP_TYPE_OR_DNS_IPV4;
		return retValue;
	}
	len = 0;
	for(len = 0;len < strLen; len++){
		if( ((*(str+len) >= '0') && (*(str+len) <= '9')) ||
			((*(str+len) >= 'a') && (*(str+len) <= 'f')) ||
			((*(str+len) >= 'A') && (*(str+len) <= 'F')) ||
			(*(str+len) == ':')){
			continue;
		}
		else{
			break;
		}
	}
	if(len == strLen){
		retValue = NWY_CUSTOM_IP_TYPE_OR_DNS_IPV6;
		return retValue;
	}
	return retValue;
}

int sendMessage(void *socketID, char * hexString){
    int sock = *(int *)socketID;
    uint8_t mesg[256]={0};
    for (int i = 0; i< strlen(hexString); i++){
        if (i % 2 == 0){
            char attemp[2] = {0};
            sprintf(attemp,"%c%c",hexString[i],hexString[i+1]);
            mesg[i/2] = (int)strtol(attemp, NULL, 16);
        }
    }
    int _send = nwy_socket_send(sock , mesg , strlen(hexString)/2,0);
    return _send;
}

int stop_data_call(void * handler){
    int hdl = *(int *)handler;
    int stopDataCall = nwy_data_stop_call(hdl);
    if (stopDataCall != NWY_RES_OK){
        VnET_echo("\r\nStop data call fail");
        return -1;
    }
    else{
        VnET_echo("\r\nStop data call success");
        return 1;
    }
}

#endif

#if iQueue
struct Queue* createQueue(int capacity){
	struct Queue* queue = (struct Queue*)calloc(1,sizeof(struct Queue));
	queue->capacity = capacity;
	queue->front = 0;
	queue->size = 0;
	queue->rear = capacity - 1;
	return queue;
}

int isFull(struct Queue* queue){
	return (queue->size == queue->capacity);
}

char * front(struct Queue* queue){
	char * item = (char *)calloc(76,sizeof(char));
	sprintf(item,"%s",queue->array[queue->front]);
	return item;
}

int isEmpty(struct Queue* queue){
	return (queue->size == 0);
}

int queueValid(){
	if (queue->size==0){
        return 0;
    }
    else{
        return 1;
    }
}

void enqueue(struct Queue* queue, char * item){
	if (!isFull(queue)){
		queue->rear = (queue->rear + 1)% queue->capacity;
		sprintf(queue->array[queue->rear],"%s",item);
		queue->size = queue->size + 1;
	}
}


void simpleDequeue(struct Queue* queue){
    queue->front = (queue->front + 1) % queue->capacity;
    queue->size = queue->size - 1;
    VnET_echo("\r\n>>>>>>Queue has ben dequeue.\n");
}

void checkOverFlowQueue(struct Queue* queue){
	if (isFull(queue)){
		queue->rear = (queue->rear - 1) % queue->capacity;
		queue->size = queue->size - 1;
	}
}
#endif

#if TMR
void nwy_app_timer_cb(void){
    tickCount100MS++;
    if(tickCount100MS >= 5){
        tickCount100MS = 0;
        nwy_ext_send_sig(g_app_thread, MAIN_TICK_1000MS);
    }
    nwy_ext_send_sig(g_app_thread, MAIN_TICK_100MS);
}
#endif

#if Peripheral
int init_4led(){
    nwy_gpio_set_direction(LED_GPS_E,1);  // 1-out 0-in
    nwy_gpio_set_direction(LED_GSM_E,1);  // 1-out 0-in
    nwy_gpio_set_direction(OUTPUT1,1);  // 1-out 0-in
    nwy_gpio_set_direction(OUTPUT2,1);  // 1-out 0-in
    return 1;
}

void led_gsm_control(int old_state, int pres_state){
    if(pres_state==1){
        light_on_ms = 1;
        light_off_ms= 9;
    }
    else if(pres_state==2){
        light_on_ms = 10;
        light_off_ms= 0;
    }
    else if(pres_state==3){
        light_on_ms = 9;
        light_off_ms= 1;
    }
    if ((old_state-pres_state==0)&&(old_state!=0)&&(pres_state!=0)){
        if (counter_led_gsm<=light_on_ms){
            counter_led_gsm++;
            turn_led_on(LED_GSM_E);
        }
        else if ((counter_led_gsm>light_on_ms)&&(counter_led_gsm<=(light_on_ms+light_off_ms))){
            counter_led_gsm++;
            turn_led_off(LED_GSM_E);
        }
        else if (counter_led_gsm>(light_on_ms+light_off_ms)){
            counter_led_gsm=0;
        }
    }
    else{
        counter_led_gsm = 0;
    }
}

void led_gps_control(int old_state, int pres_state){
    if(pres_state==1){
        light_on_ms = 1;
        light_off_ms= 9;
    }
    else if(pres_state==2){
        light_on_ms = 1;
        light_off_ms= 3;
    }
    else {
        light_on_ms = 0;
        light_off_ms= 10;
    }
    if ((old_state-pres_state==0)&&(old_state!=0)&&(pres_state!=0)){
        if (counter_led_gps<=light_on_ms){
            counter_led_gps++;
            turn_led_on(LED_GPS_E);
        }
        else if ((counter_led_gps>light_on_ms)&&(counter_led_gps<=(light_on_ms+light_off_ms))){
            counter_led_gps++;
            turn_led_off(LED_GPS_E);
        }
        else if (counter_led_gps>(light_on_ms+light_off_ms)){
            counter_led_gps=0;
        }
    }
    else{
        counter_led_gps = 0;
    }
}

void nwy_adc(){
    int value = -1;
    int channel = 4; //2-CHANNEL2,3-CHANNEL3,4-VBAT
    int scale = 3;   //ADC scale(0-1V250,1-2V444,2-3V233,3-5V000)
    value = nwy_adc_get(channel,scale);
    VnET_echo("\r\nnwy adc get value = %d", value);
    
}

static void read_ACC(int param){
    accValue = nwy_gpio_get_value(ACC);

    if (accValue == nwy_low){           // ACC ON
        VnET_echo("\r\nACC is ON++++++++++++++++++++++++++++++++++++");
        if (exceptionReboot==1){
            VnET_echo("[EXCEPTION] Force reboot due to rolling in sleep mode has problem");
            nwy_power_off(2);
        }
        if (LEDHandle == -1){
            LEDHandle = init_4led();
        }
        if ((sleepMode==0)){
            int retSetMode = nwy_pm_state_set(NWY_WAKEUP);
            if (retSetMode==0){
                VnET_echo("\r\n Rolled in awake mode");
                sleepMode = 1;
                uptime_ms = nwy_get_ms();
                sleepingTimeM = sleepingTimeM + (uptime_ms-prevTime)/60000;
                prevTime = uptime_ms;
            }
            else{
                VnET_echo("\r\n Power set failed");
            }
        }
        timeAccOn = nwy_get_ms();
        cutOffLED = 0;
        VnET_echo("\r\nMoment ACC is on: %ld s",(long) timeAccOn / 1000);
    }
    else if (accValue == nwy_high){     // ACC OFF
        VnET_echo("\r\nACC is OFF-----------------------------------");
    }
}

void check_reboot_time(){
    nwy_get_time(&vnet_time, &fota_timezone);
    if ((vnet_time.hour == atoi(hour_reboot)) && (vnet_time.min == atoi(minute_reboot))){
        uptime_ms = nwy_get_ms();
        if(uptime_ms > (long)3600000){
            nwy_power_off(2);
        }
    }
}

int deinit_led(){
    nwy_sleep(100);
    int retDeinitGpsLed  = nwy_close_gpio(LED_GPS_E);
    nwy_sleep(100);
    int retDeinitGsmLed  = nwy_close_gpio(LED_GSM_E);
    nwy_sleep(100);
    int retDeinitOut1Led = nwy_close_gpio(OUTPUT1);
    nwy_sleep(100);
    int retDeinitOut2Led = nwy_close_gpio(OUTPUT2);
    nwy_sleep(100);
    if ((retDeinitGpsLed)&&(retDeinitGsmLed)&&(retDeinitOut1Led)&&(retDeinitOut2Led)){
        VnET_echo("\r\nDeinit all LED");
        return 1;
    }
    else{
        VnET_echo("\r\nDeinit LED fail: GPS: %d | GSM: %d | OUT1: %d | OUT2: %d");
        return 0;
    }
}

void push(float val){
    if(idxTop==STACKSIZE){
        for (int idx=0;idx<STACKSIZE;idx++){
            if (idx!=STACKSIZE-1){
                stackVoltage[idx] = stackVoltage[idx+1];
            }
        }
        idxTop = idxTop -1;
    }
    if(idxTop!=STACKSIZE){
        stackVoltage[idxTop]=val;
        idxTop=idxTop+1;
    }
    // VnET_echo("\r\nStack content: ");
    // for(int i=0;i<idxTop;i++)
    //     VnET_echo("[%d] %02.02f",i,stackVoltage[i]);
}

float average_voltage(){
    float sumVoltage = 0.0;
    for(int i=0;i<idxTop;i++)
        sumVoltage += stackVoltage[i];
    return (sumVoltage/idxTop);
}

// Charging function
int init_charger_pin(){
    int ret = nwy_gpio_set_direction(CHARGER,1);  // 1-out 0-in
    return ret;
}

int enable_charger(){
    int ret = nwy_gpio_set_value(CHARGER,nwy_high);
    return ret;
}

int disable_charger(){
    int ret = nwy_gpio_set_value(CHARGER,nwy_low);
    return ret;
}

#endif

#if FOTA
int vnet_tcp_fota_init(char * url_or_ip, int port){
    nwy_ip_type_or_dns_enum fota_ip_dns_type =-1;
    char *fota_ip_str =NULL;
    char fota_ip_buf[64] = {0};
    int fota_isipv6 =0;
    int fota_on = 1;
    int fota_opt = 1,fota_ret = 0;
    uint64_t fota_start ;
    ip_addr_t fota_addr = {0};

    struct sockaddr_in fota_sa;
    struct sockaddr_in *fota_to4 = (struct sockaddr_in *)&fota_sa;

    while (!tcp_connect_fota_flag){
        nwy_sleep(1000);
        if (0 == nwy_ext_check_data_connect()) {
            VnET_echo("\r\nFOTA: data call not setup\n");
            break;
        }
        if(tcp_connect_fota_flag){
            VnET_echo("\r\ntcp alreay connect");
            break;
        }
        VnET_echo("\r\nServer info:%s:%d\r\n", url_or_ip, port);
        fota_ip_dns_type =nwy_judge_ip_or_dns(url_or_ip);
        if(fota_ip_dns_type == 2){
            fota_ip_str = nwy_gethostbyname1(url_or_ip, &fota_isipv6);
            if(fota_ip_str == NULL || 0==strlen(fota_ip_str)){
                VnET_echo("\r\n%s get ip fail\r\n", url_or_ip);
                break;
            }
            VnET_echo("\r\n%s get ip:%s\r\n", url_or_ip, fota_ip_str);
            fota_ret = nwy_hostname_check(fota_ip_str, fota_ip_buf);
            if (fota_ret != NWY_SUCESS) {
                VnET_echo("\r\ntcp ip error:\r\n");
                break;
            }
            if (ipaddr_aton(fota_ip_buf, &fota_addr) == 0){
                VnET_echo("\r\ntcp test fail1:\r\n");
                break;
            }
        } 
        else {
            fota_ret = nwy_hostname_check(url_or_ip, fota_ip_buf);
            if (fota_ret != NWY_SUCESS) {
                VnET_echo("\r\ntcp ip error:\r\n");
                break;
            }
            fota_addr.u_addr.ip4.addr = ipaddr_addr(fota_ip_buf);
        }
        if (fota_sock == 0) {
            fota_sock= nwy_socket_open(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        }
        fota_to4->sin_len = sizeof(struct sockaddr_in);
        fota_to4->sin_family = AF_INET;
        fota_to4->sin_port = htons(port);
        inet_addr_from_ip4addr(&fota_to4->sin_addr, ip_2_ip4(&fota_addr));
        nwy_socket_setsockopt(fota_sock, SOL_SOCKET, SO_REUSEADDR, (void *)&fota_on, sizeof(fota_on));
        nwy_socket_setsockopt(fota_sock, IPPROTO_TCP, TCP_NODELAY, (void *)&fota_opt, sizeof(fota_opt));
        if (0 != nwy_socket_set_nonblock(fota_sock)){
            VnET_echo("\r\nsocket set err\r\n");
            break;
        }
        fota_start = nwy_get_ms();
        do
        {
            fota_ret = nwy_socket_connect(fota_sock, (struct sockaddr *)&fota_sa, sizeof(fota_sa));
            if((nwy_get_ms()-fota_start)>= 10000 ){
                VnET_echo("\r\nfota socket connect timeout\r\n");
                nwy_socket_shutdown(fota_sock, SHUT_RD);
                nwy_socket_close(fota_sock);
                fota_sock = 0;
                break;
            }
            if(fota_ret == -1){
                if( EALREADY == nwy_socket_errno() || EISCONN == nwy_socket_errno() ){                        
                    VnET_echo( "\r\nnwy_net_connect_tcp connect ok..\n");
                    tcp_connect_fota_flag = 1;
                    break;
                }
                if (EINPROGRESS != nwy_socket_errno() ){
                    VnET_echo("\r\nfota connect errno = %d", nwy_socket_errno());
                    nwy_socket_close(fota_sock);
                    fota_sock = 0;
                    tcp_connect_fota_flag = 0;
                    break;
                }
            }
            nwy_sleep(100);
        }while(1);
        if(tcp_connect_fota_flag){
            tcp_recv_fota_thread = nwy_create_thread("tcp_recv_fota_thread",vnet_tcp_recv_fota, (void*)&fota_sock, NWY_OSI_PRIORITY_NORMAL, 1024*16, 16);
            if(tcp_recv_fota_thread ==NULL){
                VnET_echo("\r\ncreate tcp recv thread failed, close connect");
                nwy_socket_close(fota_sock);
                fota_sock = 0;
                tcp_connect_fota_flag = 0;
            }
        }
    }
    return tcp_connect_fota_flag; 
}

static void vnet_tcp_recv_fota(void *param){
    char fota_recv_buff[NWY_UART_RECV_SINGLE_MAX +1] = {0};
    uint32_t fota_MesgRecv[2048];
    int fota_recv_len =0, fota_result = 0;
    int fota_socket = *(int *)param;
    fd_set fota_rd_fd;
    fd_set fota_ex_fd;
    FD_ZERO(&fota_rd_fd);
    FD_ZERO(&fota_ex_fd);
    FD_SET(fota_socket,&fota_rd_fd);
    FD_SET(fota_socket,&fota_ex_fd);
    struct timeval fota_tv = {0};
    fota_tv.tv_sec = 20;
    fota_tv.tv_usec = 0;
    while(1){
        FD_ZERO(&fota_rd_fd);
        FD_ZERO(&fota_ex_fd);
        FD_SET(fota_socket,&fota_rd_fd);
        FD_SET(fota_socket,&fota_ex_fd);
        fota_result = nwy_socket_select(fota_socket+1, &fota_rd_fd, NULL,&fota_ex_fd, &fota_tv);
        if (fota_result < 0){
          nwy_ext_echo("\r\ntcp select error:\r\n");
          nwy_socket_close(fota_socket);
          tcp_connect_fota_flag =0;
          nwy_exit_thread();
        }
        else if(fota_result > 0){
            if (FD_ISSET(fota_socket, &fota_rd_fd)){
                memset(fota_recv_buff, 0, NWY_UART_RECV_SINGLE_MAX +1);
                memset(fota_data_bin_file, 0, 1024);
                fota_recv_len = nwy_socket_recv(fota_socket, fota_recv_buff, NWY_UART_RECV_SINGLE_MAX, 0);
                // fota_recv_buff[fota_recv_len]= '\0';
                
                if(fota_recv_len >0){ 
                    size_t count = 0;
                    for(count = 0; count < fota_recv_len; count++){
                        if (count==0){
                            sprintf(fota_MesgRecv,"%02x",fota_recv_buff[count]);
                        }
                        else{
                            sprintf(fota_MesgRecv,"%s%02x",fota_MesgRecv,fota_recv_buff[count]);
                        }
                    }
                    fota_MesgRecv[fota_recv_len*2]= '\0';
                    VnET_echo("\r\n--> Receive: %s \n",fota_MesgRecv);
                    memcpy( Header_RecvMessage, &fota_MesgRecv[0], 4 );
                    Header_RecvMessage[4]= '\0';
                    if (!strcmp(Header_RecvMessage,"2424")){
                        memcpy( DataType_RecvMessage, &fota_MesgRecv[1], 4 );
                        DataType_RecvMessage[4]= '\0';
                        if (!strcmp(DataType_RecvMessage,"01ff")){
                            VnET_echo("\r\n--> Receive login feedback message: %s \n",fota_MesgRecv);
                            recv_fota_login_package = 1;
                            waiting_login_fota = 0;
                        }
                        else if (!strcmp(DataType_RecvMessage,"03ff")){
                            VnET_echo("\r\n--> Receive firmware feedback message: %s \n",fota_MesgRecv);
                            memcpy( PackageSize_RecvMessage, &fota_MesgRecv[7], 8 );
                            PackageSize_RecvMessage[8]='\0';
                            char arr[4][4];
                            char swapped[8];
                            for (int i = 0; i< strlen(PackageSize_RecvMessage); i++){
                                if (i % 2 == 0){
                                    sprintf(arr[i/2],"%c%c",PackageSize_RecvMessage[i],PackageSize_RecvMessage[i+1]);
                                }
                            }
                            sprintf(swapped,"%s%s%s%s",arr[3],arr[2],arr[1],arr[0]);
                            fota_package_size = strtol(swapped,NULL,16);
                            VnET_echo("\r\n<> fota_package_size: %ld\n",fota_package_size);
                            memcpy( NumPackage_RecvMessage, &fota_MesgRecv[9], 4 );
                            NumPackage_RecvMessage[4]='\0';
                            fota_number_package = swap16bit(strtol(NumPackage_RecvMessage,NULL,16));
                            VnET_echo("\r\n<> fota_number_package: %d\n",fota_number_package);
                            nwy_ext_send_sig(fota_app_thread, FOTA_UPDATE);
                        }
                        else if (!strcmp(DataType_RecvMessage,"05ff")){
                            nwy_sleep(100);
                            memcpy( fota_packageID, &fota_MesgRecv[7], 4 );
                            fota_packageID[4]='\0';
                            fota_id_package_recv = swap16bit(strtol(fota_packageID,0,16));
                            VnET_echo("\r\n--> Received package: %d/%d\n",fota_id_package_recv,fota_number_package);

                            memcpy( fota_data_bin_file, &fota_MesgRecv[8], strlen(fota_MesgRecv)-12-32);
                            fota_data_bin_file[strlen(fota_MesgRecv)-12-32]='\0';
                            if (fota_id_package_recv==0){
                                VnET_echo("\r\n--> Receive data inside bin file: %s\n",fota_data_bin_file);
                            }   
                            fota_bytes_recv = fota_bytes_recv + strlen(fota_data_bin_file)/2;
                            for (int i = 0; i< strlen(fota_data_bin_file); i++){
                                if (i % 2 == 0){
                                    char attemp[2];
                                    sprintf(attemp,"%c%c",fota_data_bin_file[i],fota_data_bin_file[i+1]);
                                    fota_store_firm[fota_pos_firm] = (int)strtol(attemp, NULL, 16);
                                    fota_pos_firm++;
                                }
                            }

                            if ((fota_id_package_recv==0)&&(first_byte_hex==0)){
                                char first_byte[2];
                                memcpy(first_byte,&fota_data_bin_file[0],2);
                                first_byte[2] = '\0';
                                first_byte_hex = strtol(first_byte,NULL,16);
                                VnET_echo("\r\nFirst byte: %s | %2x",first_byte, first_byte_hex);
                            }
                            VnET_echo("\r\n--> Firmware load: %ld/%ld\n",fota_pos_firm,fota_package_size);
                            if (((fota_bytes_recv-fota_package_size)==0)&&(fota_number_package-fota_id_package_recv==1)){
                                VnET_echo("\r\n--> Received the whole bin file\n");
                                nwy_ext_send_sig(fota_app_thread, NWY_EXT_APPIMG_FOTA_DATA_REC_END);
                            }

                            if(fota_bytes_recv<fota_package_size){
                                nwy_ext_send_sig(fota_app_thread, FOTA_UPDATE);
                            }
                        }
                    }
                    else{
                        unsigned char* command = hexstr_to_char(fota_MesgRecv);
                        VnET_echo("\r\nCommand get: %s\n",command);
                        if (!strcmp(command,"update#")){
                            VnET_echo("\r\nGet update command\n");
                            fota_FirmwareMessage = Get_FOTA_Firmware_Message();
                            send_len = fota_sendMessage((void*)&fota_socket, fota_FirmwareMessage);
                            VnET_echo("\r\nsend_len FOTAFirmwareMessage: %d\n",send_len);
                            if (send_len==20){
                                VnET_echo("\r\nSent firmware FOTA message successfully\n");
                            }
                            else{
                                VnET_echo("\r\nSent firmware FOTA failed. Retrying...\n");
                            }
                            free(fota_FirmwareMessage);
                        }
                        // if (findSpecialChar(command,strlen(command),'#')>0){
                        //     VnET_echo("\r\n[FOTA] Valid command");
                        //     Center_Handle_Command(command,strlen(command),SRC_FOTA);
                        // }
                        // else{
                        //     VnET_echo("\r\n[FOTA] Invalid command");
                        // }
                        free(command);
                    }
                }
                else if((fota_recv_len == 0)||(fota_global_counter >= FOTA_LOGIN_CYCLE/2)){
                    VnET_echo("\r\nfota tcp close by server\r\n");
                    recv_fota_login_package = 0;
                    FOTA_login_flag = 0;
                    init_fota_TCP = 0;
                    FD_SET(fota_socket,&fota_ex_fd);
                }  
                else{
                    VnET_echo("\r\nfota tcp connection error\r\n");
                    nwy_socket_close(fota_socket);
                    tcp_connect_fota_flag =0;
                    recv_fota_login_package = 0;
                    init_fota_TCP = 0;
                    nwy_exit_thread();
                }
            }
            if (FD_ISSET(fota_socket, &fota_ex_fd)){
                VnET_echo("\r\nfota tcp select fota_ex_fd:\r\n");
                nwy_socket_close(fota_socket);
                tcp_connect_fota_flag =0;
                recv_fota_login_package = 0;
                nwy_exit_thread();
            }
        }
        else    
            OSI_LOGI(0,"\r\ntcp select timeout:\r\n");
        nwy_sleep(500);
    }
}

int fota_sendMessage(void *socketID, char * hexString){
    int sock = *(int *)socketID;
    uint8_t mesg[256]={0};
    for (int i = 0; i< strlen(hexString); i++){
        if (i % 2 == 0){
            char attemp[2] = {0};
            sprintf(attemp,"%c%c",hexString[i],hexString[i+1]);
            mesg[i/2] = (int)strtol(attemp, NULL, 16);
        }
    }
    int _send = nwy_socket_send(sock , mesg , strlen(hexString)/2,0);
    return _send;
}

char *Get_FOTA_Login_Message(){
    char *Login_Package;
    Login_Package = (char*)calloc(133,sizeof(char));
    memset(fota_LoginPackage, 0, strlen(fota_LoginPackage));
    strcat(fota_LoginPackage, fota_SOF);
    strcat(fota_LoginPackage, fota_Login_DataType);
    strcat(fota_LoginPackage, fota_Login_Length);
    nwy_get_time(&fota_vnet_time, &fota_timezone);
    sprintf(fota_DateTime, "%02d%02d%02d%02d%02d%02d", fota_vnet_time.day,fota_vnet_time.mon,fota_vnet_time.year-2000, fota_vnet_time.hour,fota_vnet_time.min,fota_vnet_time.sec);
    strcat(fota_LoginPackage, fota_DateTime);
    strcat(fota_LoginPackage, fota_Code);

    // strcat(fota_LoginPackage, fota_ID);
    strcat(fota_LoginPackage, IMEI_NUM);
    fota_FirmwareVersion = Read_FOTA_Firmware_Version();
    strcat(fota_LoginPackage, fota_FirmwareVersion);
    
    strcat(fota_LoginPackage, fota_missing22Byte);
    strcat(fota_LoginPackage, fota_Reserved);
    
    strcat(fota_LoginPackage, fota_SerialNumber);
    uint16_t CRC = CalCRC16Hex((uint8_t*)fota_LoginPackage);
    sprintf(fota_Login_Checksum,"%04x",swap16bit(CRC));
    strcat(fota_LoginPackage, fota_Login_Checksum);
    strcat(fota_LoginPackage, fota_EOF);
    free(fota_FirmwareVersion);
    sprintf(Login_Package, "%s",fota_LoginPackage);
    return Login_Package;
}

char *Get_FOTA_Firmware_Message(){
    char *Firmware_Package;
    Firmware_Package = (char*)calloc(40,sizeof(char));
    memset(fota_FirmwarePackage,0,strlen(fota_FirmwarePackage));
    strcat(fota_FirmwarePackage, fota_SOF); 
    strcat(fota_FirmwarePackage, fota_Firmware_DataType);
    strcat(fota_FirmwarePackage, fota_Firmware_Length);
    nwy_get_time(&fota_vnet_time, &fota_timezone);
    sprintf(fota_DateTime, "%02d%02d%02d%02d%02d%02d", fota_vnet_time.day,fota_vnet_time.mon,fota_vnet_time.year-2000, fota_vnet_time.hour,fota_vnet_time.min,fota_vnet_time.sec);
    strcat(fota_FirmwarePackage, fota_DateTime);
    strcat(fota_FirmwarePackage, fota_Code);

    strcat(fota_FirmwarePackage, fota_SerialNumber);
    uint16_t CRC = CalCRC16Hex((uint8_t*)fota_FirmwarePackage);
    sprintf(fota_Firmware_Checksum,"%04x",swap16bit(CRC));
    strcat(fota_FirmwarePackage, fota_Firmware_Checksum);
    strcat(fota_FirmwarePackage, fota_EOF);

    sprintf(Firmware_Package, "%s",fota_FirmwarePackage);
    
    return Firmware_Package;
}

char *Get_FOTA_Package_Message(int fota_ID){
    char *Packet_Package;
    Packet_Package = (char*)calloc(44,sizeof(char));
    memset(fota_PacketPackage, 0, strlen(fota_PacketPackage));
    strcat(fota_PacketPackage, fota_SOF);
    strcat(fota_PacketPackage, fota_Packet_DataType);
    strcat(fota_PacketPackage, fota_Packet_Length);
    nwy_get_time(&fota_vnet_time, &fota_timezone);
    sprintf(fota_DateTime, "%02d%02d%02d%02d%02d%02d", fota_vnet_time.day,fota_vnet_time.mon,fota_vnet_time.year-2000, fota_vnet_time.hour,fota_vnet_time.min,fota_vnet_time.sec);
    strcat(fota_PacketPackage, fota_DateTime);
    strcat(fota_PacketPackage, fota_Code);

    sprintf(fota_Packet_ID, "%04x", swap16bit(fota_ID));
    strcat(fota_PacketPackage, fota_Packet_ID);

    strcat(fota_PacketPackage, fota_SerialNumber);
    uint16_t CRC = CalCRC16Hex((uint8_t*)fota_PacketPackage);
    sprintf(fota_Packet_Checksum,"%04x",swap16bit(CRC));
    strcat(fota_PacketPackage, fota_Packet_Checksum);
    strcat(fota_PacketPackage, fota_EOF);

    sprintf(Packet_Package, "%s",fota_PacketPackage);
    
    return Packet_Package;
}

char * Read_FOTA_Firmware_Version(){
    int i, j;
    char * DataFile;
    DataFile = (char*)calloc(38,sizeof(char));
    char hexData[38]={0};
    for(i=0,j=0;i<strlen(FIRMWARE_VERSION);i++,j+=2){ 
        sprintf((char*)hexData+j,"%02X",FIRMWARE_VERSION[i]);
    }
    sprintf(DataFile, "%s",hexData);
    return DataFile;
}

void nwy_appimg_fota(){
	int fd;
	ota_package_t ota_pack = {0};
	int ota_size = 0;
	int tmp_len = 0;
	int read_len = 0;
	char buff[NWY_APPIMG_FOTA_BLOCK_SIZE] = {0};
	int ret = -1;
	int len = 0;
	char *file_bin_update = "/nwy/VNET_001D.bin";
    ret = nwy_get_fota_result();
    VnET_echo("\r\n Please send %s...",file_bin_update);
    //while(1)
    {
        VnET_echo("\r\n recv %d bytes appimg packet success", fota_package_size);

        if(nwy_sdk_fexist(file_bin_update)){
            ret = nwy_sdk_file_unlink(file_bin_update);
            VnET_echo("\r\n del %s", file_bin_update);
            nwy_sleep(500);
        }
        // if(nwy_sdk_fexist("VNET_001D.bin")){
        //     ret = nwy_sdk_file_unlink("VNET_001D.bin");
        //     VnET_echo("\r\n del %s", "VNET_001D.bin");
        //     nwy_sleep(500);
        // }
        fd = nwy_sdk_fopen(file_bin_update, NWY_CREAT | NWY_RDWR | NWY_TRUNC);

        if (fota_store_firm[0]==0){
            VnET_echo("\r\nFirst byte has been set to zero");
            fota_store_firm[0] = first_byte_hex;
            VnET_echo("\r\nFirst byte from server after reset: %02x | %2x",fota_store_firm[0],first_byte_hex);
            fota_store_firm[0] = first_byte_hex; 
        }

        len = nwy_sdk_fwrite(fd, fota_store_firm, fota_package_size);
        nwy_sdk_fclose(fd);
        if(len != fota_package_size)
            VnET_echo("\r\nfile write: len=%d, return len=%d\r\n",fota_package_size, len);
        else
            VnET_echo("\r\nfile write success and close\r\n");
    }
	fd = nwy_sdk_fopen(file_bin_update, NWY_RDONLY);
	if(fd < 0){
		VnET_echo("\r\nopen appimg fail\r\n");
		return;
	}
	ota_pack.data = (unsigned char *)calloc(NWY_APPIMG_FOTA_BLOCK_SIZE,sizeof(unsigned char));
	ota_pack.len = 0;
	ota_pack.offset = 0;
	if(ota_pack.data == NULL){
		VnET_echo("\r\ncalloc fail\r\n");
		return;
	}
	ota_size = nwy_sdk_fsize_fd(fd);
	VnET_echo("\r\nota_size:%d\r\n", ota_size);
	nwy_sdk_fseek(fd, 0, NWY_SEEK_SET);
	while(ota_size > 0){
		tmp_len = NWY_APPIMG_FOTA_BLOCK_SIZE;
		if(ota_size < tmp_len){
			tmp_len = ota_size;
		}
		read_len = nwy_sdk_fread(fd, buff, tmp_len);
		if(read_len <= 0){
			VnET_echo("\r\nread file error:%d\r\n", read_len);
			free(ota_pack.data);
			nwy_sdk_fclose(fd);
			return;
		}
		VnET_echo("\r\nread len:%d\r\n", read_len);
		memcpy(ota_pack.data, buff, read_len);
		ota_pack.len = read_len;
		ret = nwy_fota_dm(&ota_pack);
		if(ret < 0){
			VnET_echo("\r\nwrite ota error:%d\r\n", ret);
			free(ota_pack.data);
			nwy_sdk_fclose(fd);
			return;
		}
		ota_pack.offset += read_len;
		memset(ota_pack.data, 0, read_len);
		ota_size -= read_len;
	}
	free(ota_pack.data);
	nwy_sdk_fclose(fd);
	VnET_echo("\r\nwrite end\r\n");
	VnET_echo("\r\nstart checksum\r\n");
	ret = nwy_package_checksum();
	if(ret < 0){
		VnET_echo("\r\nchecksum failed\r\n");
		return;
	}
	VnET_echo("\r\nstart update\r\n");
	ret = nwy_fota_ua();
	if(ret < 0){
		VnET_echo("\r\nupdate failed\r\n");
		return;
	}
    else{
        fota_bytes_recv = 0;
        fota_package_size = 0;
        fota_id_package_req = 0;
        fota_id_package_recv = 0;
        fota_number_package = 0;
        VnET_echo("\r\nUpdate completely.\n");
    }
}

char * hexstr_to_char(char* hexstr){
    size_t len = strlen(hexstr);
    size_t final_len = len / 2;
    unsigned char* chrs = (unsigned char*)calloc((final_len+1),sizeof(unsigned char));
    for (size_t i=0, j=0; j<final_len; i+=2, j++)
        chrs[j] = (hexstr[i] % 32 + 9) % 25 * 16 + (hexstr[i+1] % 32 + 9) % 25;
    chrs[final_len] = '\0';
    return chrs;
}

#endif

#if CFG
void check_configs_file(){
    if(nwy_sdk_fexist(path_cfgs_file)){
        VnET_echo("\r\n File configs is available");
    }
    else{
        VnET_echo("\r\n File is not exist");
        if(load_default_cfg2flash()){
            VnET_echo("\r\n Load default configs completely");
            VnET_echo("\r\nREBOOTING...");
            nwy_power_off(2);
        }
        else{
            VnET_echo("\r\n Load default configs error");
        }
    }
}

int isNumber(const char *str){
    while(*str != '\0'){
        if(*str < '0' || *str > '9')
            return 0;
        str++;
    }
    return 1;
}

char * read_configs(){
    char * cfg_content;
    cfg_content = (char*)calloc(512,sizeof(char));
    char * url_tracking;
    char * url_ota;
    char data[32][64];
    char buff_configs[CONFIGS_SIZE];
    int fd = nwy_sdk_fopen(path_cfgs_file, NWY_RDONLY);
    nwy_sdk_fread(fd, buff_configs, CONFIGS_SIZE);
    nwy_sdk_fclose(fd);

    char * part = strtok((char *)buff_configs, "#");
    int u = 0;
    while( part != NULL ) {
        strcpy(data[u],part);
        part = strtok(NULL, "#");
        u += 1;
    }
    url_transform(data[1],URL_CFG2URL,&url_tracking);
    url_transform(data[3],URL_CFG2URL,&url_ota);
    sprintf(cfg_content,
   "\r\n Configure type:  %s \\
    \r\n Tracking url:    %s \\
    \r\n Tracking port:   %d \\
    \r\n FOTA url:        %s \\
    \r\n FOTA port:       %d \\
    \r\n Host number:     %s \\
    \r\n Timezone:        %s \\
    \r\n NTP server:      %s \\
    \r\n APN:             %s \\
    \r\n APN name:        %s \\
    \r\n APN user:        %s \\
    \r\n APN password:    %s \\
    \r\n IMEI:            %s \\
    \r\n Reboot time:     %s",data[0],url_tracking,atoi(data[2]),url_ota,atoi(data[4]),data[5],data[6],data[7],data[8],APN_name,APN_user,APN_pass,data[9],data[10]); 
    VnET_echo("%s",cfg_content);
    return cfg_content;
}

void url_transform(char * url, int mode, char ** url_output){
    char * url_pre = strtok(url, "*");
    int strlen_url = strlen(url_pre);
    char data[4][6];
    char * part = strtok(url_pre, ".");
    int u = 0;
    char url_out[24];
    while( part != NULL ) {
        strcpy(data[u],part);
        part = strtok(NULL, ".");
        u += 1;
    }
    
    if(isNumber(data[0])){
        if (mode==URL_CFG2URL){
            sprintf(url_out, "%d.%d.%d.%d",atoi(data[0]),atoi(data[1]),atoi(data[2]),atoi(data[3]));
        }
        else if (mode==URL_URL2CFG){
            sprintf(url_out, "%03d.%03d.%03d.%03d",atoi(data[0]),atoi(data[1]),atoi(data[2]),atoi(data[3]));
        }
    }
    else{
        if (mode==URL_CFG2URL){
            sprintf(url_out, "%s.%s.%s",data[0],data[1],data[2]);
        }
        else if (mode==URL_URL2CFG){
            sprintf(url_out, "%s.%s.%s%.*s",data[0],data[1],data[2],(15-strlen_url),"*******");
        }
    }
    *url_output = strdup(url_out);
}

void update_configs(){
    char data[32][64];
    char buff_configs[CONFIGS_SIZE];
    char buff_modify[CONFIGS_SIZE];
    char TminRead[4], TmaxRead[4],TminRaw[6],TmaxRaw[6];
    char motionRaw[11];
    char activeMotionRaw[2];
    char sensitiveMotionRaw[6];
    char alarm01Raw[27];
    char alarm02Raw[27];
    char alarm03Raw[27];
    char alarm05Raw[27];
    int fd = nwy_sdk_fopen(path_cfgs_file, NWY_RDONLY);
    nwy_sdk_fread(fd, buff_configs, CONFIGS_SIZE);
    nwy_sdk_fclose(fd);
    strcpy(buff_modify,buff_configs);
    VnET_echo("\r\n[CFG] Data configure: %s",buff_configs);
    int lenRawBuffer = strlen(buff_configs);

    char * part = strtok((char *)buff_configs, "#");
    int u = 0;
    while( part != NULL ) {
        strcpy(data[u],part);
        part = strtok(NULL, "#");
        u += 1;
    }

    strcpy(config_type,data[0]);
    url_transform(data[1],URL_CFG2URL,&tracking_url_cfg);
    strcpy(tracking_port_cfg,data[2]);
    url_transform(data[3],URL_CFG2URL,&fota_url_cfg);
    strcpy(fota_port_cfg,data[4]);
    strcpy(host_number,data[5]);
    strcpy(timezone,data[6]);
    strcpy(ntpserver,data[7]);
    strcpy(APN,data[8]);
    APN_update(APN);
    IMEI_NUM = strdup((char *)data[9]);
    strcpy(reboot_time,data[10]);
    memcpy( hour_reboot,   &reboot_time[0], 2 );    hour_reboot[2]= '\0';
    memcpy( minute_reboot, &reboot_time[2], 2 );    minute_reboot[2]= '\0';
    PowerSaving = strdup((char *)data[11]);
    voltageThres = strdup((char *)data[12]);
    
    strcpy(TminRaw,data[13]);
    memcpy( TminRead,   &TminRaw[2], 3 );    TminRead[3]= '\0';
    Tmin = atoi(TminRead);

    strcpy(TmaxRaw,data[14]);
    memcpy( TmaxRead,   &TmaxRaw[2], 3 );    TmaxRead[3]= '\0';
    Tmax = atoi(TmaxRead);

    strcpy(motionRaw,data[15]);
    memcpy(activeMotionRaw,   &motionRaw[3], 1 );    activeMotionRaw[1]= '\0';
    motionActive = atoi(activeMotionRaw);

    memcpy(sensitiveMotionRaw,&motionRaw[5], 5 );    sensitiveMotionRaw[5]= '\0';
    motionSensitive = atoff(sensitiveMotionRaw);

    strcpy(alarm01Raw,data[16]);    
    memcpy(alarm01.relayChannel,   &alarm01Raw[8], 1 );    alarm01.relayChannel[1]  = '\0';
    memcpy(alarm01.ctrlType,       &alarm01Raw[10], 1 );   alarm01.ctrlType[1]      = '\0';
    memcpy(alarm01.operationMode,  &alarm01Raw[12], 1 );   alarm01.operationMode[1] = '\0';
    memcpy(alarm01.informType,     &alarm01Raw[14], 1 );   alarm01.informType[1]    = '\0';
    memcpy(alarm01.phoneNumber,    &alarm01Raw[16], 10 );  alarm01.phoneNumber[10]  = '\0';

    VnET_echo("\r\n[ALARM01] relayChannel:  %s",alarm01.relayChannel);
    VnET_echo("\r\n[ALARM01] ctrlType:      %s",alarm01.ctrlType);
    VnET_echo("\r\n[ALARM01] operationMode: %s",alarm01.operationMode);
    VnET_echo("\r\n[ALARM01] informType:    %s",alarm01.informType);
    VnET_echo("\r\n[ALARM01] phoneNumber:   %s",alarm01.phoneNumber);

    strcpy(alarm02Raw,data[17]);
    memcpy(alarm02.relayChannel,   &alarm02Raw[8], 1 );    alarm02.relayChannel[1]  = '\0';
    memcpy(alarm02.ctrlType,       &alarm02Raw[10], 1 );   alarm02.ctrlType[1]      = '\0';
    memcpy(alarm02.operationMode,  &alarm02Raw[12], 1 );   alarm02.operationMode[1] = '\0';
    memcpy(alarm02.informType,     &alarm02Raw[14], 1 );   alarm02.informType[1]    = '\0';
    memcpy(alarm02.phoneNumber,    &alarm02Raw[16], 10 );  alarm02.phoneNumber[10]  = '\0';

    VnET_echo("\r\n[ALARM02] relayChannel:  %s",alarm02.relayChannel);
    VnET_echo("\r\n[ALARM02] ctrlType:      %s",alarm02.ctrlType);
    VnET_echo("\r\n[ALARM02] operationMode: %s",alarm02.operationMode);
    VnET_echo("\r\n[ALARM02] informType:    %s",alarm02.informType);
    VnET_echo("\r\n[ALARM02] phoneNumber:   %s",alarm02.phoneNumber);

    strcpy(alarm03Raw,data[18]);
    memcpy(alarm03.relayChannel,   &alarm03Raw[8], 1 );    alarm03.relayChannel[1]  = '\0';
    memcpy(alarm03.ctrlType,       &alarm03Raw[10], 1 );   alarm03.ctrlType[1]      = '\0';
    memcpy(alarm03.operationMode,  &alarm03Raw[12], 1 );   alarm03.operationMode[1] = '\0';
    memcpy(alarm03.informType,     &alarm03Raw[14], 1 );   alarm03.informType[1]    = '\0';
    memcpy(alarm03.phoneNumber,    &alarm03Raw[16], 10 );  alarm03.phoneNumber[10]  = '\0';

    VnET_echo("\r\n[ALARM03] relayChannel:  %s",alarm03.relayChannel);
    VnET_echo("\r\n[ALARM03] ctrlType:      %s",alarm03.ctrlType);
    VnET_echo("\r\n[ALARM03] operationMode: %s",alarm03.operationMode);
    VnET_echo("\r\n[ALARM03] informType:    %s",alarm03.informType);
    VnET_echo("\r\n[ALARM03] phoneNumber:   %s",alarm03.phoneNumber);

    strcpy(alarm05Raw,data[19]);
    memcpy(alarm05.relayChannel,   &alarm05Raw[8], 1 );    alarm05.relayChannel[1]  = '\0';
    memcpy(alarm05.ctrlType,       &alarm05Raw[10], 1 );   alarm05.ctrlType[1]      = '\0';
    memcpy(alarm05.operationMode,  &alarm05Raw[12], 1 );   alarm05.operationMode[1] = '\0';
    memcpy(alarm05.informType,     &alarm05Raw[14], 1 );   alarm05.informType[1]    = '\0';
    memcpy(alarm05.phoneNumber,    &alarm05Raw[16], 10 );  alarm05.phoneNumber[10]  = '\0';

    VnET_echo("\r\n[ALARM05] relayChannel:  %s",alarm05.relayChannel);
    VnET_echo("\r\n[ALARM05] ctrlType:      %s",alarm05.ctrlType);
    VnET_echo("\r\n[ALARM05] operationMode: %s",alarm05.operationMode);
    VnET_echo("\r\n[ALARM05] informType:    %s",alarm05.informType);
    VnET_echo("\r\n[ALARM05] phoneNumber:   %s",alarm05.phoneNumber);
    
    // Always in the top
    nwy_get_time(&vnet_time, &fota_timezone);
    if ((atoi(config_type)==0)&&(vnet_time.year >= 2021)){
        char updateTime[36];
        buff_modify[0] = '1';
        sprintf(updateTime, "%02d/%02d/%04d|%02d:%02d:%02d#", 
                vnet_time.day,vnet_time.mon,vnet_time.year, vnet_time.hour,vnet_time.min,vnet_time.sec);
        strcat(buff_modify,updateTime);
        VnET_echo("\r\nUpdate firmware    : %s | Len: %ld",updateTime,strlen(updateTime));
        VnET_echo("\r\nBuffer after modify: %s | Len: %ld",buff_modify,strlen(buff_modify));
        if ((strlen(buff_modify)-strlen(updateTime))==lenRawBuffer){
            int fd = nwy_sdk_fopen(path_cfgs_file, NWY_CREAT | NWY_RDWR | NWY_TRUNC);
            nwy_sdk_fwrite(fd, buff_modify, CONFIGS_SIZE);
            nwy_sdk_fclose(fd);
            VnET_echo("\r\n Saved update firmware time completely");
            strcpy(updateFirmwareTime,updateTime);
        }
    }
    else if (atoi(config_type)==1){
        strcpy(updateFirmwareTime,data[20]);
    }

    VnET_echo( "\r\n Configure type:  %s \\
                \r\n Tracking url:    %s \\
                \r\n Tracking port:   %d \\
                \r\n FOTA url:        %s \\
                \r\n FOTA port:       %d \\
                \r\n Host number:     %s \\
                \r\n Timezone:        %s \\
                \r\n NTP server:      %s \\
                \r\n APN:             %s \\
                \r\n APN name:        %s \\
                \r\n APN user:        %s \\
                \r\n APN password:    %s \\
                \r\n IMEI:            %.15s \\
                \r\n Reboot time:     %s \\
                \r\n Power Mode :     %s \\
                \r\n Voltage Thres:   %s \\
                \r\n Timer min:       %d \\
                \r\n Timer max:       %d \\
                \r\n Motion:          %d \\
                \r\n Motion Sensitive:%.03f",config_type,tracking_url_cfg,atoi(tracking_port_cfg),fota_url_cfg,atoi(fota_port_cfg),host_number,
                timezone,ntpserver,APN,APN_name,APN_user,APN_pass,IMEI_NUM,reboot_time,PowerSaving,voltageThres,Tmin,Tmax,motionActive,motionSensitive);
}

int load_default_cfg2flash(){
    char data_configs[CONFIGS_SIZE];
    char default_cfg[] = "0";                       // 0-default | 1-customize
    char ip_server_tracking[] = "125.212.203.114";
    char port_server_tracking[] = "16060";
    char ip_server_fota[] = "vxyz.ddns.net**";
    char port_server_fota[] = "08886";
    char host_number_dft[] = "+84123456789";
    char timezone[] = "+7.0";
    char ntpserver[] = "vn.pool.ntp.org";
    char APN[]="VTEL";
    char reboot_time_default[] = "0000";
    char powerModeDefault[] = "PS1";
    char voltageThresDefault[] = "11.60";
    char TminDefault[] = "Ts005";
    char TmaxDefault[] = "Tx030";
    char motionDefault[] = "Mot1,0.125";
    int total_saved = 0;
    int total_executed = 0;
    int ret_write = 0;
    char alarm01Default[] = "ALARM01,0,0,1,1,0123456789";
    char alarm02Default[] = "ALARM02,0,0,0,1,0123456789";
    char alarm03Default[] = "ALARM03,1,1,1,0,0123456789";
    char alarm05Default[] = "ALARM05,x,x,1,0,0123456789";
    int fd = nwy_sdk_fopen(path_cfgs_file, NWY_CREAT | NWY_RDWR | NWY_TRUNC);

    memset ((void*)&data_configs[CONFIGS_TYPE_POS], '\0', 2);
    total_executed++;
    ret_write = sprintf((void*)&data_configs[CONFIGS_TYPE_POS], "%s#", default_cfg);
    if (ret_write==2){
        VnET_echo("\r\n Saved configs type completely");
        total_saved++;
    }
    else{
        VnET_echo("\r\n Saved configs type error");
    }

    memset ((void*)&data_configs[CONFIGS_IP_GPS_POS], '\0', 16);
    total_executed++;
    ret_write = sprintf((void*)&data_configs[CONFIGS_IP_GPS_POS], "%s#", ip_server_tracking);
    if (ret_write==16){
        VnET_echo("\r\n Saved IP tracking completely");
        total_saved++;
    }
    else{
        VnET_echo("\r\n Saved IP tracking error");
    }

    memset ((void*)&data_configs[CONFIGS_PORT_GPS_POS], '\0', 6);
    total_executed++;
    ret_write = sprintf((void*)&data_configs[CONFIGS_PORT_GPS_POS], "%s#", port_server_tracking);
    if (ret_write==6){
        VnET_echo("\r\n Saved port tracking completely");
        total_saved++;
    }
    else{
        VnET_echo("\r\n Saved port tracking error");
    }

    memset ((void*)&data_configs[CONFIGS_IP_FOTA_POS], '\0', 16);
    total_executed++;
    ret_write = sprintf((void*)&data_configs[CONFIGS_IP_FOTA_POS], "%s#", ip_server_fota);
    if (ret_write==16){
        VnET_echo("\r\n Saved IP fota completely");
        total_saved++;
    }
    else{
        VnET_echo("\r\n Saved IP fota error");
    }

    memset ((void*)&data_configs[CONFIGS_PORT_FOTA_POS], '\0', 6);
    total_executed++;
    ret_write = sprintf((void*)&data_configs[CONFIGS_PORT_FOTA_POS], "%s#", port_server_fota);
    if (ret_write==6){
        VnET_echo("\r\n Saved port fota completely");
        total_saved++;
    }
    else{
        VnET_echo("\r\n Saved port fota error");
    }

    memset ((void*)&data_configs[CONFIGS_HOST_NUMBER_POS], '\0', 13);
    total_executed++;
    ret_write = sprintf((void*)&data_configs[CONFIGS_HOST_NUMBER_POS], "%s#", host_number_dft);
    if (ret_write==13){
        VnET_echo("\r\n Saved host number completely");
        total_saved++;
    }
    else{
        VnET_echo("\r\n Saved host number error");
    }

    memset ((void*)&data_configs[CONFIGS_TIMEZONE_POS], '\0', 5);
    total_executed++;
    ret_write = sprintf((void*)&data_configs[CONFIGS_TIMEZONE_POS], "%s#", timezone);
    if (ret_write==5){
        VnET_echo("\r\n Saved timezone completely");
        total_saved++;
    }
    else{
        VnET_echo("\r\n Saved timezone error");
    }

    memset ((void*)&data_configs[CONFIGS_NTP_POS], '\0', 16);
    total_executed++;
    ret_write = sprintf((void*)&data_configs[CONFIGS_NTP_POS], "%s#", ntpserver);
    if (ret_write==16){
        VnET_echo("\r\n Saved NTP completely");
        total_saved++;
    }
    else{
        VnET_echo("\r\n Saved NTP error");
    }

    memset ((void*)&data_configs[CONFIGS_APN_POS], '\0', 5);
    total_executed++;
    ret_write = sprintf((void*)&data_configs[CONFIGS_APN_POS], "%s#", APN);
    if (ret_write ==5){
        VnET_echo("\r\n Saved APN completely");
        total_saved++;
    }
    else{
        VnET_echo("\r\n Saved APN error");
    }

    memset ((void*)&data_configs[CONFIGS_IMEI_POS], '\0', 17);
    total_executed++;	
    getImei(&IMEI_NUM);	
    ret_write = sprintf((void*)&data_configs[CONFIGS_IMEI_POS], "%s#", IMEI_NUM);	
    if (ret_write ==17){
        VnET_echo("\r\n Saved IMEI completely");	
        total_saved++;	
    }	
    else{	
        VnET_echo("\r\n Saved IMEI error");	
    }

    memset ((void*)&data_configs[CONFIGS_REBOOT_POS], '\0', 5);
    total_executed++;	
    ret_write = sprintf((void*)&data_configs[CONFIGS_REBOOT_POS], "%s#", reboot_time_default);	
    if (ret_write ==5){
        VnET_echo("\r\n Saved reboot time completely");	
        total_saved++;	
    }	
    else{	
        VnET_echo("\r\n Saved reboot time error");	
    }

    memset ((void*)&data_configs[CONFIGS_POWER_MODE], '\0', 4);
    total_executed++;	
    ret_write = sprintf((void*)&data_configs[CONFIGS_POWER_MODE], "%s#", powerModeDefault);	
    if (ret_write ==4){
        VnET_echo("\r\n Saved power mode completely");	
        total_saved++;	
    }	
    else{	
        VnET_echo("\r\n Saved power mode error");	
    }

    memset ((void*)&data_configs[CONFIGS_VOLTAGE_THRES], '\0', 6);
    total_executed++;	
    ret_write = sprintf((void*)&data_configs[CONFIGS_VOLTAGE_THRES], "%s#", voltageThresDefault);	
    if (ret_write ==6){
        VnET_echo("\r\n Saved voltage threshold completely");	
        total_saved++;	
    }	
    else{	
        VnET_echo("\r\n Saved voltage threshold error");	
    }

    memset ((void*)&data_configs[CONFIGS_TIMER_MIN], '\0', 6);
    total_executed++;	
    ret_write = sprintf((void*)&data_configs[CONFIGS_TIMER_MIN], "%s#", TminDefault);	
    if (ret_write ==6){
        VnET_echo("\r\n Saved timer min completely");	
        total_saved++;	
    }	
    else{	
        VnET_echo("\r\n Saved timer min error");	
    }
    
    memset ((void*)&data_configs[CONFIGS_TIMER_MAX], '\0', 6);
    total_executed++;	
    ret_write = sprintf((void*)&data_configs[CONFIGS_TIMER_MAX], "%s#", TmaxDefault);	
    if (ret_write ==6){
        VnET_echo("\r\n Saved timer max completely");	
        total_saved++;	
    }	
    else{	
        VnET_echo("\r\n Saved timer max error");	
    }

    memset ((void*)&data_configs[CONFIGS_MOTION], '\0', 11);
    total_executed++;	
    ret_write = sprintf((void*)&data_configs[CONFIGS_MOTION], "%s#", motionDefault);	
    if (ret_write ==11){
        VnET_echo("\r\n Saved motion configure completely");	
        total_saved++;	
    }	
    else{	
        VnET_echo("\r\n Saved motion configure error");	
    }

    memset ((void*)&data_configs[CONFIGS_ALARM_01], '\0', 27);
    total_executed++;	
    ret_write = sprintf((void*)&data_configs[CONFIGS_ALARM_01], "%s#", alarm01Default);	
    if (ret_write ==27){
        VnET_echo("\r\n Saved alarm01 configure completely");	
        total_saved++;	
    }	
    else{	
        VnET_echo("\r\n Saved alarm01 configure error");	
    }

    memset ((void*)&data_configs[CONFIGS_ALARM_02], '\0', 27);
    total_executed++;	
    ret_write = sprintf((void*)&data_configs[CONFIGS_ALARM_02], "%s#", alarm02Default);	
    if (ret_write ==27){
        VnET_echo("\r\n Saved alarm02 configure completely");	
        total_saved++;	
    }	
    else{	
        VnET_echo("\r\n Saved alarm02 configure error");	
    }

    memset ((void*)&data_configs[CONFIGS_ALARM_03], '\0', 27);
    total_executed++;	
    ret_write = sprintf((void*)&data_configs[CONFIGS_ALARM_03], "%s#", alarm03Default);	
    if (ret_write ==27){
        VnET_echo("\r\n Saved alarm03 configure completely");	
        total_saved++;	
    }	
    else{	
        VnET_echo("\r\n Saved alarm03 configure error");	
    }

    memset ((void*)&data_configs[CONFIGS_ALARM_05], '\0', 27);
    total_executed++;	
    ret_write = sprintf((void*)&data_configs[CONFIGS_ALARM_05], "%s#", alarm05Default);	
    if (ret_write ==27){
        VnET_echo("\r\n Saved alarm05 configure completely");	
        total_saved++;	
    }	
    else{	
        VnET_echo("\r\n Saved alarm05 configure error");	
    }

    int len_write = nwy_sdk_fwrite(fd, data_configs, CONFIGS_SIZE);
    VnET_echo("\r\n len_write; %d",len_write);
    nwy_sdk_fclose(fd);
    
    if (total_saved==total_executed){
        VnET_echo("\r\n Wrote all done %d/%d",total_saved,total_executed);
        return 1;
    }
    else{
        VnET_echo("\r\n Wrote error");
        return 0;
    }
}

int delete_configs(){
    int ret = nwy_sdk_file_unlink(path_cfgs_file);
    if (ret == NWY_SUCESS){
        VnET_echo("\r\n Delete file successfully");
    }
    else{
        VnET_echo("\r\n Delete file failed");
    }
}

int check_GPS(){
    if(strncmp(GPRMC.Flag,"A",1)==0){
        return 1;
    }
    else{
        return 0;
    }
}

int check_GSM(){
    return call_PPP;
}

int check_Server(){
    return recv_login_package;
}
	
void getImei(char ** IMEI_){	
    uint8_t imei[17] = {0};	
    nwy_sim_result_type sim_info;	
    memset(&sim_info, 0, sizeof(sim_info));	
    nwy_sim_get_imei(&sim_info);	
    memcpy(imei, sim_info.nImei, strlen(sim_info.nImei));	
    imei[strlen(sim_info.nImei)]='0';	
    imei[strlen(sim_info.nImei)+1]='\0';	
    VnET_echo("\r\nIMEI: %s",imei);	
    * IMEI_ = strdup((char *)imei);		
}

void read_Battery(char ** Voltage_input){
    int value = -1;
    char template[6];
    int channel = 3; //2-CHANNEL2,3-CHANNEL3,4-VBAT
    int scale = 0;   //ADC scale(0-1V250,1-2V444,2-3V233,3-5V000)
    value = nwy_adc_get(channel,scale);
    sprintf(template,"%d.%d",(value*52+VOLTAGE_BIAS)/1000,(value*52+VOLTAGE_BIAS)%1000);  //R_up = 510k | R_down = 10k
    *Voltage_input = strdup(template);
}

int findSpecialChar(unsigned char *buff, int strLength, char c){
  int i = 0;
  for (i = 0; (i < strLength); i++){
    if (buff[i] == c) return (i+1);
    if (buff[i] == 0) break;
  }
  return 0;
}

int findTotalSpecialChar(unsigned char *buff, unsigned char leng, char c){
  unsigned char i = 0, j = 0;
  if (leng > 255) return 0;
  for (i = 0; i < leng; i++){
    if (buff[i] == c) j++;
  }
  return j;
}

//          http://maps.google.com/maps?q= N %20 21 %20 2.9235 %20 E %20 105 %20 44.4293
int GetaLinkOfGoogleMap(char **ContentPtr){
    if (strncmp(GPRMC.Flag,"A",1)==0){
        int lati_deg = 0;
        float lati_p = 0.0;
        int long_deg = 0;
        float long_p = 0.0;
        lati_deg = ((int)(GPRMC.Latitude))/100;
        lati_p   = GPRMC.Latitude - lati_deg*100.0;
        long_deg = ((int)(GPRMC.Longitude))/100;
        long_p   = GPRMC.Longitude -long_deg*100.0;
        char ContentBuf[128];
        sprintf (ContentBuf, "http://maps.google.com/maps?q=%s%c20%d%c20%2.4f%c20%s%c20%d%c20%2.4f \n"  , (GPRMC.bit_1) ? "N" : "S", '%'
                                                                                                        , lati_deg, '%'
                                                                                                        , lati_p, '%'
                                                                                                        , (GPRMC.bit_2) ? "E" : "W", '%'
                                                                                                        , long_deg, '%'
                                                                                                        , long_p);
        (*ContentPtr) = strdup(ContentBuf);                                                                                                
        return strlen(ContentBuf);
    }
    else{
        (*ContentPtr) = strdup("GPS have been not latched yet");                                                                                                
        return 0;
    }
    
}

int Response_Command(char * header, int src, char * content_type){
    char content[1024];
    sprintf(content,"%s,%s#",header,content_type);
    switch (src)
    {
    case SRC_UART:
        // UART_CFG.uart_send(UART_CFG.uart_connection,(uint8_t *)content, strlen(content));
        break;
    case SRC_SMS:
        nwy_sleep(3000);
        vnet_send_sms(sender_number,content);
        break;
    case SRC_FOTA:
        // nwy_sleep(1000);
        // VnET_echo("\r\n[FOTA-RESPONSE] %s",content);
        break;
    default:
        break;
    }
}

char* replaceWord(const char* s, const char* oldW,const char* newW){
    char* result;
    int i, cnt = 0;
    int newWlen = strlen(newW);
    int oldWlen = strlen(oldW);
  
    // Counting the number of times old word
    // occur in the string
    for (i = 0; s[i] != '\0'; i++) {
        if (strstr(&s[i], oldW) == &s[i]) {
            cnt++;
  
            // Jumping to index after the old word.
            i += oldWlen - 1;
        }
    }
  
    // Making new string of enough length
    result = (char*)calloc(i + cnt * (newWlen - oldWlen) + 1,sizeof(char));
  
    i = 0;
    while (*s) {
        // compare the substring with the result
        if (strstr(s, oldW) == s) {
            strcpy(&result[i], newW);
            i += newWlen;
            s += oldWlen;
        }
        else
            result[i++] = *s++;
    }
  
    result[i] = '\0';
    return result;
}

int APN_update(char apn[]){
    if(!strncmp(strupr(apn),"VTEL",strlen(apn))){
        strcpy(APN_name,"v-internet");
        strcpy(APN_user,"");
        strcpy(APN_pass,"");
        return 1;
    }
    else if (!strncmp(strupr(apn),"VINA",strlen(apn))){
        strcpy(APN_name,"m3-world");
        strcpy(APN_user,"mms");
        strcpy(APN_pass,"mms");
        return 1;
    }
    else if (!strncmp(strupr(apn),"MOBI",strlen(apn))){
        strcpy(APN_name,"m-wap");
        strcpy(APN_user,"mms");
        strcpy(APN_pass,"mms");
        return 1;
    }
    else{
        return 0;
    }
}

int Center_Handle_Command(unsigned char *cmd, int len, int src ){
    char *ptr0; 
	char *ptr1;
	char *ptr2;
	char *ptr3;
    char *ptr4;
	char *ptr5;
    int comma_number = 0;
    char data_configs[CONFIGS_SIZE];
    int fd = nwy_sdk_fopen(path_cfgs_file, NWY_RDONLY);
    nwy_sdk_fread(fd, data_configs, CONFIGS_SIZE);
    nwy_sdk_fclose(fd);
    comma_number = findTotalSpecialChar(cmd,len-1,',');
    VnET_echo("\r\nNumber of comma: %d",comma_number);
    switch (comma_number)
    {
    case 0:
        ptr0 = strtok(cmd, "#");
        VnET_echo("\r\nCommand header : %s",strupr(ptr0));
        break;
    case 1:
        ptr0 = strtok(cmd, ",");
        ptr1 = strtok(NULL, "#");
        VnET_echo("\r\nCommand header : %s",strupr(ptr0));
        VnET_echo("\r\nArgument-1 : %s",strupr(ptr1));
        break;
    case 2:
        ptr0 = strtok(cmd, ",");
        ptr1 = strtok(NULL, ",");
        ptr2 = strtok(NULL, "#");
        VnET_echo("\r\nCommand header : %s",strupr(ptr0));
        VnET_echo("\r\nArgument-1 : %s",strupr(ptr1));
        VnET_echo("\r\nArgument-2 : %s",strupr(ptr2));
        break;
    case 3:
        ptr0 = strtok(cmd, ",");
        ptr1 = strtok(NULL, ",");
        ptr2 = strtok(NULL, ",");
        ptr3 = strtok(NULL, "#");
        VnET_echo("\r\nCommand header : %s",strupr(ptr0));
        VnET_echo("\r\nArgument-1 : %s",strupr(ptr1));
        VnET_echo("\r\nArgument-2 : %s",strupr(ptr2));
        VnET_echo("\r\nArgument-3 : %s",strupr(ptr3));
    case 5:
        ptr0 = strtok(cmd, ",");
        ptr1 = strtok(NULL, ",");
        ptr2 = strtok(NULL, ",");
        ptr3 = strtok(NULL, ",");
        ptr4 = strtok(NULL, ",");
        ptr5 = strtok(NULL, "#");
        VnET_echo("\r\nCommand header : %s",strupr(ptr0));
        VnET_echo("\r\nArgument-1 : %s",strupr(ptr1));
        VnET_echo("\r\nArgument-2 : %s",strupr(ptr2));
        VnET_echo("\r\nArgument-3 : %s",strupr(ptr3));
        VnET_echo("\r\nArgument-4 : %s",strupr(ptr4));
        VnET_echo("\r\nArgument-5 : %s",strupr(ptr5));
    default:
        break;
    }
    // RESET
    if (((!strncmp(strupr(ptr0),"RESET",strlen(ptr0)))||(!strncmp(strupr(ptr0),"HARDRESET",strlen(ptr0)))||(!strncmp(strupr(ptr0),"REBOOT",strlen(ptr0)))||(!strncmp(strupr(ptr0),"RESTART",strlen(ptr0))))&&(src>0)){
        if (comma_number==0){
            Response_Command(ptr0,src,"OK");
            VnET_echo("\r\nREBOOTING...");
            nwy_power_off(2);
        }
        else if (comma_number==1){
            Response_Command(ptr0,src,reboot_time);
        }
        else if (comma_number==2){
            if((atoi(ptr1) < 24) && (atoi(ptr2) < 60)){
                char reboot_time_cfg[5];
                sprintf(reboot_time_cfg,"%02d%02d",atoi(ptr1),atoi(ptr2));
                VnET_echo("\r\n Data configs before: %s|%ld",data_configs,strlen(data_configs));
                char * result = replaceWord(data_configs, reboot_time, reboot_time_cfg);
                VnET_echo("\r\n Data configs after: %s|%ld",result,strlen(result));
                if (strlen(data_configs)==strlen(result)){
                    VnET_echo("\r\n Saved host number completely");
                    int fd = nwy_sdk_fopen(path_cfgs_file, NWY_CREAT | NWY_RDWR | NWY_TRUNC);
                    nwy_sdk_fwrite(fd, result, CONFIGS_SIZE);
                    nwy_sdk_fclose(fd);
                    sprintf(reboot_time,"%02d%02d",atoi(ptr1),atoi(ptr2)); 
                    memcpy( hour_reboot,   &reboot_time[0], 2 );    hour_reboot[2]= '\0';
                    memcpy( minute_reboot, &reboot_time[2], 2 );    minute_reboot[2]= '\0';
                    Response_Command(ptr0,src,"OK");
                }
                else{
                    Response_Command(ptr0,src,"SAVE ERROR");
                }
            }
            else{
                Response_Command(ptr0,src,"ERR");
            }
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }
    else 
    // RESTORE
    if (((!strncmp(strupr(ptr0),"RESTORE",strlen(ptr0))) || (!strncmp(strupr(ptr0),"DEFAULT",strlen(ptr0))))&&(src>0)){
        if (!comma_number){
            Response_Command(ptr0,src,"OK");
            delete_configs();
            load_default_cfg2flash();
            VnET_echo("\r\nREBOOTING...");
            nwy_power_off(2);
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }
    else 
    // Firmware
    if (((!strncmp(strupr(ptr0),"FW",strlen(ptr0))) || (!strncmp(strupr(ptr0),"FIRMWARE",strlen(ptr0))))&&(src>0)){
        if (comma_number==0){
            Response_Command(ptr0,src,FIRMWARE_VERSION);
        }
        else if (comma_number==1){
            if (!strncmp(strupr(ptr1),"TIME",strlen(ptr1))){
                Response_Command(ptr0,src,updateFirmwareTime);
            }   
            else{
                Response_Command(ptr0,src,"ERR");
            }
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }
    else 
    // SERVER
    if ((!strncmp(strupr(ptr0),"SERVER",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            char template[32];
            sprintf(template,"%s,%d",tracking_url_cfg,atoi(tracking_port_cfg));
            Response_Command(ptr0,src,template);
        }
        else if (comma_number==2){
            char url_temp[16];
            strcpy(url_temp,ptr1);
            char old_data[32];
            char * url_tracking_1;
            char * url_tracking_2;
            url_transform(tracking_url_cfg,URL_URL2CFG,&url_tracking_1);
            sprintf(old_data,"%s#%05d",url_tracking_1,atoi(tracking_port_cfg));
            char new_data[32];
            url_transform(ptr1,URL_URL2CFG,&url_tracking_2);
            sprintf(new_data,"%s#%05d",url_tracking_2,atoi(ptr2));
            VnET_echo("\r\n Data configs before: %s|%ld",data_configs,strlen(data_configs));
            char * result = replaceWord(data_configs, old_data, new_data);
            VnET_echo("\r\n Data configs after: %s|%ld",result,strlen(result));
            if (strlen(data_configs)==strlen(result)){
                int fd = nwy_sdk_fopen(path_cfgs_file, NWY_CREAT | NWY_RDWR | NWY_TRUNC);
                nwy_sdk_fwrite(fd, result, CONFIGS_SIZE);
                nwy_sdk_fclose(fd);
                Response_Command(ptr0,src,"OK");
                VnET_echo("\r\n Saved server tracking completely");
                url_transform(url_temp,URL_CFG2URL,&tracking_url_cfg);
                VnET_echo("\r\n tracking_url_cfg=%s",tracking_url_cfg);
                strcpy(tracking_port_cfg,ptr2);
            }
            else{
                Response_Command(ptr0,src,"ERR");
                VnET_echo("\r\n Saved server tracking error");
            }
            free(result);
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }
    else 
    // FOTA
    if ((!strncmp(strupr(ptr0),"FOTA",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            char template[32];
            sprintf(template,"%s,%d",fota_url_cfg,atoi(fota_port_cfg));
            Response_Command(ptr0,src,template);
        }
        else if (comma_number==1){
            if(!strncmp(strupr(ptr1),"UPDATE",strlen(ptr1))){
                Response_Command(ptr0,src,"OK");
            }
            else{
                Response_Command(ptr0,src,"ERR");
            }
        }
        else if (comma_number==2){
            char url_temp[16];
            strcpy(url_temp,ptr1);
            char old_data[32];
            char * url_fota_1;
            char * url_fota_2;
            url_transform(fota_url_cfg,URL_URL2CFG,&url_fota_1);
            sprintf(old_data,"%s#%05d",url_fota_1,atoi(fota_port_cfg));
            char new_data[32];
            url_transform(ptr1,URL_URL2CFG,&url_fota_2);
            sprintf(new_data,"%s#%05d",url_fota_2,atoi(ptr2));
            VnET_echo("\r\n Data configs before: %s|%ld",data_configs,strlen(data_configs));
            char * result = replaceWord(data_configs, old_data, new_data);
            VnET_echo("\r\n Data configs after: %s|%ld",result,strlen(result));
            if (strlen(data_configs)==strlen(result)){
                int fd = nwy_sdk_fopen(path_cfgs_file, NWY_CREAT | NWY_RDWR | NWY_TRUNC);
                nwy_sdk_fwrite(fd, result, CONFIGS_SIZE);
                nwy_sdk_fclose(fd);
                Response_Command(ptr0,src,"OK");
                VnET_echo("\r\n Saved server fota completely");
                url_transform(url_temp,URL_CFG2URL,&fota_url_cfg);
                VnET_echo("\r\n fota_url_cfg=%s",fota_url_cfg);
                strcpy(fota_port_cfg,ptr2);
            }
            else{
                Response_Command(ptr0,src,"ERR");
                VnET_echo("\r\n Saved server fota error");
            }
            free(result);
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }
    else 
    // HOST NUMBER
    if ((!strncmp(strupr(ptr0),"HOST",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            Response_Command(ptr0,src,host_number);
        }
        else if (comma_number==1){
            char pNumber[13];
            strcpy(pNumber,ptr1);
            if ((strlen(ptr1)==12) && (!strncmp(pNumber,"+84",3))){
                VnET_echo("\r\n%s is valid phone number");
                goto do_save_phone_number;
            }
            else if ((strlen(ptr1)==10) && (pNumber[0] = '0')){
                VnET_echo("\r\nReformat phone number");
                char tmp[12];
                memcpy(tmp,&pNumber[1],9);
                sprintf(ptr1,"+84%s",tmp);
                VnET_echo("\r\nPhone number then: %s",ptr1);
                goto do_save_phone_number;
            }
            else{
                Response_Command(ptr0,src,"ERR");
                return;
            }
            do_save_phone_number:
                VnET_echo("\r\n Data configs before: %s|%ld",data_configs,strlen(data_configs));
                char * result = replaceWord(data_configs, host_number, ptr1);
                VnET_echo("\r\n Data configs after: %s|%ld",result,strlen(result));
                if (strlen(data_configs)==strlen(result)){
                    VnET_echo("\r\n Saved host number completely");
                    int fd = nwy_sdk_fopen(path_cfgs_file, NWY_CREAT | NWY_RDWR | NWY_TRUNC);
                    nwy_sdk_fwrite(fd, result, CONFIGS_SIZE);
                    nwy_sdk_fclose(fd);
                    strcpy(host_number,ptr1);
                    Response_Command(ptr0,src,"OK");
                }
                else{
                    Response_Command(ptr0,src,"ERR");
                    VnET_echo("\r\n Saved host number error");
                }
                free(result);
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }
    else 
    // DEVICE STATE
    if ((!strncmp(strupr(ptr0),"STATUS",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            char template[32];
            sprintf(template,"GPS:%d,4G:%d,SV:%d",check_GPS(),check_GSM(),check_Server());
            Response_Command(ptr0,src,template);
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }
    else 
    // DEVICE STATE
    if ((!strncmp(strupr(ptr0),"BATTERY",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            read_Battery(&Battery_voltage);
            Response_Command(ptr0,src,Battery_voltage);
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }
    else 
    // Time sync
    if ((!strncmp(strupr(ptr0),"RTCSYNC",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            char template[32];
            sprintf(template,"%s,%s",timezone,ntpserver);
            Response_Command(ptr0,src,template);
        }
        else if (comma_number==1){
            VnET_echo("\r\n Data configs before: %s|%ld",data_configs,strlen(data_configs));
            char * result = replaceWord(data_configs, timezone, ptr1);
            VnET_echo("\r\n Data configs after: %s|%ld",result,strlen(result));
            if (strlen(data_configs)==strlen(result)){
                VnET_echo("\r\n Saved timezone completely");
                int fd = nwy_sdk_fopen(path_cfgs_file, NWY_CREAT | NWY_RDWR | NWY_TRUNC);
                nwy_sdk_fwrite(fd, result, CONFIGS_SIZE);
                nwy_sdk_fclose(fd);
                strcpy(timezone,ptr1);
                Response_Command(ptr0,src,"OK");
            }
            else{
                Response_Command(ptr0,src,"ERR");
                VnET_echo("\r\n Saved timezone error");
            }
            free(result);
        }
        else if (comma_number==2){
            char old_data[32];
            sprintf(old_data,"%s#%s",timezone,ntpserver);
            char new_data[32];
            sprintf(new_data,"%s#%s",ptr1,ptr2);
            VnET_echo("\r\n Data configs before: %s|%ld",data_configs,strlen(data_configs));
            char * result = replaceWord(data_configs, old_data, new_data);
            VnET_echo("\r\n Data configs after: %s|%ld",result,strlen(result));
            if (strlen(data_configs)==strlen(result)){
                int fd = nwy_sdk_fopen(path_cfgs_file, NWY_CREAT | NWY_RDWR | NWY_TRUNC);
                nwy_sdk_fwrite(fd, result, CONFIGS_SIZE);
                nwy_sdk_fclose(fd);
                Response_Command(ptr0,src,"OK");
                VnET_echo("\r\n Saved RTCSYNC completely");
                strcpy(timezone,ptr1);
                strcpy(ntpserver,ptr2);
            }
            else{
                Response_Command(ptr0,src,"ERR");
                VnET_echo("\r\n Saved RTCSYNC error");
            }
            free(result);
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }
    else 
    // APN
    if ((!strncmp(strupr(ptr0),"APN",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            Response_Command(ptr0,src,APN);
        }
        else if (comma_number==1){
            if(!strncmp(strupr(ptr1),"VIETTEL",strlen(ptr1))){
                VnET_echo("\r\n Data configs before: %s|%ld",data_configs,strlen(data_configs));
                char * result = replaceWord(data_configs, APN, "VTEL");
                VnET_echo("\r\n Data configs after: %s|%ld",result,strlen(result));
                if (strlen(data_configs)==strlen(result)){
                    VnET_echo("\r\n Saved APN completely");
                    int fd = nwy_sdk_fopen(path_cfgs_file, NWY_CREAT | NWY_RDWR | NWY_TRUNC);
                    nwy_sdk_fwrite(fd, result, CONFIGS_SIZE);
                    nwy_sdk_fclose(fd);
                    strcpy(APN,"VTEL");
                    Response_Command(ptr0,src,"OK");
                    APN_update(APN);
                }
                else{
                    Response_Command(ptr0,src,"ERR");
                    VnET_echo("\r\n Saved APN error");
                }
                free(result);
            }
            else if ((!strncmp(strupr(ptr1),"VINA",strlen(ptr1)))||((!strncmp(strupr(ptr1),"VINAPHONE",strlen(ptr1))))){
                VnET_echo("\r\n Data configs before: %s|%ld",data_configs,strlen(data_configs));
                char * result = replaceWord(data_configs, APN, "VINA");
                VnET_echo("\r\n Data configs after: %s|%ld",result,strlen(result));
                if (strlen(data_configs)==strlen(result)){
                    VnET_echo("\r\n Saved APN completely");
                    int fd = nwy_sdk_fopen(path_cfgs_file, NWY_CREAT | NWY_RDWR | NWY_TRUNC);
                    nwy_sdk_fwrite(fd, result, CONFIGS_SIZE);
                    nwy_sdk_fclose(fd);
                    strcpy(APN,"VINA");
                    Response_Command(ptr0,src,"OK");
                    APN_update(APN);
                }
                else{
                    Response_Command(ptr0,src,"ERR");
                    VnET_echo("\r\n Saved APN error");
                }
                free(result);
            }
            else if ((!strncmp(strupr(ptr1),"MOBI",strlen(ptr1)))||((!strncmp(strupr(ptr1),"MOBIPHONE",strlen(ptr1))))){
                VnET_echo("\r\n Data configs before: %s|%ld",data_configs,strlen(data_configs));
                char * result = replaceWord(data_configs, APN, "MOBI");
                VnET_echo("\r\n Data configs after: %s|%ld",result,strlen(result));
                if (strlen(data_configs)==strlen(result)){
                    VnET_echo("\r\n Saved APN completely");
                    int fd = nwy_sdk_fopen(path_cfgs_file, NWY_CREAT | NWY_RDWR | NWY_TRUNC);
                    nwy_sdk_fwrite(fd, result, CONFIGS_SIZE);
                    nwy_sdk_fclose(fd);
                    strcpy(APN,"MOBI");
                    Response_Command(ptr0,src,"OK");
                    APN_update(APN);
                }
                else{
                    Response_Command(ptr0,src,"ERR");
                    VnET_echo("\r\n Saved APN error");
                }
                free(result);
            }
            else{
                Response_Command(ptr0,src,"ERR");
            }
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }
    else
    // DEL_CFG
    if ((!strncmp(strupr(ptr0),"DCFG",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            Response_Command(ptr0,src,"OK");
            delete_configs();
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }
    else
    // LOAD_CFG
    if ((!strncmp(strupr(ptr0),"LCFG",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            Response_Command(ptr0,src,"OK");
            load_default_cfg2flash();
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    } 
    else
    // READ_CFG
    if ((!strncmp(strupr(ptr0),"RCFG",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            char *ret_content = read_configs();
            Response_Command(ptr0,src,ret_content);
            free(ret_content);
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }
    else
    // UPDATE_CFG
    if ((!strncmp(strupr(ptr0),"UCFG",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            update_configs();
            Response_Command(ptr0,src,"OK");
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }

    // update_configs();
    else
    // UPTIME
    if ((!strncmp(strupr(ptr0),"UPTIME",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            uptime_ms = nwy_get_ms();
            VnET_echo("\r\nUptime: %ld s",(long) uptime_ms / 1000);
            char uptime_sec[12] = {0};
            sprintf(uptime_sec,"%ldsec",(long) uptime_ms / 1000);
            Response_Command(ptr0,src,uptime_sec);
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }

    else
    // IMEI
    if ((!strncmp(strupr(ptr0),"IMEI",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            if (strlen(IMEI_NUM)!=16){
                getImei(&IMEI_NUM);
            }
            Response_Command(ptr0,src,IMEI_NUM);
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }

    else
    // DATETIME
    if ((!strncmp(strupr(ptr0),"DATETIME",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            char template[36];
            nwy_get_time(&vnet_time, &fota_timezone);
            sprintf(template, "%02d.%02d.%04d %02d:%02d:%02d", vnet_time.day,vnet_time.mon,vnet_time.year, vnet_time.hour,vnet_time.min,vnet_time.sec);
            Response_Command(ptr0,src,template);
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }

    else
    // ACC
    if ((!strncmp(strupr(ptr0),"ACC",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            if (accValue == nwy_low){
                Response_Command(ptr0,src,"ON");
            }
            else{
                Response_Command(ptr0,src,"OFF");
            }
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }

    else
    // MOTION
    if ((!strncmp(strupr(ptr0),"MOTION",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            char template[8];
            sprintf(template,"%d,%.03f",motionActive,motionSensitive);
            Response_Command(ptr0,src,template);
        }
        else if (comma_number==2){
            if (((atoi(ptr1)==1)||(atoi(ptr1)==0))&&(atoff(ptr2)>0.0)&&(atoff(ptr2)<=3.0)){
                char prevPart[12];
                sprintf(prevPart,"Mot%d,%.03f",motionActive,motionSensitive);
                char cfgPart[12];
                sprintf( cfgPart,"Mot%d,%.03f",atoi(ptr1),atoff(ptr2));

                VnET_echo("\r\n Data configs before: %s|%ld",data_configs,strlen(data_configs));
                char * result = replaceWord(data_configs, prevPart, cfgPart);
                VnET_echo("\r\n Data configs after: %s|%ld",result,strlen(result));
                if (strlen(data_configs)==strlen(result)){
                    VnET_echo("\r\n Saved motion configure completely");
                    int fd = nwy_sdk_fopen(path_cfgs_file, NWY_CREAT | NWY_RDWR | NWY_TRUNC);
                    nwy_sdk_fwrite(fd, result, CONFIGS_SIZE);
                    nwy_sdk_fclose(fd);
                    Response_Command(ptr0,src,"OK");
                }
                else{
                    VnET_echo("\r\n Save motion configure fail, writing data length is not equal data saving length");
                    Response_Command(ptr0,src,"ERR");
                }

                motionActive = atoi(ptr1);
                motionSensitive = atoff(ptr2);
            }
            else{
                Response_Command(ptr0,src,"ERR, 0.0 < Sensitive <= 3.0");
            }
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }

    else
    // Overview
    if ((!strncmp(strupr(ptr0),"OVERVIEW",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            char contentResponse[384];
            nwy_get_time(&vnet_time, &fota_timezone);
            if (sleepMode==0){  // SLEEP
                uptime_ms = nwy_get_ms();
                sleepingTimeM = sleepingTimeM + (uptime_ms-prevTime)/60000;
                prevTime = uptime_ms;
            }
            else{               // WORK
                uptime_ms = nwy_get_ms();
                workingTimeM = workingTimeM + (uptime_ms-prevTime)/60000;
                prevTime = uptime_ms;
            }
            sprintf(contentResponse,"%d:%d:%d %d/%d/%d \\
                                    \r\n%s\\
                                    \r\n%.15s\\
                                    \r\nGSM:%d/99\\
                                    \r\nGPS:%s/%s/%d/%d\\
                                    \r\nI/R:%s/1-1\\
                                    \r\nPwr: %s,%s,%d,%d,%d,%d\\
                                    \r\n%s,%s"
                                    ,vnet_time.hour,vnet_time.min,vnet_time.sec,vnet_time.day,vnet_time.mon,vnet_time.year,
                                    FIRMWARE_VERSION,
                                    IMEI_NUM,
                                    rssi,
                                    (INIT_GPS) ? "Nor" : "Stnd",(strncmp(GPRMC.Flag,"A",1)==0) ? "1" : "0",NoOfSatellite,GPRMC.Speed,
                                    (accValue) ? "1" : "0",
                                    (average_voltage() < 5) ? "1" : "0",(!strncmp(strupr(PowerSaving),"PS1",strlen(PowerSaving))) ? "1" : "0",chargerState,sleepMode,sleepingTimeM,workingTimeM,
                                    tracking_url_cfg,tracking_port_cfg);
            Response_Command(ptr0,src,contentResponse);
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }

    else
    // FINDER
    if ((!strncmp(strupr(ptr0),"FINDER",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            if (atoi(alarm02.informType)==0){
                call_alert = 1;
                Response_Command(ptr0,src,"OK");
            }
            else{
                Response_Command(ptr0,src,"To find vehicle by SMS. Please set ALARM02->INFORM_TYPE is 1");
            }
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }

    else
    // URL
    if ((!strncmp(strupr(ptr0),"URL",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            char * gpsLink;
            GetaLinkOfGoogleMap(&gpsLink);
            VnET_echo("\r\nLink get: %s",gpsLink);
            Response_Command(ptr0,src,gpsLink);
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }

    else
    // CHARGER
    if ((!strncmp(strupr(ptr0),"CHARGER",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            Response_Command(ptr0,src,(chargerState) ? "CHARGING" : "NO CHARGING");
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }

    else
    // CHARGER
    if ((!strncmp(strupr(ptr0),"HANGUP",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            VnET_echo("\r\nTao dang vao hangup");
            while(1)
            {
                ;
            }
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }

    else
    // ALARM01
    if ((!strncmp(strupr(ptr0),"ALARM01",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            char template[36];
            sprintf(template,"%s,%s,%s,%s,%s",alarm01.relayChannel,alarm01.ctrlType,alarm01.operationMode,alarm01.informType,alarm01.phoneNumber);
            Response_Command(ptr0,src,template);
        }
        else if (comma_number==5){
            if(checkAlarmMessage(ptr1,ptr2,ptr3,ptr4,ptr5)){
                char prevPart[42];
                sprintf(prevPart,"ALARM01,%s,%s,%s,%s,%s",alarm01.relayChannel,alarm01.ctrlType,alarm01.operationMode,alarm01.informType,alarm01.phoneNumber);
                char cfgPart[42];
                sprintf( cfgPart,"ALARM01,%s,%s,%s,%s,%s",ptr1,ptr2,ptr3,ptr4,ptr5);
                VnET_echo("\r\n Data configs before: %s|%ld",data_configs,strlen(data_configs));
                char * result = replaceWord(data_configs, prevPart, cfgPart);
                VnET_echo("\r\n Data configs after: %s|%ld",result,strlen(result));
                if (strlen(data_configs)==strlen(result)){
                    VnET_echo("\r\n Saved alarm01 configure completely");
                    int fd = nwy_sdk_fopen(path_cfgs_file, NWY_CREAT | NWY_RDWR | NWY_TRUNC);
                    nwy_sdk_fwrite(fd, result, CONFIGS_SIZE);
                    nwy_sdk_fclose(fd);
                    Response_Command(ptr0,src,"OK");
                }
                else{
                    VnET_echo("\r\n Save alarm01 configure fail, writing data length is not equal data saving length");
                    Response_Command(ptr0,src,"ERR");
                }
                strcpy(alarm01.relayChannel,ptr1);
                strcpy(alarm01.ctrlType,ptr2);
                strcpy(alarm01.operationMode,ptr3);
                strcpy(alarm01.informType,ptr4);
                strcpy(alarm01.phoneNumber,ptr5);
            }
            else{
                Response_Command(ptr0,src,"SYNTAX ERROR");
            }
        } 
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }

    else
    // MOTIONALARM
    if ((!strncmp(strupr(ptr0),"MOTIONALARM",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            Response_Command(ptr0,src,(motionAlarm) ? "ON" : "OFF");
        }
        else if (comma_number==1){
            if (!alarm01.operationMode){
                if (atoi(ptr1)==1){
                    motionAlarm = 1;
                    Response_Command(ptr0,src,"OK");
                }
                else if (atoi(ptr1)==0){
                    motionAlarm = 0;
                    Response_Command(ptr0,src,"OK");
                }
                else{
                    Response_Command(ptr0,src,"ERR");
                }
            }
            else{
                Response_Command(ptr0,src,"Device is in auto mode");
            }
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }

    else
    // ALARM02
    if ((!strncmp(strupr(ptr0),"ALARM02",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            char template[36];
            sprintf(template,"%s,%s,%s,%s,%s",alarm02.relayChannel,alarm02.ctrlType,alarm02.operationMode,alarm02.informType,alarm02.phoneNumber);
            Response_Command(ptr0,src,template);
        }
        else if (comma_number==5){
            if(checkAlarmMessage(ptr1,ptr2,ptr3,ptr4,ptr5)){
                char prevPart[42];
                sprintf(prevPart,"ALARM02,%s,%s,%s,%s,%s",alarm02.relayChannel,alarm02.ctrlType,alarm02.operationMode,alarm02.informType,alarm02.phoneNumber);
                char cfgPart[42];
                sprintf( cfgPart,"ALARM02,%s,%s,%s,%s,%s",ptr1,ptr2,ptr3,ptr4,ptr5);
                VnET_echo("\r\n Data configs before: %s|%ld",data_configs,strlen(data_configs));
                char * result = replaceWord(data_configs, prevPart, cfgPart);
                VnET_echo("\r\n Data configs after: %s|%ld",result,strlen(result));
                if (strlen(data_configs)==strlen(result)){
                    VnET_echo("\r\n Saved alarm02 configure completely");
                    int fd = nwy_sdk_fopen(path_cfgs_file, NWY_CREAT | NWY_RDWR | NWY_TRUNC);
                    nwy_sdk_fwrite(fd, result, CONFIGS_SIZE);
                    nwy_sdk_fclose(fd);
                    Response_Command(ptr0,src,"OK");
                }
                else{
                    VnET_echo("\r\n Save alarm02 configure fail, writing data length is not equal data saving length");
                    Response_Command(ptr0,src,"ERR");
                }
                strcpy(alarm02.relayChannel,ptr1);
                strcpy(alarm02.ctrlType,ptr2);
                strcpy(alarm02.operationMode,ptr3);
                strcpy(alarm02.informType,ptr4);
                strcpy(alarm02.phoneNumber,ptr5);
            }
            else{
                Response_Command(ptr0,src,"SYNTAX ERROR");
            }
        } 
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }

    else
    // ALARM03
    if ((!strncmp(strupr(ptr0),"ALARM03",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            char template[36];
            sprintf(template,"%s,%s,%s,%s,%s",alarm03.relayChannel,alarm03.ctrlType,alarm03.operationMode,alarm03.informType,alarm03.phoneNumber);
            Response_Command(ptr0,src,template);
        }
        else if (comma_number==5){
            if(checkAlarmMessage(ptr1,ptr2,ptr3,ptr4,ptr5)){
                char prevPart[42];
                sprintf(prevPart,"ALARM03,%s,%s,%s,%s,%s",alarm03.relayChannel,alarm03.ctrlType,alarm03.operationMode,alarm03.informType,alarm03.phoneNumber);
                char cfgPart[42];
                sprintf( cfgPart,"ALARM03,%s,%s,%s,%s,%s",ptr1,ptr2,ptr3,ptr4,ptr5);
                VnET_echo("\r\n Data configs before: %s|%ld",data_configs,strlen(data_configs));
                char * result = replaceWord(data_configs, prevPart, cfgPart);
                VnET_echo("\r\n Data configs after: %s|%ld",result,strlen(result));
                if (strlen(data_configs)==strlen(result)){
                    VnET_echo("\r\n Saved alarm03 configure completely");
                    int fd = nwy_sdk_fopen(path_cfgs_file, NWY_CREAT | NWY_RDWR | NWY_TRUNC);
                    nwy_sdk_fwrite(fd, result, CONFIGS_SIZE);
                    nwy_sdk_fclose(fd);
                    Response_Command(ptr0,src,"OK");
                }
                else{
                    VnET_echo("\r\n Save alarm03 configure fail, writing data length is not equal data saving length");
                    Response_Command(ptr0,src,"ERR");
                }
                strcpy(alarm03.relayChannel,ptr1);
                strcpy(alarm03.ctrlType,ptr2);
                strcpy(alarm03.operationMode,ptr3);
                strcpy(alarm03.informType,ptr4);
                strcpy(alarm03.phoneNumber,ptr5);
            }
            else{
                Response_Command(ptr0,src,"SYNTAX ERROR");
            }
        } 
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }

    else
    // ALARM05
    if ((!strncmp(strupr(ptr0),"ALARM05",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            char template[36];
            sprintf(template,"%s,%s,%s,%s,%s",alarm05.relayChannel,alarm05.ctrlType,alarm05.operationMode,alarm05.informType,alarm05.phoneNumber);
            Response_Command(ptr0,src,template);
        }
        else if (comma_number==5){
            if(checkAlarmMessage(ptr1,ptr2,ptr3,ptr4,ptr5)){
                char prevPart[27];
                sprintf(prevPart,"ALARM05,%s,%s,%s,%s,%s",alarm05.relayChannel,alarm05.ctrlType,alarm05.operationMode,alarm05.informType,alarm05.phoneNumber);
                char cfgPart[27];
                sprintf( cfgPart,"ALARM05,%s,%s,%s,%s,%s",ptr1,ptr2,ptr3,ptr4,ptr5);
                VnET_echo("\r\n Data configs before: %s|%ld",data_configs,strlen(data_configs));
                char * result = replaceWord(data_configs, prevPart, cfgPart);
                VnET_echo("\r\n Data configs after: %s|%ld",result,strlen(result));
                if (strlen(data_configs)==strlen(result)){
                    VnET_echo("\r\n Saved alarm05 configure completely");
                    int fd = nwy_sdk_fopen(path_cfgs_file, NWY_CREAT | NWY_RDWR | NWY_TRUNC);
                    nwy_sdk_fwrite(fd, result, CONFIGS_SIZE);
                    nwy_sdk_fclose(fd);
                    Response_Command(ptr0,src,"OK");
                }
                else{
                    VnET_echo("\r\n Save alarm05 configure fail, writing data length is not equal data saving length");
                    Response_Command(ptr0,src,"ERR");
                }
                strcpy(alarm05.relayChannel,ptr1);
                strcpy(alarm05.ctrlType,ptr2);
                strcpy(alarm05.operationMode,ptr3);
                strcpy(alarm05.informType,ptr4);
                strcpy(alarm05.phoneNumber,ptr5);
            }
            else{
                Response_Command(ptr0,src,"SYNTAX ERROR");
            }
        } 
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }

    else
    // TIMER - duration between messages update GPS to server
    if ((!strncmp(strupr(ptr0),"TIMER",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            char template[36];
            sprintf(template,"%d,%d",Tmin,Tmax);
            Response_Command(ptr0,src,template);
        }
        else if (comma_number==2){
            if ((atoi(ptr1) < atoi(ptr2)) && (atoi(ptr2) <= 180) && (atoi(ptr1) >= 5)){
                char prevPart[12];
                sprintf(prevPart,"Ts%03d#Tx%03d",Tmin,Tmax);
                char cfgPart[12];
                sprintf( cfgPart,"Ts%03d#Tx%03d",atoi(ptr1),atoi(ptr2));

                VnET_echo("\r\n Data configs before: %s|%ld",data_configs,strlen(data_configs));
                char * result = replaceWord(data_configs, prevPart, cfgPart);
                VnET_echo("\r\n Data configs after: %s|%ld",result,strlen(result));
                if (strlen(data_configs)==strlen(result)){
                    VnET_echo("\r\n Saved timer completely");
                    int fd = nwy_sdk_fopen(path_cfgs_file, NWY_CREAT | NWY_RDWR | NWY_TRUNC);
                    nwy_sdk_fwrite(fd, result, CONFIGS_SIZE);
                    nwy_sdk_fclose(fd);
                    Response_Command(ptr0,src,"OK");
                }
                else{
                    VnET_echo("\r\n Save timer fail, writing data length is not equal data saving length");
                    Response_Command(ptr0,src,"ERR");
                }
                Tmin = atoi(ptr1);
                Tmax = atoi(ptr2);
            }
            else{
                Response_Command(ptr0,src,"[ERROR] Note: 180 >= Tmax > Tmin >= 5 (s)"); 
            }
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }

    else
    // sleepMode
    if ((!strncmp(strupr(ptr0),"POWER",strlen(ptr0)))&&(src>0)){
        if (comma_number==0){
            if (sleepMode == 0){
                Response_Command(ptr0,src,"SLEEP");
            }
            else{
                Response_Command(ptr0,src,"AWAKE");
            }
        }
        else if (comma_number==1){
            if (!strncmp(strupr(ptr1),"MODE",strlen(ptr1))){
                Response_Command(ptr1,src,PowerSaving);
            }
            else if ((!strncmp(strupr(ptr1),"VOLTAGE",strlen(ptr1))) || (!strncmp(strupr(ptr1),"VOL",strlen(ptr1)))){
                read_Battery(&Battery_voltage);
                VnET_echo("\r\nMainPower: %s",Battery_voltage);
                Response_Command(ptr1,src,Battery_voltage);
            }
            else if ((!strncmp(strupr(ptr1),"THRES",strlen(ptr1))) || (!strncmp(strupr(ptr1),"THRESOLD",strlen(ptr1)))){
                Response_Command(ptr1,src,voltageThres);
            }
            else{
                Response_Command(ptr1,src,"SYNTAX ERROR");
            }
        }
        else if (comma_number==2){
            if (!strncmp(strupr(ptr1),"MODE",strlen(ptr1))){
                if ((!strncmp(strupr(ptr2),"PS0",strlen(ptr2))) || (!strncmp(strupr(ptr2),"PS1",strlen(ptr2)))){
                    VnET_echo("\r\n Data configs before: %s|%ld",data_configs,strlen(data_configs));
                    char * result = replaceWord(data_configs, PowerSaving, ptr2);
                    VnET_echo("\r\n Data configs after: %s|%ld",result,strlen(result));
                    if (strlen(data_configs)==strlen(result)){
                        VnET_echo("\r\n Saved power saving mode completely");
                        int fd = nwy_sdk_fopen(path_cfgs_file, NWY_CREAT | NWY_RDWR | NWY_TRUNC);
                        nwy_sdk_fwrite(fd, result, CONFIGS_SIZE);
                        nwy_sdk_fclose(fd);
                        PowerSaving = strdup(ptr2);
                        Response_Command(ptr1,src,"OK");
                    }
                    else{
                        Response_Command(ptr1,src,"SAVE ERROR");
                    }
                }
                else{
                    Response_Command(ptr1,src,"SYNTAX ERROR");
                }
            }
            else if ((!strncmp(strupr(ptr1),"THRES",strlen(ptr1))) || (!strncmp(strupr(ptr1),"THRESOLD",strlen(ptr1)))){
                if ((atof(ptr2) >= 11) || (atof(ptr2) <= 13)){
                    VnET_echo("\r\n Data configs before: %s|%ld",data_configs,strlen(data_configs));
                    char voltageThreshConv[5];
                    sprintf(voltageThreshConv,"%02.02f",atof(ptr2));
                    char * result = replaceWord(data_configs, voltageThres, voltageThreshConv);
                    VnET_echo("\r\n Data configs after: %s|%ld",result,strlen(result));
                    if (strlen(data_configs)==strlen(result)){
                        VnET_echo("\r\n Saved voltage threshold completely");
                        int fd = nwy_sdk_fopen(path_cfgs_file, NWY_CREAT | NWY_RDWR | NWY_TRUNC);
                        nwy_sdk_fwrite(fd, result, CONFIGS_SIZE);
                        nwy_sdk_fclose(fd);
                        voltageThres = strdup((char *)voltageThreshConv);
                        Response_Command(ptr1,src,"OK");
                    }
                    else{
                        Response_Command(ptr1,src,"SAVE ERROR");
                    }
                }
                else{
                    Response_Command(ptr1,src,"ERROR,OUT OF RANGE(11.0-13.0 V)");
                }
            }
            else{
                Response_Command(ptr1,src,"SYNTAX ERROR");
            }
        }
        else{
            Response_Command(ptr0,src,"ERR");
        }
    }
    else{
        Response_Command(ptr0,src,"SYNTAX ERROR");
    }
}

int checkAlarmMessage(char * ptr1, char * ptr2, char * ptr3, char * ptr4, char * ptr5){
    int argumentValidCounter = 0;
    if ((!strncmp(ptr1,"0",strlen(ptr1))) || (!strncmp(ptr1,"1",strlen(ptr1))) || (!strncmp(strupr(ptr1),"X",strlen(ptr1)))){
        argumentValidCounter ++;
        VnET_echo("\r\n ptr1 -  valid");
    }
    if ((!strncmp(ptr2,"0",strlen(ptr2))) || (!strncmp(ptr2,"1",strlen(ptr2))) || (!strncmp(strupr(ptr2),"X",strlen(ptr2)))){
        argumentValidCounter ++;
        VnET_echo("\r\n ptr2 -  valid");
    }
    if ((!strncmp(ptr3,"0",strlen(ptr3))) || (!strncmp(ptr3,"1",strlen(ptr3)))){
        argumentValidCounter ++;
        VnET_echo("\r\n ptr3 -  valid");
    }
    if ((!strncmp(ptr4,"0",strlen(ptr4))) || (!strncmp(ptr4,"1",strlen(ptr4)))){
        argumentValidCounter ++;
        VnET_echo("\r\n ptr4 -  valid");
    }
    if (strlen(ptr5)==10){
        argumentValidCounter ++;
        VnET_echo("\r\n ptr5 -  valid");
    }
    VnET_echo("\r\nNumber of argument valid: %d",argumentValidCounter);
    if (argumentValidCounter==5){
        return 1;
    }
    else{
        return 0;
    }
}

#endif

#if iUART
void uart_recv_handle (unsigned char *str,uint32_t length){
    VnET_echo("\r\n Receive: %s - Length: %d ",str,length);
    if (findSpecialChar(str,length,'#')>0){
        VnET_echo("\r\n[UART] Valid command");
        Center_Handle_Command(str,length,SRC_UART);
    }
    else{
        VnET_echo("\r\n[UART] Invalid command");
    }
}

void uart_main(void){
    nwy_sleep(200);
    UART_CFG.uart_init = nwy_uart_init;
    UART_CFG.uart_send = nwy_uart_send_data;
    // 0-at mode, 1-data mode
    UART_CFG.uart_connection = UART_CFG.uart_init(NWY_NAME_UART1,1);
    if (UART_CFG.uart_connection!=-1){
        VnET_echo("\r\n  Open COM %d successfully",UART_CFG.uart_connection);
    }
    nwy_uart_reg_recv_cb(UART_CFG.uart_connection,uart_recv_handle);
}

int deinit_uart(void * hdlr){
    int uartHdlr = *(int *)hdlr;
    int retDeinitUART = nwy_uart_deinit(uartHdlr);
    return retDeinitUART;
}
#endif

#if SIM_Service
void checkSIMSlot(){
    int ret_sim_slot = nwy_test_cli_get_sim_slot(); // 0-uSIM1 | 1-USIM2
    nwy_sleep(1000);
    if (!ret_sim_slot){
        nwy_test_cli_set_sim_slot();
        nwy_sleep(1000);
        nwy_power_off(2);
    }
}
int vnet_send_sms(char phoneNumber[], char message[]){
	nwy_sms_info_type_t sms_data = {0};
    strcpy(sms_data.phone_num,phoneNumber);
	sms_data.msg_context_len = strlen(message);
    strcpy(sms_data.msg_contxet,message);
	sms_data.msg_format = 0; //0:GSM7 2:UNICODE

	int ret = nwy_sms_send_message(&sms_data);
	if(NWY_SMS_SUCCESS == ret){
        VnET_echo("\r\nSent SMS success\n");
		return 0;
	}
	else{
        VnET_echo("\r\nSent SMS fail\n");    
        return 0;
    }
}
int set_report_mode_sms(){
    uint8_t mode = 2;
	uint8_t mt = 2;
	uint8_t bm = 0;
	uint8_t ds = 0;
	uint8_t bfr = 0;
	nwy_result_t ret = NWY_SMS_SUCCESS;
	ret = nwy_set_report_option(mode, mt, bm, ds, bfr);
	if(NWY_SMS_SUCCESS != ret){
	    VnET_echo("\r\nnwy set sms report mode fail");
		return -1;
	}
	return 0;
}
void nwy_recv_sms(){
    nwy_sms_recv_info_type_t sms_data = {0};
    nwy_sms_recv_message(&sms_data);
    int ret = 0;
    if(1 == sms_data.cnmi_mt){  // set_report_mode_sms(2,1,0,0,0)
        VnET_echo("\r\n recv one sms index:%d",sms_data.nIndex);
    }
	else if(2 == sms_data.cnmi_mt){ // set_report_mode_sms(2,2,0,0,0)
	    VnET_echo("\r\n recv one sms from:%s msg_context:%s",sms_data.oa,sms_data.pData);
        strcpy(sender_number,sms_data.oa);
        if (findSpecialChar(sms_data.pData,strlen(sms_data.pData),'#')>0){
            VnET_echo("\r\n[SMS] Valid command");
            Center_Handle_Command(sms_data.pData,strlen(sms_data.pData),SRC_SMS);
        }
        else{
            VnET_echo("\r\n[SMS] Invalid command");
        }
	}
}

// CALL
int vnet_voice_call(char phoneNumber[]){
    int sim_id = 0x00;
    nwy_voice_setvolte(sim_id,0);//volte(1-on 0-off)
    int ret = nwy_voice_call_start(sim_id,phoneNumber);
    if (ret == 0) {
        VnET_echo("\r\nCall success\n");
        return 1;
    }
    else{
        VnET_echo("\r\nCall fail\n");    
        return 0;
    }
}

static void nwy_voice_ind(){
	char state[64] ={0};
	nwy_get_voice_state(state);
    nwy_ext_echo("\r\nSomeone called - %s",state);
    call_alert = 1;
}
void nwy_test_cli_set_sim_slot(){
  char* sptr = NULL;
  uint8 nSwitchSimID = 0;
  nwy_result_type ret = NWY_RES_OK;
  nSwitchSimID = 1;

  ret = nwy_sim_set_simid(nSwitchSimID);
  if(NWY_RES_OK != ret)
    VnET_echo("\r\nnwy set switch simid fail!\r\n");
  else
    VnET_echo("\r\nnwy set switch simid success!\r\n");

  return;
}

int  nwy_test_cli_get_sim_slot(){
  uint8 nSwitchSimID = 0;

  nSwitchSimID = nwy_sim_get_simid();

  VnET_echo("\r\n simid: %d \r\n", nSwitchSimID);
  return nSwitchSimID;
}


// I2C
static bool nwy_i2c_set_addr(int bus, uint8_t dev, uint16_t reg, bool read)
{
    if (nwy_i2c_raw_put_byte(bus, (dev << 1), 1, 0))
    {
        OSI_LOGE(0, "I2C dev addr send fail");
        return false;
    }
    if (reg & 0xff00)
    {
        if (nwy_i2c_raw_put_byte(bus, reg >> 8, 0, 0))
            OSI_LOGE(0, "I2C reg addrh send fail");
    }
    if (reg)
    {
        if (nwy_i2c_raw_put_byte(bus, reg & 0x00ff, 0, 0))
            OSI_LOGE(0, "I2C reg addrl send fail");
    }
    if (read)
    {
        if (nwy_i2c_raw_put_byte(bus, ((dev << 1) | 0x1), 1, 0))
        {
            OSI_LOGE(0, "I2C dev restart addr send fail");
            return false;
        }
    }

    return true;
}

static bool nwy_i2c_read_dev_reg(int bus, uint8_t dev, uint16_t reg, uint8_t *buf, uint32_t length)
{
    bool result = false;

    osiThreadSleep(1);
    // write slave address (write mode, to write memory address)
    if (!nwy_i2c_set_addr(bus, dev, reg, true))
    {
        OSI_LOGE(0, "I2C read fail, send address");
        return false;
    }

    // read all values but the last one
    for (uint32_t c = 0; c < length - 1; c++)
    {
        // Read value
        if (nwy_i2c_raw_get_byte(bus, buf + c, 0, 0))
        {
            OSI_LOGE(0, "I2C read fail, read data fail");
            return false;
        }
    }

    // Read last value - send no acknowledge - send stop condition/bit
    if (nwy_i2c_raw_get_byte(bus, buf + length - 1, 0, 1))
    {
        OSI_LOGE(0, "I2C read fail, read last byte fail");
        return false;
    }

    return true;
}

static bool nwy_i2c_write_dev_reg(int bus, uint8_t dev, uint16_t reg, uint8_t *buf, uint32_t length)
{
    bool result = false;

    osiThreadSleep(1);
    // write slave address (write mode, to write memory address)
    if (!nwy_i2c_set_addr(bus, dev, reg, false))
    {
        OSI_LOGE(0, "I2C write fail, write address");
        return false;
    }

    // write all values but the last one
    for (uint32_t c = 0; c < length - 1; c++)
    {
        // Read value
        if (nwy_i2c_raw_put_byte(bus, buf[c], 0, 0))
        {
            OSI_LOGE(0, "I2C write fail, write data fail");
            return false;
        }
    }
    // write last value - send stop condition/bit
    if (nwy_i2c_raw_put_byte(bus, buf[length - 1], 0, 1))
    {
        OSI_LOGE(0, "I2C write fail, write last byte fail");
        return false;
    }
    return true;
}

int check_i2c_device(){
    //Read WHO_AM_I
    uint8_t ptrBuff = 0;
    uint8_t ptrBuff2 = 0;
    
    
    nwy_i2c_read_dev_reg(i2c_init,LIS3DH_DEV_ADDR,LIS3DH_WHO_AM_I,&ptrBuff2,1);
    VnET_echo("\r\nWHO-AM-I: %02X",ptrBuff2);
    if (ptrBuff2 == LIS3DH_WHO_AM_I_VALUE){
        VnET_echo("\r\nI2C Device: LIS3DH");
        configs_i2c_device = &configs_LIS3DH;
        motion_detect = &motion_LIS3DH;
        return 1;
    }
    else{
        for(int i=10;i>0;i--)
            nwy_i2c_read_dev_reg(i2c_init,LIS3DSH_DEV_ADDR,LIS3DSH_WHO_AM_I,&ptrBuff,1);
        VnET_echo("\r\nWHO-AM-I: %02X",ptrBuff);
        if (ptrBuff == LIS3DSH_WHO_AM_I_VALUE){
            VnET_echo("\r\nI2C Device: LIS3DSH");
            configs_i2c_device = &configs_LIS3DSH;
            motion_detect = &motion_LIS3DSH;
            return 1;
        }
        else{
            VnET_echo("\r\nI2C Device is not available");
            return 0;
        }
    }
}

void configs_LIS3DSH(){
    uint8_t ctrl = 0;
    int ret = -1;
    sensitivity = 16384.0;  // corresponding for 2g | 65536/4 - sensitivity
    // LIS3DSH_CTRL_REG4_ADDR
    ctrl = 0x67;
    ret = nwy_i2c_write_dev_reg(i2c_init, LIS3DSH_DEV_ADDR,LIS3DSH_CTRL_REG4_ADDR,&ctrl,1);
    VnET_echo((ret) ? "\r\nSetup CTRL_REG4 completed" : "\r\nSetup CTRL_REG4 false");
    // LIS3DSH_CTRL_REG5_ADDR
    ctrl = 0xC0;
    ret = nwy_i2c_write_dev_reg(i2c_init, LIS3DSH_DEV_ADDR,LIS3DSH_CTRL_REG5_ADDR,&ctrl,1);
    VnET_echo((ret) ? "\r\nSetup CTRL_REG5 completed" : "\r\nSetup CTRL_REG5 false");
}

int motion_LIS3DSH(){
    VnET_echo("\r\n----------------");
    // Method 1
    // nwy_i2c_read_dev_reg(i2c_init, LIS3DSH_DEV_ADDR, LIS3DSH_OUT_X_L, &ACC_X_L, 1);
    // nwy_i2c_read_dev_reg(i2c_init, LIS3DSH_DEV_ADDR, LIS3DSH_OUT_X_H, &ACC_X_H, 1);
    // nwy_i2c_read_dev_reg(i2c_init, LIS3DSH_DEV_ADDR, LIS3DSH_OUT_Y_L, &ACC_Y_L, 1);
    // nwy_i2c_read_dev_reg(i2c_init, LIS3DSH_DEV_ADDR, LIS3DSH_OUT_Y_H, &ACC_Y_H, 1);
    // nwy_i2c_read_dev_reg(i2c_init, LIS3DSH_DEV_ADDR, LIS3DSH_OUT_Z_L, &ACC_Z_L, 1);
    // nwy_i2c_read_dev_reg(i2c_init, LIS3DSH_DEV_ADDR, LIS3DSH_OUT_Z_H, &ACC_Z_H, 1);

    // Method 2
    uint8_t rawData[7];
    nwy_i2c_read_dev_reg(i2c_init, LIS3DSH_DEV_ADDR, LIS3DSH_STATUS_ADDR, rawData, 7);
    if (rawData[0] & 0x08){
        ACC_X_L = rawData[1];
        ACC_X_H = rawData[2];
        ACC_Y_L = rawData[3];
        ACC_Y_H = rawData[4];
        ACC_Z_L = rawData[5];
        ACC_Z_H = rawData[6];
    }
    short axisX,axisY,axisZ;
    axisX = (ACC_X_H << 8) + ACC_X_L;
    axisY = (ACC_Y_H << 8) + ACC_Y_L;
    axisZ = (ACC_Z_H << 8) + ACC_Z_L;
    lis3dsh.AccelX      = (float)(axisX)/sensitivity;
    lis3dsh.AccelY      = (float)(axisY)/sensitivity;
    lis3dsh.AccelZ      = (float)(axisZ)/sensitivity;
    lis3dsh.biasAccelX  = fabsf(lis3dsh.prevAccelX-lis3dsh.AccelX);
    lis3dsh.biasAccelY  = fabsf(lis3dsh.prevAccelY-lis3dsh.AccelY);
    lis3dsh.biasAccelZ  = fabsf(lis3dsh.prevAccelZ-lis3dsh.AccelZ);
    VnET_echo("\r\nAccelX: %.03f (g) | %.03f (g) | %.03f (g)",lis3dsh.AccelX,lis3dsh.prevAccelX,lis3dsh.biasAccelX);   
    VnET_echo("\r\nAccelY: %.03f (g) | %.03f (g) | %.03f (g)",lis3dsh.AccelY,lis3dsh.prevAccelY,lis3dsh.biasAccelY);
    VnET_echo("\r\nAccelZ: %.03f (g) | %.03f (g) | %.03f (g)",lis3dsh.AccelZ,lis3dsh.prevAccelZ,lis3dsh.biasAccelZ);
    if (((lis3dsh.biasAccelX >= motionSensitive) || (lis3dsh.biasAccelY >= motionSensitive) || (lis3dsh.biasAccelZ >= motionSensitive)) 
            && (lis3dsh.prevAccelX!=0.0) && (lis3dsh.prevAccelY!=0.0) && (lis3dsh.prevAccelZ!=0.0)){
        lis3dsh.motionDetect = 1;
    }
    else{
        lis3dsh.motionDetect = 0;
    }
    lis3dsh.prevAccelX = lis3dsh.AccelX;
    lis3dsh.prevAccelY = lis3dsh.AccelY;
    lis3dsh.prevAccelZ = lis3dsh.AccelZ;
    return lis3dsh.motionDetect;
}
#endif
// =========================== End of API source





// This section is used for new functions, which is not stable ******************************************************************************

void configs_LIS3DH(){
    uint8_t ctrl = 0;
    int ret = -1;
    sensitivity = 16384.0;  // corresponding for 2g | 65536/4 - sensitivity
    // LIS3DH_CTRL_REG1_ADDR
    ctrl = 0x67;
    ret = nwy_i2c_write_dev_reg(i2c_init, LIS3DH_DEV_ADDR,LIS3DH_CTRL_REG1_ADDR,&ctrl,1);
    VnET_echo((ret) ? "\r\nSetup CTRL_REG1 completed" : "\r\nSetup CTRL_REG1 false");
    // LIS3DH_CTRL_REG4_ADDR
    ctrl = 0x48;
    ret = nwy_i2c_write_dev_reg(i2c_init, LIS3DH_DEV_ADDR,LIS3DH_CTRL_REG4_ADDR,&ctrl,1);
    VnET_echo((ret) ? "\r\nSetup CTRL_REG5 completed" : "\r\nSetup CTRL_REG5 false");
}

int motion_LIS3DH(){
    VnET_echo("\r\n----------------");
    nwy_i2c_read_dev_reg(i2c_init, LIS3DH_DEV_ADDR, LIS3DH_OUT_X_L, &ACC_X_L, 1);
    nwy_i2c_read_dev_reg(i2c_init, LIS3DH_DEV_ADDR, LIS3DH_OUT_X_H, &ACC_X_H, 1);
    nwy_i2c_read_dev_reg(i2c_init, LIS3DH_DEV_ADDR, LIS3DH_OUT_Y_L, &ACC_Y_L, 1);
    nwy_i2c_read_dev_reg(i2c_init, LIS3DH_DEV_ADDR, LIS3DH_OUT_Y_H, &ACC_Y_H, 1);
    nwy_i2c_read_dev_reg(i2c_init, LIS3DH_DEV_ADDR, LIS3DH_OUT_Z_L, &ACC_Z_L, 1);
    nwy_i2c_read_dev_reg(i2c_init, LIS3DH_DEV_ADDR, LIS3DH_OUT_Z_H, &ACC_Z_H, 1);
    lis3dh.AccelX      = ((ACC_X_H*256 + ACC_X_L)/*/sensitivity*/);
    lis3dh.AccelY      = ((ACC_Y_H*256 + ACC_Y_L)/*/sensitivity*/);
    lis3dh.AccelZ      = ((ACC_Z_H*256 + ACC_Z_L)/*/sensitivity*/);
    lis3dh.biasAccelX  = fabsf(lis3dh.prevAccelX-lis3dh.AccelX);
    lis3dh.biasAccelY  = fabsf(lis3dh.prevAccelY-lis3dh.AccelY);
    lis3dh.biasAccelZ  = fabsf(lis3dh.prevAccelZ-lis3dh.AccelZ);
    VnET_echo("\r\nAccelX: %.03f (g) | %.03f (g) | %.03f (g)",lis3dh.AccelX,lis3dh.prevAccelX,lis3dh.biasAccelX);   
    VnET_echo("\r\nAccelX: %.03f (g) | %.03f (g) | %.03f (g)",lis3dh.AccelY,lis3dh.prevAccelY,lis3dh.biasAccelY);
    VnET_echo("\r\nAccelX: %.03f (g) | %.03f (g) | %.03f (g)",lis3dh.AccelZ,lis3dh.prevAccelZ,lis3dh.biasAccelZ);
    if (((lis3dh.biasAccelX >= motionSensitive) || (lis3dh.biasAccelY >= motionSensitive) || (lis3dh.biasAccelZ >= motionSensitive)) 
            && (lis3dh.prevAccelX!=0.0) && (lis3dh.prevAccelY!=0.0) && (lis3dh.prevAccelZ!=0.0)){
        lis3dh.motionDetect = 1;
    }
    else{
        lis3dh.motionDetect = 0;
    }
    lis3dh.prevAccelX = lis3dh.AccelX;
    lis3dh.prevAccelY = lis3dh.AccelY;
    lis3dh.prevAccelZ = lis3dh.AccelZ;
    return lis3dh.motionDetect;
}
