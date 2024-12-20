#ifndef GPS_H
#define GPS_H

#include "main.h"

#define GPS_MSG_NMEA_ADDR_LEN 16

#define GPS_MSG_NMEA_TIMESTAMP_LEN 10
#define GPS_MSG_NMEA_LATITUDE_LEN 12
#define GPS_MSG_NMEA_LATDIR_LEN 1
#define GPS_MSG_NMEA_LONGITUDE_LEN 12
#define GPS_MSG_NMEA_LONGDIR_LEN 1
#define GPS_MSG_NMEA_PRECISION_LEN 1
#define GPS_MSG_NMEA_NSATELLITE_LEN 2
#define GPS_MSG_NMEA_HDOP_LEN 4
#define GPS_MSG_NMEA_ALTITUDE_LEN 7
#define GPS_MSG_NMEA_ALTUNIT_LEN 1
#define GPS_MSG_NMEA_GEOSEP_LEN 6
#define GPS_MSG_NMEA_GEOUNIT_LEN 1
#define GPS_MSG_NMEA_CORRECTIONAGE_LEN 3
#define GPS_MSG_NMEA_CORRECTIONSTATID_LEN 4
#define GPS_MSG_NMEA_CHECKSUM_LEN 3
#define GPS_MSG_NMEA_AUTOMODE_LEN 1
#define GPS_MSG_NMEA_MODE_LEN 1
#define GPS_MSG_NMEA_PRN_NUM_OF_SAT 12-1
#define GPS_MSG_NMEA_PRN_SAT_LEN 2
#define GPS_MSG_NMEA_PDOP_LEN 3
#define GPS_MSG_NMEA_VDOP_LEN 3
#define GPS_MSG_NMEA_STATUS_LEN 1
#define GPS_MSG_NMEA_GROUNDSPEED_LEN 4
#define GPS_MSG_NMEA_GROUNDCOURSE_LEN 6
#define GPS_MSG_NMEA_DATE_LEN 6
#define GPS_MSG_NMEA_MAGVAR_LEN 1
#define GPS_MSG_NMEA_EASTWEST_LEN 1

#define GPS_NMEA_MAX_SENTENCE_LEN 82
#define GPS_NMEA_NUM_OF_SENTENCES_IN_PARA 4
#define GPS_NMEA_MAX_PARA_LEN GPS_NMEA_MAX_SENTENCE_LEN * GPS_NMEA_NUM_OF_SENTENCES_IN_PARA

enum Gps_Task_Status_enum {
	GPS_TASK_UART = 0,
	GPS_TASK_DATA
} typedef Gps_TaskStatus;

struct Time_struct {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
} typedef Time;

struct Position_struct {
	float latitude;
	float longitude;
} typedef Position;

struct Gps_Data_struct {
	Position pos;
	Time time;
} typedef GpsPosition;

void write_GpsData(GpsPosition* data);
GpsPosition* read_GpsData();

/* =========== UART MESSAGE PROCESSING  ========= */
/* =                                            = */
/* ============================================== */
enum Gps_Uart_ProcessStatus_enum {
	GPS_UART_PROCESS_RX = 0,
	GPS_UART_PROCESS_RX_WAIT,
	GPS_UART_PROCESS_RX_PROCESS,
	GPS_UART_PROCESS_RX_COMPLETE,
	GPS_UART_PROCESS_RX_ERROR
} typedef Gps_Uart_ProcessStatus;

struct Gps_Msg_struct {
	char address[GPS_MSG_NMEA_ADDR_LEN];
} typedef GpsMsg;

struct Gps_Msg_LatLong_struct {
	char latitude[GPS_MSG_NMEA_LATITUDE_LEN];
	char latDir[GPS_MSG_NMEA_LATDIR_LEN];
	char longitude[GPS_MSG_NMEA_LONGITUDE_LEN];
	char longDir[GPS_MSG_NMEA_LONGDIR_LEN];
} typedef GpsMsg_LatLong;

/*
 * GPGGA sentence:
 * - utc timestamp
 * - latitude
 * - latitude direction
 * - longitude
 * - longitude direction
 * - quality indicator
 * - number of satellites
 * - horizontal dilution of precision (HDOP)
 * - altitude
 * - altitude units (Meters or Feet)
 * - geoidal seperation
 * - geoidal seperation units (Meters or Feet)
 * - correction station ID
 * - checksum
 */
struct Gps_Msg_GPGG_struct {
	GpsMsg address;
	char timestamp[GPS_MSG_NMEA_TIMESTAMP_LEN];
	GpsMsg_LatLong latLong;
	char precision[GPS_MSG_NMEA_PRECISION_LEN];
	char nSatellites[GPS_MSG_NMEA_NSATELLITE_LEN];
	char hdop[GPS_MSG_NMEA_HDOP_LEN];
	char altitude[GPS_MSG_NMEA_ALTITUDE_LEN];
	char altUnit[GPS_MSG_NMEA_ALTUNIT_LEN];
	char geoSeperation[GPS_MSG_NMEA_GEOSEP_LEN];
	char geoUnit[GPS_MSG_NMEA_GEOUNIT_LEN];
	char correctionAge[GPS_MSG_NMEA_CORRECTIONAGE_LEN];
//	char correctionStationID[GPS_MSG_NMEA_CORRECTIONSTATID_LEN];
	char chksm[GPS_MSG_NMEA_CHECKSUM_LEN];
} typedef GpsMsg_GPGGA;

struct Gps_Msg_GPGSA_struct {
	GpsMsg address;
	char isModeAuto[GPS_MSG_NMEA_AUTOMODE_LEN];
	char mode[GPS_MSG_NMEA_MODE_LEN];
	char prn[GPS_MSG_NMEA_PRN_NUM_OF_SAT][GPS_MSG_NMEA_PRN_SAT_LEN];
	char pdop[GPS_MSG_NMEA_PDOP_LEN];
	char hdop[GPS_MSG_NMEA_HDOP_LEN];
	char vdop[GPS_MSG_NMEA_VDOP_LEN];
//	char systemID[2];
	char chksm[GPS_MSG_NMEA_CHECKSUM_LEN];
} typedef GpsMsg_GPGSA;

struct Gps_Msg_GPRMC_struct {
	GpsMsg address;
	char timestamp[GPS_MSG_NMEA_TIMESTAMP_LEN];
	char status[GPS_MSG_NMEA_STATUS_LEN];
	GpsMsg_LatLong latLong;
	char groundSpeed[GPS_MSG_NMEA_GROUNDSPEED_LEN];
	char groundCourse[GPS_MSG_NMEA_GROUNDCOURSE_LEN];
	char date[GPS_MSG_NMEA_DATE_LEN];
	char magVar[GPS_MSG_NMEA_MAGVAR_LEN];
	char eastWest[GPS_MSG_NMEA_EASTWEST_LEN];
	char chksm[GPS_MSG_NMEA_CHECKSUM_LEN];
} typedef GpsMsg_GPRMC;

void gps_main();
void gps_uart_rxComplete(size_t size);
void gps_msg_process();
void gps_uart_process();

void gps_uart_msg_processMulti();
uint8_t gps_uart_msg_packStruct(uint8_t* startIdx, uint8_t* msgArrayIdx);
void gps_uart_msg_packFixType(GpsMsg* baseMsg, uint8_t* currIdx);
void gps_uart_msg_packGSA(GpsMsg* baseMsg, uint8_t* currIdx);
void gps_uart_msg_packRMC(GpsMsg* baseMsg, uint8_t* currIdx);
void gps_uart_msg_fillLatLong(GpsMsg_LatLong* latLongPtr, uint8_t* currIdx);
void gps_uart_msg_fillData(char* dataPtr, uint8_t len, uint8_t* currIdx);
#endif
