#include "gps.h"
#include "cmsis_os.h"
#include "usart.h"
#include "string.h"

static Gps_Uart_ProcessStatus gps_uart_processStatus;
static Gps_TaskStatus gps_task_status;
static GpsPosition gps_posData;

static size_t rxSize = 0;
static uint8_t msgsArray[GPS_NMEA_MAX_PARA_LEN];
static uint8_t gps_uart_data[GPS_NMEA_MAX_PARA_LEN] = {0};
static uint8_t gps_data_readingFlag = 0;

void gps_main() {
	// change baudrate of gps sensor and then uart4 to 115200
//	  uint8_t baudrateCmd[20] = "$PMTK251,38400*27\r\n";
//	  HAL_UART_Transmit(&huart4, &baudrateCmd, 20, 500);
//	  UART4->CR1 &= ~(USART_CR1_UE);
//	  UART4->BRR = 115200;
//	  UART4->CR1 |= USART_CR1_UE;

	while(1){
		switch(gps_task_status) {
			case GPS_TASK_UART:
				gps_uart_process();
				break;
			case GPS_TASK_DATA:
				gps_msg_process();
				break;
			default:
				// should not get here
				break;
		}
		osDelay(1);
	}
}

void gps_uart_process() {
	switch(gps_uart_processStatus) {
		case GPS_UART_PROCESS_RX:
			HAL_UARTEx_ReceiveToIdle_DMA(&huart4, gps_uart_data, GPS_NMEA_MAX_PARA_LEN);
			gps_uart_processStatus = GPS_UART_PROCESS_RX_WAIT;
			break;
		case GPS_UART_PROCESS_RX_WAIT:
			// Nothing needs to be done here
			// The switch to rx_complete will be done in interrupt
			osDelay(1);
			break;
		case GPS_UART_PROCESS_RX_PROCESS:
			gps_uart_msg_processMulti();
			break;
		case GPS_UART_PROCESS_RX_COMPLETE:
			gps_uart_processStatus = GPS_UART_PROCESS_RX;
			gps_task_status = GPS_TASK_DATA;
			break;
		default:
			break;
	}
}

uint8_t gps_uart_nmea_getAddressLen(unsigned char* start) {
	for(uint8_t i = 0; (size_t) start + i < (size_t) start + GPS_MSG_NMEA_ADDR_LEN; i++) {
		if (start[i] == ',') return i;
	}
	return 255;
}

void gps_uart_msg_processMulti() {
	uint8_t i = 0;
	uint8_t totalSize = 0;
	while (i < rxSize) {
		i += 1; // Skip dollar sign
		if (0 == gps_uart_msg_packStruct(&i, &totalSize)) {
			gps_uart_processStatus = GPS_UART_PROCESS_RX_ERROR;
			return;
		}
		i += 1; // Skip \r\n
	}
}

uint8_t gps_uart_msg_packStruct(uint8_t* startIdx, uint8_t* msgArrayIdx) {
	// start from gps_uart_data[1] because [0] will be $
	uint8_t len = gps_uart_nmea_getAddressLen(&gps_uart_data[*startIdx]);
	GpsMsg* msg = (GpsMsg*) &msgsArray[*msgArrayIdx];

	if (len == 255) {
		return 0;
	}
	for (uint8_t j = 0; j < len; j++) {
		msg->address[j] = gps_uart_data[*startIdx + j];
	}
	msg->address[len] = 0;
	*startIdx += len + 1;

	if (strcmp(msg->address, "GPGGA") == 0) {
		gps_uart_msg_packFixType(msg, startIdx);
		GpsMsg_GPGGA* gpgga = (GpsMsg_GPGGA*) &msg;
		*msgArrayIdx += sizeof(GpsMsg_GPGGA);
//	} else if (strcmp(msg.address, "GPGLL") == 0) {
//		gps_uart_msg_packLatLong(&msg);
	} else if (strcmp(msg->address, "GPGSA") == 0) {
		gps_uart_msg_packGSA(msg, startIdx);
		*msgArrayIdx += sizeof(GpsMsg_GPGSA);
//	} else if (strcmp(msg.address, "GPGSV") == 0) {
//		gps_uart_msg_packSatView(&msg);
	} else if (strcmp(msg->address, "GPRMC") == 0) {
		gps_uart_msg_packRMC(&msg);
//	} else if (strcmp(msg.address, "GPVTG") == 0) {
//		gps_uart_msg_packRelative(&msg);
	} else {
		return 0;
	}
	return 1;
}

void gps_uart_msg_fillData(char* dataPtr, uint8_t len, uint8_t* currIdx) {
	size_t i = 0;
	for(; i < len; i++) {
		if (gps_uart_data[*currIdx + i] == ',') {
			break; // reached end of timestamp
		}
		dataPtr[i] = gps_uart_data[*currIdx + i];
	}
	*currIdx += i + 1;
}

void gps_uart_msg_fillLatLong(GpsMsg_LatLong* latLongPtr, uint8_t* currIdx) {
	gps_uart_msg_fillData(latLongPtr->latitude, GPS_MSG_NMEA_LATITUDE_LEN, currIdx);
	gps_uart_msg_fillData(latLongPtr->latDir, GPS_MSG_NMEA_LATDIR_LEN, currIdx);
	gps_uart_msg_fillData(latLongPtr->longitude, GPS_MSG_NMEA_LONGITUDE_LEN, currIdx);
	gps_uart_msg_fillData(latLongPtr->longDir, GPS_MSG_NMEA_LONGDIR_LEN, currIdx);
}

void gps_uart_msg_packRMC(GpsMsg* baseMsg, uint8_t* currIdx) {
	GpsMsg_GPRMC* msg = (GpsMsg_GPRMC*) baseMsg;
	gps_uart_msg_fillData(msg->timestamp, GPS_MSG_NMEA_TIMESTAMP_LEN, currIdx);
	gps_uart_msg_fillData(msg->status, GPS_MSG_NMEA_STATUS_LEN, currIdx);
	gps_uart_msg_fillLatLong(msg->latLong, currIdx);
	gps_uart_msg_fillData(msg->groundSpeed, GPS_MSG_NMEA_GROUNDSPEED_LEN, currIdx);
	gps_uart_msg_fillData(msg->groundCourse, GPS_MSG_NMEA_GROUNDCOURSE_LEN, currIdx);
	gps_uart_msg_fillData(msg->date, GPS_MSG_NMEA_DATE_LEN, currIdx);
	gps_uart_msg_fillData(msg->magVar, GPS_MSG_NMEA_MAGVAR_LEN, currIdx);
	gps_uart_msg_fillData(msg->eastWest, GPS_MSG_NMEA_EASTWEST_LEN, currIdx);
	gps_uart_msg_fillData(msg->chksm, GPS_MSG_NMEA_CHECKSUM_LEN, currIdx);
}

void gps_uart_msg_packGSA(GpsMsg* baseMsg, uint8_t* currIdx) {
	GpsMsg_GPGSA* msg = (GpsMsg_GPGSA*) baseMsg;
	gps_uart_msg_fillData(msg->isModeAuto, GPS_MSG_NMEA_AUTOMODE_LEN, currIdx);
	gps_uart_msg_fillData(msg->mode, GPS_MSG_NMEA_MODE_LEN, currIdx);
	for (uint8_t i = 0; i < GPS_MSG_NMEA_PRN_NUM_OF_SAT; i++) {
		gps_uart_msg_fillData(msg->prn[i], GPS_MSG_NMEA_PRN_SAT_LEN, currIdx);
	}
	gps_uart_msg_fillData(msg->pdop, GPS_MSG_NMEA_PDOP_LEN, currIdx);
	gps_uart_msg_fillData(msg->hdop, GPS_MSG_NMEA_HDOP_LEN, currIdx);
	gps_uart_msg_fillData(msg->vdop, GPS_MSG_NMEA_VDOP_LEN, currIdx);
	gps_uart_msg_fillData(msg->chksm, GPS_MSG_NMEA_CHECKSUM_LEN, currIdx);
}

/*
 * Processes a GPGGA sentence:
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
void gps_uart_msg_packFixType(GpsMsg* baseMsg, uint8_t* currIdx) {
	GpsMsg_GPGGA* msg = (GpsMsg_GPGGA*) baseMsg;
	gps_uart_msg_fillData(msg->timestamp, GPS_MSG_NMEA_TIMESTAMP_LEN, currIdx);
	gps_uart_msg_fillLatLong(&msg->latLong, currIdx);
	gps_uart_msg_fillData(msg->precision, GPS_MSG_NMEA_PRECISION_LEN, currIdx);
	gps_uart_msg_fillData(msg->nSatellites, GPS_MSG_NMEA_NSATELLITE_LEN, currIdx);
	gps_uart_msg_fillData(msg->hdop, GPS_MSG_NMEA_HDOP_LEN, currIdx);
	gps_uart_msg_fillData(msg->altitude, GPS_MSG_NMEA_ALTITUDE_LEN, currIdx);
	gps_uart_msg_fillData(msg->altUnit, GPS_MSG_NMEA_ALTUNIT_LEN, currIdx);
	gps_uart_msg_fillData(msg->geoSeperation, GPS_MSG_NMEA_GEOSEP_LEN, currIdx);
	gps_uart_msg_fillData(msg->geoUnit, GPS_MSG_NMEA_GEOUNIT_LEN, currIdx);
	gps_uart_msg_fillData(msg->correctionAge, GPS_MSG_NMEA_CORRECTIONAGE_LEN, currIdx);
//	gps_uart_msg_fillData(msg->correctionStationID, GPS_MSG_NMEA_CORRECTIONSTATID_LEN, &currIdx); // not used with gps device
	gps_uart_msg_fillData(msg->chksm, GPS_MSG_NMEA_CHECKSUM_LEN, currIdx);
}

void gps_uart_rxComplete(size_t size) {
	if (gps_uart_data[0] == '$') {
		rxSize = size;
		gps_uart_processStatus = GPS_UART_PROCESS_RX_PROCESS;
	} else {
		HAL_UARTEx_ReceiveToIdle_DMA(&huart4, gps_uart_data, GPS_NMEA_MAX_PARA_LEN);
	}
}

void gps_msg_process() {

}

void write_GpsData(GpsPosition* data) {
	osSemaphoreAcquire(&writingGpsMutex_attributes, HAL_MAX_DELAY);

	/*
	 * do the writing of the data
	 */
	osSemaphoreRelease(&writingGpsMutex_attributes);
}

void read_GpsData(GpsPosition* ret) {
	osSemaphoreAcquire(&writingGpsMutex_attributes, HAL_MAX_DELAY);
	/*
	 * do the reading of the data
	 */
	ret->pos =
	osSemaphoreRelease(&writingGpsMutex_attributes);
}
