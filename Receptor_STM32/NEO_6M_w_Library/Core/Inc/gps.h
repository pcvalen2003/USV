/*
 * gps.h
 *
 *  Created on: Jun 10, 2025
 *      Author: Adriano
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#define GPS_DEBUG	0
#define	GPS_USART	&huart1
#define GPSBUFSIZE  128       // GPS buffer size

typedef struct{

    // calculated values
    float dec_longitude;
    float dec_latitude;
    float altitude_ft;

    // GGA - Global Positioning System Fixed Data
    float nmea_longitude;
    float nmea_latitude;
    float utc_time;
    char ns, ew;
    int lock;
    int satelites;
    float hdop;
    float msl_altitude;
    char msl_units;

    // RMC - Recommended Minimmum Specific GNS Data
    char rmc_status;
    float speed_k;
    float course_d;
    int date;

    // GLL
    char gll_status;

    // VTG - Course over ground, ground speed
    float course_t; // ground speed true
    char course_t_unit;
    float course_m; // magnetic
    char course_m_unit;
    char speed_k_unit;
    float speed_km; // speek km/hr
    char speed_km_unit;
} GPS_t;

extern uint8_t rx_data;
extern uint8_t rx_buffer[GPSBUFSIZE];
extern uint8_t rx_index;
extern uint8_t rx_reception;
extern uint8_t rx_print_buffer[GPSBUFSIZE];

extern GPS_t GPS;

//#if (GPS_DEBUG == 1)
//void GPS_print(char *data);
//#endif

void GPS_Init();
void GSP_USBPrint(char *data);
void GPS_print_val(char *data, int value);
void GPS_UART_CallBack();
int GPS_validate(char *nmeastr);
void GPS_parse(char *GPSstrParse);
float GPS_nmea_to_dec(float deg_coord, char nsew);
void GPS_Debug_Print(char* data);

#endif /* INC_GPS_H_ */
