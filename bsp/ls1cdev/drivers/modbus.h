#ifndef __OPENLOONGSON_MODBUS_H
#define __OPENLOONGSON_MODBUS_H
#include <rtthread.h>


rt_uint8_t AM_Speed[10];
rt_uint8_t AM_Direct[10];
rt_uint8_t AM_temperature[10];
rt_uint8_t AM_humidity[10];
rt_uint8_t AM_pressure[10];


rt_uint16_t CRC16(rt_uint8_t *puchMsg, rt_uint16_t usDataLen);
void AM_uart_irqhandler(int IRQn, void *param);
rt_uint8_t get_ambuff(rt_uint8_t rebuff[],rt_uint16_t *p);

void Process_direct(rt_uint8_t Pro_BUFF[],rt_uint8_t AM_direct[],rt_int16_t *p);
void Process_speed(rt_uint8_t Pro_BUFF[],rt_uint8_t AM_speed[],rt_int16_t *p);
void Process_temperature(rt_uint8_t Pro_BUFF[],rt_uint8_t AM_temperature[],rt_int16_t *p);
void Process_humidity(rt_uint8_t Pro_BUFF[],rt_uint8_t AM_humidity[],rt_int16_t *p);
void Process_pressure(rt_uint8_t Pro_BUFF[],rt_uint8_t AM_pressure[],rt_int16_t *p);
short int change(short int temperature);
rt_uint16_t direct_change(rt_uint16_t wind_direct);
rt_uint16_t speed_change(rt_uint16_t wind_speed);






void clean_buff(void);
int AM_Send(char *p,int len);
void AM_Init(void);



#endif

