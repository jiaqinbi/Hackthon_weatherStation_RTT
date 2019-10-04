#include <rtthread.h>

#include "modbus.h"
#include <rtthread.h>
#include <stdlib.h>  
#include "ls1c.h"
#include <drivers/pin.h>
#include "ls1c_public.h"
#include "ls1c_uart.h"
#include "ls1c_pin.h"  
#include "../drivers/drv_uart.h"
#include "../drivers/modbus.h"
#include "../libraries/ls1c_delay.h"
#include "string.h"


#define DEVICE_ID	           "529040114"	
#define API_KEY		           "fp2tCoI51h=QjxkoEDrKhUcrTbI="
#define DATA_temperature	   "temperature"
#define DATA_humidity	       "humidity"
#define DATA_air_pressure	   "air_pressure"
#define DATA_wind_speed	       "wind_speed"
#define DATA_wind_power	       "wind_power"
#define DATA_wind_angle	       "wind_angle"
#define DATA_wind_direction	   "wind_direction"

extern rt_err_t onenet_sample(void);
extern short int temperature_1;
extern rt_uint16_t humidity_1;
extern rt_uint16_t wind_direct;
extern rt_uint16_t direct_1;
extern rt_uint16_t wind_speed;
extern rt_uint16_t speed_1;
extern rt_uint16_t pressure_1;
extern rt_uint16_t humidity_1;



rt_uint8_t direct[8]={0x01,0x03,0x00,0x00,0x00,0x02,0xC4,0x0B};
rt_uint8_t speed[8]={0x02,0x03,0x00,0x00,0x00,0x02,0xC4,0x38};
rt_uint8_t temperature[8]={0x03,0x03,0x00,0x00,0x00,0x02,0xC5,0xE9};  
rt_uint8_t humidity[8]={0x01,0x03,0x00,0x00,0x00,0x02,0xC4,0x0B};
rt_uint8_t pressure[8]={0x06,0x03,0x00,0x00,0x00,0x01,0x85,0xBD};
rt_uint8_t receve[20]; 
rt_uint8_t AM_Speed[10];
rt_uint8_t AM_Direct[10];
rt_uint8_t AM_temperature[10];
rt_uint8_t AM_humidity[10];
rt_uint8_t AM_pressure[10];






void direct_cal(void)
{
    rt_uint16_t i=0,p=0;
    AM_Send(direct,8);
	get_ambuff(receve,&i);
    clean_buff();
	//rt_kprintf("the length is %d\r\n",i);
    Process_direct(receve,AM_Direct,&p);
    //for(i=0;i<p;i++)rt_kprintf("AM_Direct=%X\r\n",AM_Direct[i]);
	delay_ms(500);

}


void speed_cal(void)
{
    rt_uint16_t i=0,p=0;
    AM_Send(speed,8);
	get_ambuff(receve,&i);
    clean_buff();
	//rt_kprintf("the length is %d\r\n",i);
    Process_speed(receve,AM_Speed,&p);
    //for(i=0;i<p;i++)rt_kprintf("AM_SPEED=%X\r\n",AM_Speed[i]);
	delay_ms(500);

}

void temperature_cal(void)
{
    rt_uint16_t i=0,p=0;
    AM_Send(temperature,8);
	get_ambuff(receve,&i);
    clean_buff();
	//rt_kprintf("the length is %d\r\n",i);
    Process_temperature(receve,AM_temperature,&p);
    //for(i=0;i<p;i++)rt_kprintf("AM_temperature=%X\r\n",AM_temperature[i]);
	delay_ms(500);

}

void pressure_cal(void)
{
    rt_uint16_t i=0,p=0;
    AM_Send(pressure,8);
	get_ambuff(receve,&i);
    clean_buff();
	//rt_kprintf("the length is %d\r\n",i);
    Process_pressure(receve,AM_pressure,&p);
    //for(i=0;i<p;i++)rt_kprintf("AM_pressure=%X\r\n",AM_pressure[i]);
	delay_ms(500);

}





void AM_thread_entry(void* parameter)
{ 
	AM_Init();
	long timex=0;				//connect and post
	onenet_sample();
	connect_onenet_http_device(); 
	 while(1)
	{	
      
	  direct_cal();
	  speed_cal();
	  temperature_cal();
	  pressure_cal();


	 //转换气压温度（bmp180 driver）
		if(timex%30==0) 		 //强制 connect again（待改）
		{
		connect_onenet_http_device(); 
		}	

		post_data_stream_to_onenet(DEVICE_ID, API_KEY, DATA_temperature,temperature_1*10);
		rt_kprintf("temperature_1=%d\r\n",temperature_1);
		post_data_stream_to_onenet(DEVICE_ID, API_KEY, DATA_humidity,humidity_1*10);
		rt_kprintf("humidity_1=%d\r\n",humidity_1);
		post_data_stream_to_onenet(DEVICE_ID, API_KEY, DATA_air_pressure,pressure_1);
		rt_kprintf("pressure_1=%d\r\n",pressure_1);
		post_data_stream_to_onenet(DEVICE_ID, API_KEY, DATA_wind_speed,wind_speed);
		rt_kprintf("wind_speed=%d\r\n",wind_speed*10);
		post_data_stream_to_onenet(DEVICE_ID, API_KEY, DATA_wind_power,speed_1*10);
		rt_kprintf("speed_1=%d\r\n",speed_1);
		post_data_stream_to_onenet(DEVICE_ID, API_KEY, DATA_wind_angle,wind_direct*10);
		rt_kprintf("wind_direct=%d\r\n",wind_direct);
		post_data_stream_to_onenet(DEVICE_ID, API_KEY, DATA_wind_direction,direct_1*10);
		rt_kprintf("direct_1=%d\r\n",direct_1);
	 
		 rt_thread_delay(50);
		 timex++; 
	}


}


int main(int argc, char** argv)
{   

	rt_err_t result;
	
    rt_thread_t thread = rt_thread_create("VMS_3000",
						AM_thread_entry, RT_NULL,
						1024, 25, 7);
	    if (thread!= RT_NULL)
        rt_thread_startup(thread); 

    return 0;
}

