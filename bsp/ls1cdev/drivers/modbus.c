#include <rtthread.h>
#include <stdlib.h>  
#include "ls1c.h"
#include <drivers/pin.h>
#include "ls1c_public.h"
#include "ls1c_uart.h"
#include "ls1c_pin.h"  
#include "../drivers/drv_uart.h"
#include "../libraries/ls1c_delay.h"
#include "../libraries/ls1c_gpio.h"
#include "string.h"
#include "modbus.h" 
 





#define AM_BUFF_SIZE 1024
#define RD_GPIO 57
volatile rt_uint16_t uart_p=0;
rt_uint8_t AM_BUFF[AM_BUFF_SIZE];

/*crc校验*/
static const rt_uint8_t auchCRCHi[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
};
 
static const rt_uint8_t auchCRCLo[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
};


//获取CRC16校验值
//puchMsg:要校验的数组
//usDataLen:校验的长度
//返回值:CRC16码
rt_uint16_t CRC16(rt_uint8_t *puchMsg, rt_uint16_t usDataLen)
{
  rt_uint8_t uchCRCHi = 0xFF; // 高CRC字节初始化
  rt_uint8_t uchCRCLo = 0xFF; // 低CRC 字节初始化
  rt_uint32_t uIndex; // CRC循环中的索引
  while (usDataLen--) // 传输消息缓冲区
  {
    uIndex = uchCRCHi ^ *puchMsg++; // 计算CRC
    uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
    uchCRCLo = auchCRCLo[uIndex];
  }
  return (((rt_uint16_t)uchCRCLo << 8u) | uchCRCHi);
}


/*
*风速计中断函数
*/
void AM_uart_irqhandler(int IRQn, void *param)
{
	rt_uint8_t res;
	rt_uint32_t tick;
    ls1c_uart_t uartx = uart_irqn_to_uartx(IRQn);
    void *uart_base = uart_get_base(uartx);
   unsigned char iir = reg_read_8(uart_base + LS1C_UART_IIR_OFFSET); //接收8位数据
    // 判断是否为接收超时或接收到有效数据
    if ((IIR_RXTOUT & iir) || (IIR_RXRDY & iir))
    {
         while (LSR_RXRDY & reg_read_8(uart_base + LS1C_UART_LSR_OFFSET))
        {
		    AM_BUFF[uart_p]=reg_read_8(uart_base + LS1C_UART_DAT_OFFSET);
           // rt_kprintf("\r\nAMBUFF[%d]=%x\r\n",uart_p,AM_BUFF[uart_p]);
            uart_p++;
        }
	}
}

/*
*   获取数据和长度
*/
rt_uint8_t get_ambuff(rt_uint8_t rebuff[],rt_uint16_t *p)
{
    *p= uart_p; 
	int i=0;
    for(i=0;i<20;i++)
    {
       //rt_kprintf("\r\nAM_BUFF[%d]=0%X\r\n",i,AM_BUFF[i]);
        rebuff[i]=AM_BUFF[i];
    }
    return 1;
}
/*
*   风向计数据处理函数
*   输入：Pro_BUFF :需处理的数组,AM_speed[]:速度数据，*p数据长度
*   输出：无
*/

short int temperature_1;
rt_uint16_t humidity_1;
rt_uint16_t wind_direct;
rt_uint16_t direct_1;
rt_uint16_t wind_speed;
rt_uint16_t speed_1;
rt_uint16_t pressure_1;
rt_uint16_t humidity_1;






void Process_direct(rt_uint8_t Pro_BUFF[],rt_uint8_t AM_direct[],rt_int16_t *p)

{
    int i=0,j;
    rt_uint32_t CRC=0;
    for(i=0;i<20;i++)
    {
     if(Pro_BUFF[i]==1&&Pro_BUFF[i+1]==3)
     {
        //rt_kprintf("get AM 485 informations\r\n");
        *p=Pro_BUFF[i+2];
        rt_uint8_t CRC_BUFF[*p+3];
        //rt_kprintf("AM length =%d\r\n",*p);
        for(j=0;j<*p;j++)
            AM_direct[j]=Pro_BUFF[i+3+j];
        for(j=0;j<*p+3;j++)
        {
            CRC_BUFF[j]=Pro_BUFF[i+j];
            //rt_kprintf("%X\r\n",CRC_BUFF[j]);
        }
       CRC=CRC16(CRC_BUFF,7); 
      // rt_kprintf("crc=%X\r\n",CRC);

	   wind_direct=((AM_direct[2]<<8)|AM_direct[3]);
	   direct_1=direct_change(wind_direct);
	 //  rt_kprintf("AM_direct[7] =%d\r\n",AM_direct[2]);
	 //  rt_kprintf("AM_direct[6] =%d\r\n",AM_direct[3]);
	   rt_kprintf("wind_direct =%d\r\n",wind_direct);
     }
    }
}


/*
*   风速计数据处理函数
*   输入：Pro_BUFF :需处理的数组,AM_speed[]:速度数据，*p数据长度
*   输出：无
*/

void Process_speed(rt_uint8_t Pro_BUFF[],rt_uint8_t AM_speed[],rt_int16_t *p)
{
    int i=0,j;
    rt_uint32_t CRC=0;
    for(i=0;i<20;i++)
    {
     if(Pro_BUFF[i]==2&&Pro_BUFF[i+1]==3)
     {
        //rt_kprintf("get AM 485 informations\r\n");
        *p=Pro_BUFF[i+2];
        rt_uint8_t CRC_BUFF[*p+3];
        //rt_kprintf("AM length =%d\r\n",*p);
        for(j=0;j<*p;j++)
            AM_speed[j]=Pro_BUFF[i+3+j];
        for(j=0;j<*p+3;j++)
        {
            CRC_BUFF[j]=Pro_BUFF[i+j];
           // rt_kprintf("%X\r\n",CRC_BUFF[j]);
        }
       CRC=CRC16(CRC_BUFF,7); 
       //rt_kprintf("crc=%X\r\n",CRC);

	   wind_speed=((AM_speed[2]<<8)|AM_speed[3])*10;
	   speed_1=speed_change(wind_speed);
	  // rt_kprintf("AM_speed[7] =%d\r\n",AM_speed[2]);
	   //rt_kprintf("AM_speed[6] =%d\r\n",AM_speed[3]);
	   rt_kprintf("wind_speed =%d\r\n",wind_speed);
     }
    }
}

/*
*   温度计数据处理函数
*   输入：Pro_BUFF :需处理的数组,AM_speed[]:速度数据，*p数据长度
*   输出：无
*/
void Process_temperature(rt_uint8_t Pro_BUFF[],rt_uint8_t AM_temperature[],rt_int16_t *p)
{
    int i=0,j;
    rt_uint32_t CRC=0;
    for(i=0;i<20;i++)
    {
     if(Pro_BUFF[i]==3&&Pro_BUFF[i+1]==3)
     {
        //rt_kprintf("get AM 485 informations\r\n");
        *p=Pro_BUFF[i+2];
        rt_uint8_t CRC_BUFF[*p+3];
        //rt_kprintf("AM length =%d\r\n",*p);
        for(j=0;j<*p;j++)
            AM_temperature[j]=Pro_BUFF[i+3+j];
        for(j=0;j<*p+3;j++)
        {
            CRC_BUFF[j]=Pro_BUFF[i+j];
            //rt_kprintf("%X\r\n",CRC_BUFF[j]);
        }
       CRC=CRC16(CRC_BUFF,7); 
       //rt_kprintf("crc=%X\r\n",CRC);
	   humidity_1=((AM_temperature[0]<<8)|AM_temperature[1]);
	   temperature_1=((AM_temperature[2]<<8)|AM_temperature[3]);
	   temperature_1=change(temperature_1);
	   temperature_1=temperature_1/10;
	   humidity_1=humidity_1/10;
	   rt_kprintf("temperature_1 =%d\r\n",temperature_1);
	   rt_kprintf("humidity_1 =%d\r\n",humidity_1);
     }
    }
}



void Process_humidity(rt_uint8_t Pro_BUFF[],rt_uint8_t AM_humidity[],rt_int16_t *p)
{
    int i=0,j;
    rt_uint32_t CRC=0;
    for(i=0;i<20;i++)
    {
     if(Pro_BUFF[i]==1&&Pro_BUFF[i+1]==3)
     {
        //rt_kprintf("get AM 485 informations\r\n");
        *p=Pro_BUFF[i+2];
        rt_uint8_t CRC_BUFF[*p+3];
        //rt_kprintf("AM length =%d\r\n",*p);
        for(j=0;j<*p;j++)
            AM_humidity[j]=Pro_BUFF[i+3+j];
        for(j=0;j<*p+3;j++)
        {
            CRC_BUFF[j]=Pro_BUFF[i+j];
            //rt_kprintf("%X\r\n",CRC_BUFF[j]);
        }
       CRC=CRC16(CRC_BUFF,7); 
       rt_kprintf("crc=%X\r\n",CRC);
	   rt_uint16_t humidity;
	   humidity_1=((AM_humidity[2]<<8)|AM_humidity[3]);
	  //rt_kprintf("AM_humidity[7] =%d\r\n",AM_humidity[2]);
	   //rt_kprintf("AM_direct[6] =%d\r\n",AM_humidity[3]);
	   rt_kprintf("humidity =%d\r\n",humidity_1);
     }
    }
}
/*
*   大气压计数据处理函数
*   输入：Pro_BUFF :需处理的数组,AM_speed[]:速度数据，*p数据长度
*   输出：无
*/
void Process_pressure(rt_uint8_t Pro_BUFF[],rt_uint8_t AM_pressure[],rt_int16_t *p)
{
    int i=0,j;
    rt_uint32_t CRC=0;
    for(i=0;i<20;i++)
    {
     if(Pro_BUFF[i]==6&&Pro_BUFF[i+1]==3)
     {
        //rt_kprintf("get AM 485 informations\r\n");
        *p=Pro_BUFF[i+2];
        rt_uint8_t CRC_BUFF[*p+3];
        //rt_kprintf("AM length =%d\r\n",*p);
        for(j=0;j<*p;j++)
            AM_pressure[j]=Pro_BUFF[i+3+j];
        for(j=0;j<*p+3;j++)
        {
            CRC_BUFF[j]=Pro_BUFF[i+j];
            //rt_kprintf("%X\r\n",CRC_BUFF[j]);
        }
       CRC=CRC16(CRC_BUFF,7); 
       //rt_kprintf("crc=%X\r\n",CRC);
	   
	   pressure_1=((AM_pressure[0]<<8)|AM_pressure[1]);
	   rt_kprintf("pressure =%d\r\n",pressure_1);
     }
    }
}


void clean_buff(void)
{
       rt_uint16_t i=AM_BUFF_SIZE+1;
       uart_p=0;
       while(i)
       {
            AM_BUFF[i--]=0;
       }
}
/*
* 485发送
*/
int AM_Send(char *p,int len)
{
	int i=0;
     //rt_thread_delay(200);
     gpio_set(RD_GPIO, gpio_level_high); //485进入发送状态
     rt_thread_delay(200);
     for(i=0;i<len;i++)
	 {
         uart_putc(4, *p);
		 //rt_kprintf("%X\r\n",*p);
         p++;
     }
	 rt_thread_delay(1);
     gpio_set(RD_GPIO, gpio_level_low); //485进入接收状态
     rt_thread_delay(50);
     gpio_set(RD_GPIO, gpio_level_high); //485进入发送状态
     return 0;
}

short int change(short int temperature)
{
	short int temperature_value;
	if(temperature&0x8000)temperature_value=(~(temperature_value&0x7FFF)+1)+1;
	if((temperature&0x8000)==0)temperature_value=temperature;
	return temperature_value;
}


/*
*   风速计串口初始化
*   RX,TX:GPIO4,5
*   波特率4800
*   输入输出：无
*/
void  AM_Init(void)
{
	// 调试串口信息
	ls1c_uart_info_t AM_uart_info = {0};	
	int dat;

    unsigned int tx_gpio = 1;
    unsigned int rx_gpio = 0;
    // 初始化RE/DE_GPIO  
    gpio_init(RD_GPIO, gpio_mode_output);
    gpio_set(RD_GPIO, gpio_level_low);       // 进入接收状态
    // 设置复用
    pin_set_remap(tx_gpio, PIN_REMAP_FOURTH);
    pin_set_remap(rx_gpio, PIN_REMAP_FOURTH);
    
    // 初始化相关寄存器
    AM_uart_info.UARTx = LS1C_UART3;
    AM_uart_info.baudrate = 4800;
    AM_uart_info.rx_enable = TRUE;  
    uart_init(&AM_uart_info);
	
	rt_hw_interrupt_umask(LS1C_UART3_IRQ);		
    
    rt_hw_interrupt_install(LS1C_UART3_IRQ, AM_uart_irqhandler, RT_NULL, "UART3");

}

/*
*   风向判断
*   输入：风速
* 
*   输出：风力等级
*/
rt_uint16_t speed_change(rt_uint16_t wind_speed)
{
	rt_uint16_t speed_1;
	if( 0<=wind_speed<2)speed_1=0,rt_kprintf("无风\r\n");
	if( 2<=wind_speed&&wind_speed<15)speed_1=1,rt_kprintf("软风\r\n");
	if( 15<=wind_speed&&wind_speed<33)speed_1=2,rt_kprintf("轻风\r\n");
	if( 33<=wind_speed&&wind_speed<54)speed_1=3,rt_kprintf("微风\r\n");
	if( 54<=wind_speed&&wind_speed<79)speed_1=4,rt_kprintf("和风\r\n");
	if( 79<=wind_speed&&wind_speed<107)speed_1=5,rt_kprintf("清风\r\n");
	if( 107<=wind_speed&&wind_speed<138)speed_1=6,rt_kprintf("强风\r\n");
	if( 138<=wind_speed&&wind_speed<171)speed_1=7,rt_kprintf("劲风\r\n");
	if( 171<=wind_speed&&wind_speed<207)speed_1=8,rt_kprintf("大风\r\n");
	if( 207<=wind_speed&&wind_speed<244)speed_1=9,rt_kprintf("烈风\r\n");
	if( 244<=wind_speed&&wind_speed<284)speed_1=10,rt_kprintf("狂风\r\n");
	if( 284<=wind_speed&&wind_speed<326)speed_1=11,rt_kprintf("暴风\r\n");
	if( 326<=wind_speed&&wind_speed<369)speed_1=12,rt_kprintf("台风（一级飓风）\r\n");
	if( 369<=wind_speed&&wind_speed<414)speed_1=13,rt_kprintf("台风（一级飓风）\r\n");
	if( 414<=wind_speed&&wind_speed<461)speed_1=14,rt_kprintf("强台风（二级飓风）\r\n");
	if( 461<=wind_speed&&wind_speed<509)speed_1=15,rt_kprintf("强台风（三级飓风）\r\n");
	if( 509<=wind_speed&&wind_speed<560)speed_1=16,rt_kprintf("超强台风（三级飓风）\r\n");
	if( 560<=wind_speed&&wind_speed<612)speed_1=17,rt_kprintf("超级台风（四级飓风）\r\n");
	if( 615<=wind_speed)speed_1=18,rt_kprintf("超级台风（五级飓风）\r\n");
	return speed_1;
}

/*
*   风向判断
*   输入：角度
* 
*   输出：风向0-7 北-西北
*/
rt_uint16_t direct_change(rt_uint16_t wind_direct)
{
	rt_uint16_t direct_1;
	if( 0==wind_direct)direct_1=0,rt_kprintf("北风\r\n");
	if( (0<wind_direct)&&(wind_direct<90))direct_1=1,rt_kprintf("东北风\r\n");
	if( 90==wind_direct)direct_1=2,rt_kprintf("东风\r\n");
	if( (90<wind_direct)&&(wind_direct<180))direct_1=3,rt_kprintf("东南风\r\n");
	if( 180==wind_direct)direct_1=4,rt_kprintf("南风\r\n");
	if( (180<wind_direct)&&(wind_direct<270))direct_1=5,rt_kprintf("西南风\r\n");
	if( 270==wind_direct)direct_1=6,rt_kprintf("西风\r\n");
	if( (270<wind_direct)&&(wind_direct<360))direct_1=7,rt_kprintf("西北风\r\n");
	return direct_1;
}
