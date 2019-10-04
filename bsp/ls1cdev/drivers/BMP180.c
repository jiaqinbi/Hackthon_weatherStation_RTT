#include <rtthread.h>
#include <drivers/pin.h>
#include "ls1c.h"  
#include "ls1c_i2c.h"  
#include "ls1c_pin.h"
#include "ls1c_gpio.h"
#include "ls1c_delay.h"
#include "BMP180.h"  

#define BMP180_I2C_BUS_NAME                ("i2c2") // 注意与i2c bus初始化函数中的bus name保持一致
struct rt_i2c_bus_device *bmp180_i2c_bus = RT_NULL;
int bmp180_addr = 0xEE >> 1;                       // 地址前7位

long  result_UT=0;
long  result_UP=0;

short ac1;
short ac2; 
short ac3; 
unsigned short ac4;
unsigned short ac5;
unsigned short ac6;
short b1;
short b2;
short mb;
short mc;
short md;

/*
 * 从指定地址读出一个字节
 * @read_addr 地址
 */
unsigned char bmp180_read_byte(unsigned char read_addr)
{
    struct rt_i2c_msg msgs[2];
    unsigned char data;
    unsigned char reg_addr[1];
    
    reg_addr[0] = read_addr;
    
    msgs[0].addr    = bmp180_addr;
    msgs[0].flags   = RT_I2C_WR;
    msgs[0].buf     = reg_addr;
    msgs[0].len     = 1;

    msgs[1].addr    = bmp180_addr;
    msgs[1].flags   = RT_I2C_RD;
    msgs[1].buf     = &data;
    msgs[1].len     = 1;
    rt_i2c_transfer(bmp180_i2c_bus, msgs, 2);

    return data;
}

/*
 * 在指定地址写入一个字节的数据
 * @write_addr 地址
 * @data 待写入的数据
 */
void bmp180_write_byte(unsigned char write_addr, unsigned char data)
{
    struct rt_i2c_msg msg[1] = {0};
    unsigned char buf[2] = {0};
    
    buf[0] = write_addr;
    buf[1] = data;
    
    msg[0].addr    = bmp180_addr;
    msg[0].flags   = RT_I2C_WR;
    msg[0].buf     = buf;
    msg[0].len     = 2;
    rt_i2c_transfer(bmp180_i2c_bus, msg, 1);
}

/* 连续读两个字节 */
rt_uint16_t Multiple_read(rt_uint8_t ST_Address)
{
	rt_uint16_t _data;
	rt_uint8_t recv_buff[2] = {0};

	recv_buff[0] = bmp180_read_byte(ST_Address);
	recv_buff[1] = bmp180_read_byte(ST_Address + 1);

	rt_thread_delay(1);
	
	_data = (recv_buff[0] << 8)+recv_buff[1];
	return _data;
}

/* 读取温度 */
rt_uint16_t bmp180ReadTemp(void)
{
  bmp180_write_byte(0xF4,0x2E);	          // write register address
  rt_thread_delay(1);	// max time is 4.5ms
  return Multiple_read(0xF6);
}

/* 读取压力 */
rt_uint16_t bmp180ReadPressure(void)
{
  rt_uint16_t pressure = 0;

  bmp180_write_byte(0xF4,0x34);	          // write register address
  rt_thread_delay(1);    	                  // max time is 4.5ms

  pressure = Multiple_read(0xF6);
  pressure &= 0x0FFFF;

  return pressure;
}

/* 初始化BMP180 */
void Init_BMP180()
{
	// find设备
    bmp180_i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(BMP180_I2C_BUS_NAME);
    if (RT_NULL == bmp180_i2c_bus)
    {
        rt_kprintf("[%s] no i2c device -- bmp180!\n", __FUNCTION__);
        return ;
    }
	ac1 = Multiple_read(0xAA);
	ac2 = Multiple_read(0xAC);
	ac3 = Multiple_read(0xAE);
	ac4 = Multiple_read(0xB0);
	ac5 = Multiple_read(0xB2);
	ac6 = Multiple_read(0xB4);
	b1 =  Multiple_read(0xB6);
	b2 =  Multiple_read(0xB8);
	mb =  Multiple_read(0xBA);
	mc =  Multiple_read(0xBC);
	md =  Multiple_read(0xBE);
	// rt_kprintf("ac1 = %d\n",ac1);
	// rt_kprintf("ac2 = %d\n",ac2);
	// rt_kprintf("ac3 = %d\n",ac3);
	// rt_kprintf("ac4 = %d\n",ac4);
	// rt_kprintf("ac5 = %d\n",ac5);
	// rt_kprintf("ac6 = %d\n",ac6);
	// rt_kprintf("b1 = %d\n",b1);
	// rt_kprintf("b2 = %d\n",b2);
	// rt_kprintf("mb = %d\n",mb);
	// rt_kprintf("mc = %d\n",mc);
	// rt_kprintf("md = %d\n",md);
}

/* 转化为实际温度和压力 */
void bmp180Convert()
{
  unsigned int ut;
  unsigned long up;
  long x1, x2, b5, b6, x3, b3, p;
  unsigned long b4, b7;

  ut = bmp180ReadTemp();	   // 读取温度
  up = bmp180ReadPressure();  // 读取压强    ;	
  //*************
  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long) mc << 11) / (x1 + md);
  b5 = x1 + x2;
  result_UT = ((b5 + 8) >> 4)-40;
  //*************		
  b6 = b5 - 4000;
                           // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;	
                           // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
  p = (b7<<1)/b4;
  else
  p = (b7/b4)<<1;
  
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  result_UP = p+((x1 + x2 + 3791)>>4);
}