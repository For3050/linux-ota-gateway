/* 
 * 
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation; either version 2 of the License. 
 * 
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include 
 */  
  
#include <stdint.h>  
#include <unistd.h>
#include <stdlib.h>  
#include <getopt.h>  
#include <fcntl.h>  
#include <sys/ioctl.h>  
#include <linux/types.h>  
#include <linux/spi/spidev.h>    

#include <linux/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <stdio.h>

#define GPIO_U_IOCTL_BASE 'x'
#define GPIOC_OPS   _IOWR(GPIO_U_IOCTL_BASE,0,int)

#define GPIO_SET(no,state) 	( no | (state << 31))
#define GPIO_GET(val)		(val >> 31)

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))  

int gpio_init()
{
	int gpio;
	
	gpio = open("/dev/gpio",O_RDWR);
	if(gpio < 0)
	{
		perror("open gpio error");
		exit(1);
	}
	return gpio;
}

void gpio_set_value(int fd,int gpio_no,int state)
{
	unsigned long val;
	val = (!!state << 31) | gpio_no;

	if(ioctl(fd,GPIOC_OPS,&val) < 0){
		perror("ioctl");
	}
}

int  gpio_get_value(int fd,int gpio_no)
{
	unsigned long val = gpio_no;
	if(ioctl(fd,GPIOC_OPS,&val) < 0){
		perror("ioctl");
	}
	return val;
}




 
static void pabort(const char *s)  
{  
    perror(s);  
    abort();  
}  
  
static const char *device = "/dev/spidev0.0";  
static uint8_t mode;  
static uint8_t bits = 8;    /* spi transfer data width */
static uint32_t speed = 4000000;  /* spi clock rate */




void spi_write_data(int fd, uint8_t* tx_buf, const uint8_t len)
{
	int ret;  
	
    struct spi_ioc_transfer tr = {  
        .tx_buf = (unsigned long)tx_buf,  
        .rx_buf = (unsigned long)NULL,  
        .len = len, 
        .speed_hz = speed,  
        .bits_per_word = bits,  
    };  
  
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);  
    if (ret < 0)  
        pabort("can't send spi message");  
}






void spi_read_data(int fd, uint8_t* rx_buf, uint8_t len)
{
	int ret;  
    struct spi_ioc_transfer tr = {  
        .tx_buf = (unsigned long)NULL,  
        .rx_buf = (unsigned long)rx_buf,  
        .len = len, 
        .speed_hz = speed,  
        .bits_per_word = bits,  
    };  
  
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);  
    if (ret < 0)  
        pabort("can't read spi message");  
}


void spi_write(int fd, const uint8_t* tx_buf,  const uint8_t len)
{
	uint8_t wDataLen = len;
	spi_write_data(fd, &wDataLen, 1);
	spi_write_data(fd, tx_buf, len);
}
  
uint8_t spi_read(int fd, uint8_t* rx_buf)
{
	uint8_t dataLen = 0;
	spi_read_data(fd, &dataLen, 1);
	spi_read_data(fd, rx_buf, dataLen);
	return dataLen;
}

int spi_init(void)
{
	int ret = 0;  
    int fd;  
  
    fd = open(device, O_RDWR);  
    if (fd < 0)
	{
		pabort("can't open device");  
		return fd;
	}
        
  
    /* 
     * spi mode 
     */  
    ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);  
    if (ret == -1)  
	{
		pabort("can't set spi mode");  
		return ret;
	}
        
  
    ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);  
    if (ret == -1)  
	{
		pabort("can't get spi mode");  
		return ret;
	}  
  
    /* 
     * bits per word 
     */  
    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);  
    if (ret == -1)  
	{
		pabort("can't set bits per word");  
		return ret;
	}
        
  
    ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);  
    if (ret == -1)  
	{
		pabort("can't get bits per word");  
		return ret;
	}
        
  
    /* 
     * max speed hz 
     */  
    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);  
    if (ret == -1)  
	{
		pabort("can't set max speed hz");
		return ret;
	}
          
  
    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);  
    if (ret == -1)  
	{
		pabort("can't get max speed hz"); 
		return ret;
	}
         
	
	return fd;
}



