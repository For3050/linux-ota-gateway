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
#include <stdio.h>  
#include <stdlib.h>  
#include <getopt.h>  
#include <fcntl.h>  
#include <sys/ioctl.h>  
#include <linux/types.h>  
#include <linux/spi/spidev.h>    

#include <linux/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <string.h> /*  */
#include <errno.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>


/* for thread */
#include<pthread.h>
#include<semaphore.h>

#include "header-files/spi_api.h"

/**********************************************************************************/
#define SERVER_THREAD_COUNT 2
#define SPI_READ_BUF_LEN  16
#define SPI_WRITE_BUF_LEN  140

#define OTA_BLOCK_SIZE 128

#define OTA_START_COMMAND 0x01
#define OTA_SEND_COMMAND 0x02
#define OTA_REQUEST_COMMAND 0x03
/**********************************************************************************/
void* monitor_cc2650_thread(void* arg_info);
void* process_request_thread(void* arg_info);

/**********************************************************************************/
typedef void*(*THREAD_FUNC)(void*);

typedef struct ARG_INFO
{
	int spi_fd;
	FILE* fd;
}arg_info_t, *arg_info_pt;

arg_info_t arg_info;

struct image_msg
{
	int command;  //command = 1 for ota;
	int ota_target;  //target = 0x01 for smart controller;
	int payload_size;  
	uint32_t image_size;  //unit: byte
}image_msg_t, *image_msg_pt;


typedef struct S_C_FRAME //server to client
{
	int command;  	//command = 0x02 for OTA frame;
	int frame_len;
	uint16_t crc;
	uint32_t frame_count;
	uint8_t payload[OTA_BLOCK_SIZE];
}sc_frame_t, *sc_frame_pt;

typedef struct C_S_FRAME //client to server
{
	int command;   //command = 0x03 for CLIENT REQUEST;
	uint32_t frame_count;
}cs_frame_t, *cs_frame_pt;

/**********************************************************************************/
static int spi_read_len = 0;
static int spi_write_len = 0;
static uint8_t spi_read_buf[SPI_READ_BUF_LEN] = {0x00,};
static uint8_t spi_write_buf[SPI_WRITE_BUF_LEN] = {0x00,};

static pthread_t server_thread_id[SERVER_THREAD_COUNT] = {};
static THREAD_FUNC server_thread[SERVER_THREAD_COUNT] = {monitor_cc2650_thread, process_request_thread};
static pthread_cond_t process_request_cond = PTHREAD_COND_INITIALIZER;

/**********************************************************************************/
void main_exit(void)
{
	printf("gateway process exit...\n");
}
/**********************************************************************************/
void* monitor_cc2650_thread(void* arg_info)
{
	int spi_fd = (*(arg_info_pt)arg_info).spi_fd;
	int gpio_fd = 0;

	gpio_fd = gpio_init();

	if(gpio_fd < 0)
	{
		fprintf(stderr,"gpio init error!\n");
		pthread_exit(NULL);
	}

	gpio_set_value(gpio_fd, 2, 0);
	printf("shutdown cc2650 for 1s\n");
	sleep(1);
	gpio_set_value(gpio_fd, 2, 1);
	printf("monitor_cc2650_thread is running...\n");

	while(1)
	{
		if(1 == gpio_get_value(gpio_fd, 0))
		{
			memset(spi_read_buf, 0, sizeof(spi_read_buf));
			//spi_read_len = spi_read(spi_fd, spi_read_buf);
			pthread_cond_signal(&process_request_cond);
			printf("monitor_cc2650_thread:spi_irq happened!\n");
		}
		else
		{
			usleep(100);
		}
	}

	printf("%s pthread exit, TID=%d\n", __FUNCTION__, (int)pthread_self());
	return NULL;
}

/**********************************************************************************/
void* process_request_thread(void* arg_info)
{
	int spi_fd = (*(arg_info_pt)arg_info).spi_fd;
	FILE* fd = (*(arg_info_pt)arg_info).fd;
	uint32_t frame_count = 0;
	uint32_t off_addr = 0;

	int retval = 0;

	static pthread_mutex_t process_request_mutex = PTHREAD_MUTEX_INITIALIZER;
	printf("process_request_thread is running...\n");
	while(1)
	{
		pthread_mutex_lock(&process_request_mutex);
		pthread_cond_wait(&process_request_cond, &process_request_mutex);
		printf("process_request_thread is triggled!\n");

		/* 1, operations on share memory, eg.ring_buf */
		memset(spi_read_buf, 0, sizeof(spi_read_buf));

		spi_read(spi_fd, spi_read_buf);

		pthread_mutex_unlock(&process_request_mutex);

		/* 2, parse spi_read_buf[]; */
		if(spi_read_buf[0] == OTA_REQUEST_COMMAND)
		{
		printf("process_request_thread:recv request command!\n");
		frame_count = (spi_read_buf[1]<<24)||(spi_read_buf[2]<<16)\
				   ||(spi_read_buf[3]<<8)||(spi_read_buf[4]);

		off_addr = frame_count * OTA_BLOCK_SIZE;
		fseek(fd, off_addr, SEEK_SET);

		memset(spi_write_buf, 0, sizeof(spi_write_buf));
		spi_write_buf[0] = 0x02; //command = 0x02 for OTA FRAME;
		spi_write_buf[1] = OTA_BLOCK_SIZE + 6; //length
		spi_write_buf[2] = 0x00; //crc
		spi_write_buf[3] = 0x00; //crc
		spi_write_buf[4] = (frame_count>>24)&0xff;
		spi_write_buf[5] = (frame_count>>16)&0xff;
		spi_write_buf[6] = (frame_count>>8)&0xff;
		spi_write_buf[7] = (frame_count>>0)&0xff;

		fread(spi_write_buf+8, OTA_BLOCK_SIZE, 1, fd);

		/* 3, send to cc2650 by spi_write(); */
		spi_write(spi_fd, spi_write_buf, spi_write_buf[1]+2);
		}
		usleep(100);
	}

}

/**********************************************************************************/
int main(int argc, char *argv[])
{ 
	int pthread_ret, thread_index;
	static arg_info_t arg_info;
	uint32_t file_len = 0;

	if(argc != 1)
	{
		fprintf(stderr,"Usage:%s filename\a\n",argv[0]);
		return -1;
	}

	arg_info.fd = fopen("./ota-image-sensortag.bin", "rb");
	if(!(arg_info.fd))
	{
		fprintf(stderr, "can't open file %s", argv[1]);
		exit(-1);
	}

	arg_info.spi_fd = spi_init();
	if((arg_info.spi_fd) < 0)
	{
		printf("open spi error\n");  
		close(arg_info.spi_fd);
		return -1;
	}
	printf("spi init success.\n");

	/* create frontend threads */
	for(thread_index = 0; thread_index < SERVER_THREAD_COUNT; thread_index++)
	{
		pthread_ret = pthread_create(&server_thread_id[thread_index], NULL, server_thread[thread_index], (void*)&arg_info);
		if(pthread_ret != 0)
		{
			perror("create server thread failed.");
			return -1;
		}
	}

	/* 1,send ota command */
	fseek(arg_info.fd, 0, SEEK_END);
	file_len = ftell(arg_info.fd);
	printf("file_len = %ld\n", file_len);

	sleep(10);

	memset(spi_write_buf, 0, sizeof(spi_write_buf));
	spi_write_buf[0] = 0x01;  //command
	spi_write_buf[1] = 0x01;  //ota_target
	spi_write_buf[2] = 128;  //payload_size
	spi_write_buf[3] = (file_len>>24)&0xff;  
	spi_write_buf[4] = (file_len>>16)&0xff;  
	spi_write_buf[5] = (file_len>>8)&0xff;  
	spi_write_buf[6] = (file_len>>0)&0xff;  

	spi_write(arg_info.spi_fd, spi_write_buf, 7);
	printf("1,send OTA command!\n");

	atexit(main_exit);

	for(thread_index = 0; thread_index < SERVER_THREAD_COUNT; thread_index++)
	{
		pthread_join(server_thread_id[thread_index],NULL);
	}

	close(arg_info.spi_fd);  
	fclose(arg_info.fd);
	arg_info.fd = NULL;

	return 0;  
}

/**********************************************************************************/

