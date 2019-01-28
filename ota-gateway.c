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
#define SPI_READ_BUF_SIZE 16
#define SPI_WRITE_BUF_SIZE  80

#define PAYLOAD_SIZE 64

#define OTA_DOWNLOAD 		0x01
#define OTA_ERROR_FRAME 	0x02

/**********************************************************************************/
typedef struct ARG_INFO
{
	int spi_fd;
	FILE* fd;
}arg_info_t, *arg_info_pt;

arg_info_t arg_info;


#if 0
struct FRAME
{
	uint8_t func;
	uint16_t frame_id;
	uint16_t total_frame;
	uint8_t length;
	uint8_t payload[PAYLOAD_SIZE];
	uint16_t crc;
};

struct ACK
{
	uint8_t func;
	uint16_t frame_id;
	uint16_t crc;
};
#endif

static uint16_t frame_id = 0;  /* from 0 */
static uint16_t total_frame = 0; /* decieded by bytes and PAYLOAD_SIZE*/
static uint32_t off_addr = 0;  /* offset address from the HEAD  */
/**********************************************************************************/

/**********************************************************************************/
void* monitor_cc2650_thread(void* arg_info);
void* process_request_thread(void* arg_info);
typedef void*(*THREAD_FUNC)(void*);
/**********************************************************************************/

static int spi_read_len = 0;
static int spi_write_len = 0;
static uint8_t spi_read_buf[SPI_READ_BUF_SIZE] = {0x00,};
static uint8_t spi_write_buf[SPI_WRITE_BUF_SIZE] = {0x00,};

static pthread_t server_thread_id[SERVER_THREAD_COUNT] = {};
static THREAD_FUNC server_thread[SERVER_THREAD_COUNT] = {monitor_cc2650_thread, process_request_thread};
static pthread_cond_t process_request_cond = PTHREAD_COND_INITIALIZER;
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
	uint16_t request_frame = 0;

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
		if(spi_read_buf[0] == OTA_ERROR_FRAME)
		{
		printf("process_request_thread:recv request command!\n");
		request_frame = ((spi_read_buf[1]<<8) || (spi_read_buf[2]));
		printf("request frame_id:%ld\n", request_frame);

		off_addr = request_frame*PAYLOAD_SIZE;

		}
		usleep(100);
	}

}


/**********************************************************************************/
void main_exit(void)
{
	printf("gateway process exit...\n");
}

/**********************************************************************************/
int main(int argc, char *argv[])
{ 
	int pthread_ret, thread_index;
	static arg_info_t arg_info;
	uint32_t file_len = 0;
	uint32_t single_off_size = 0;  //normally equals to PAYLOAD_SIZE or less.
	//FILE* fd = (*(arg_info_pt)arg_info).fd;

	if(argc != 2)
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

	/* 1,parse file */
	fseek(arg_info.fd, 0, SEEK_END);
	file_len = ftell(arg_info.fd);
	printf("file_len = %ld\n", file_len);

	if(0 == (file_len%PAYLOAD_SIZE))
	{
		total_frame = (file_len/PAYLOAD_SIZE);
	}
	else
	{
		total_frame = (file_len/PAYLOAD_SIZE)+1;
	}
	printf("total_frame = %d\n", total_frame);

	sleep(10);

	/* 2,start OTA download */
	printf("Start OTA download!\n");

	for(off_addr = 0; off_addr <= file_len; off_addr += PAYLOAD_SIZE)
	{
		sleep(1);

		if((file_len - off_addr) > PAYLOAD_SIZE)
		{
			single_off_size = PAYLOAD_SIZE;
		}
		else
		{
			single_off_size = (file_len - off_addr);
		}
		frame_id = (off_addr/PAYLOAD_SIZE);
		printf("frame_id = %ld\t, off_addr = %ld\t, single_off_size = %ld\n", frame_id, off_addr, single_off_size);

		memset(spi_write_buf, 0, sizeof(spi_write_buf));
		fseek(arg_info.fd, off_addr, SEEK_SET);
		fread(spi_write_buf+6, 1, single_off_size, arg_info.fd);
		spi_write_buf[0] = OTA_DOWNLOAD; //FUNC
		spi_write_buf[1] = (uint8_t)(frame_id>>8); //high byte
		spi_write_buf[2] = (uint8_t)frame_id; 
		spi_write_buf[3] = (uint8_t)(total_frame>>8); 
		spi_write_buf[4] = (uint8_t)total_frame; 
		spi_write_buf[5] = single_off_size; 
		spi_write_buf[5+single_off_size+1] = 0x12;  //CRC
		spi_write_buf[5+single_off_size+2] = 0x34;  //CRC

		//spi write
		spi_write(arg_info.spi_fd, spi_write_buf, single_off_size+8);
		printf("frame_id %d\t send out!\n", frame_id);

	}
		sleep(5);
		printf("OTA DOWNLOAD FINISHED!\n");


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

