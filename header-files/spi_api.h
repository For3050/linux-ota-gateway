/* 
 * 
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation; either version 2 of the License. 
 * 
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include 
 */  
  
#ifndef		__SPI_API_HEADER__
#define		__SPI_API_HEADER__
#include <stdint.h>


int spi_init(void);

uint8_t spi_read(int fd, uint8_t* rx_buf);

void spi_write(int fd, uint8_t* tx_buf,  const uint8_t len);

#endif
