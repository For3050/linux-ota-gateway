CFLAGS= -O2 --static

LIB= -lpthread

CC=/opt/freescale/usr/local/gcc-4.6.2-glibc-2.13-linaro-multilib-2011.12/fsl-linaro-toolchain/bin/arm-linux-gcc

###############################################
#holliot_gateway_test_thread: holliot_gateway_test.o backend_to_frontend.o frontend_to_backend.o  spi_api.o linklist.o
#	$(CC) $(CFLAGS)  -o holliot_gateway_test_thread  holliot_gateway_test.o  spi_api.o  backend_to_frontend.o frontend_to_backend.o linklist.o $(LIB)

#clean: 
#   rm -f holliot_gateway_test holliot_gateway_test.o  spi_api.o linklist.o backend_to_frontend.o frontend_to_backend.o 
###############################################
#new_board_spi_test: new_board_spi_test.o spi_api.o 
#	$(CC) $(CFLAGS)  -o new_board_spi_test  new_board_spi_test.o  spi_api.o  $(LIB)
#clean: 
#	rm -f new_board_spi_test new_board_spi_test.o spi_api.o
###############################################
ota-gateway: ota-gateway.o spi_api.o 
	$(CC) $(CFLAGS)  -o ota-gateway  ota-gateway.o  spi_api.o  $(LIB)
clean: 
	rm -f *.o
	rm -f ota-gateway 

###############################################
