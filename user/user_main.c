
#include <ets_sys.h>
#include <osapi.h>
#include <os_type.h>
#include <gpio.h>
#include "user_config.h"
#include "driver/spi.h"
#include "driver/spi_register.h"
#include "osapi.h"
#include "user_interface.h"
#include "espconn.h"
#include "mem.h"
#include "driver/uart.h"
#include "driver/gpio16.h"
//SPI
#define SPI_CLK_PRE_DIV 4
#define SPI_CLK_CNT_DIV 4

//STATION
#define SSID "giggidygiggidy"
#define PASSWORD "BEEFBEEF02"

//AP
#define WIFI_APSSID "prosciuttobayvid"

//SOCKET
#define PORT 6000


//CPU
#define SYS_CPU_160MHz 160


extern uint8_t pin_num[GPIO_PIN_NUM];

const char *gpio_type_desc[] =
{
	    "GPIO_PIN_INTR_DISABLE (DISABLE INTERRUPT)",
	    "GPIO_PIN_INTR_POSEDGE (UP)",
	    "GPIO_PIN_INTR_NEGEDGE (DOWN)",
	    "GPIO_PIN_INTR_ANYEDGE (BOTH)",
	    "GPIO_PIN_INTR_LOLEVEL (LOW LEVEL)",
	    "GPIO_PIN_INTR_HILEVEL (HIGH LEVEL)"
};

static struct espconn pUdpServer;
static struct espconn pTcpServer;
extern int ets_uart_printf(const char *fmt, ...);
int (*console_printf)(const char *fmt, ...) = ets_uart_printf;

volatile uint8 ready = 1;

void ICACHE_FLASH_ATTR InitStation(void)
{
	char ssid[32] = SSID;
	char password[64] = PASSWORD;
	struct station_config stationConf;

    wifi_set_opmode(STATION_MODE);
	stationConf.bssid_set = 0; //need not check MAC address of AP
	os_memcpy(&stationConf.ssid, ssid, 32);
	os_memcpy(&stationConf.password, password, 64);
	wifi_station_set_config(&stationConf);
}

void ICACHE_FLASH_ATTR InitServer(void)
{
	struct softap_config apConfig;
	struct ip_info ipinfo;
	char ssid[33];
	char password[33];
	char macaddress[17];
	char info[150];

	wifi_set_opmode(SOFTAP_MODE);
	IP4_ADDR(&ipinfo.ip, 192, 168, 4, 1);
	IP4_ADDR(&ipinfo.gw, 192, 168, 4, 1);
	IP4_ADDR(&ipinfo.netmask, 255, 255, 255, 0);
	wifi_set_ip_info(SOFTAP_IF, &ipinfo);
	wifi_softap_get_config(&apConfig);
	os_memset(apConfig.ssid, 0, sizeof(apConfig.ssid));
	os_sprintf(ssid, "%s", WIFI_APSSID);
	os_memcpy(apConfig.ssid, ssid, os_strlen(ssid));
	apConfig.authmode = AUTH_OPEN;
	apConfig.channel = 7;
	apConfig.max_connection = 255;
	apConfig.ssid_hidden = 0;
	wifi_softap_set_config(&apConfig);
}

void ICACHE_FLASH_ATTR SpiInit(void)
{
	spi_init_gpio(HSPI, SPI_CLK_USE_DIV);
	spi_clock(HSPI, SPI_CLK_PRE_DIV, SPI_CLK_CNT_DIV);
	spi_tx_byte_order(HSPI, SPI_BYTE_ORDER_HIGH_TO_LOW);

	SET_PERI_REG_MASK(SPI_USER(HSPI), SPI_CS_SETUP|SPI_CS_HOLD);

	CLEAR_PERI_REG_MASK(SPI_USER(HSPI), SPI_FLASH_MODE);


}

uint32 SpiSend_old(uint8 spi_no, uint32 bits, uint32 data0, uint32 data1, uint32 data2, uint32 data3, uint32 data4){

	//code for custom Chip Select as GPIO PIN here

	while(spi_busy(spi_no)); //wait for SPI to be ready

//########## Enable SPI Functions ##########//
	//disable MOSI, MISO, ADDR, COMMAND, DUMMY in case previously set.
	CLEAR_PERI_REG_MASK(SPI_USER(spi_no), SPI_USR_MOSI|SPI_USR_MISO|SPI_USR_COMMAND|SPI_USR_ADDR|SPI_USR_DUMMY);


	WRITE_PERI_REG(SPI_USER1(HSPI), ((bits-1)&SPI_USR_MOSI_BITLEN)<<SPI_USR_MOSI_BITLEN_S); //((0-1)&SPI_USR_ADDR_BITLEN)<<SPI_USR_ADDR_BITLEN_S | //Number of bits in Address
									  //((bits-1)&SPI_USR_MOSI_BITLEN)<<SPI_USR_MOSI_BITLEN_S | //Number of bits to Send
									  //((0-1)&SPI_USR_MISO_BITLEN)<<SPI_USR_MISO_BITLEN_S |  //Number of bits to receive
									  //((0-1)&SPI_USR_DUMMY_CYCLELEN)<<SPI_USR_DUMMY_CYCLELEN_S); //Number of Dummy bits to insert

		SET_PERI_REG_MASK(SPI_USER(spi_no), SPI_USR_MOSI); //enable MOSI function in SPI module

		WRITE_PERI_REG(SPI_W0(spi_no), data0);//<<(32-bits));
		WRITE_PERI_REG(SPI_W1(spi_no), data1);//<<(32-bits));
		WRITE_PERI_REG(SPI_W2(spi_no), data2);//<<(32-bits));
		WRITE_PERI_REG(SPI_W3(spi_no), data3);//<<(32-bits));
		WRITE_PERI_REG(SPI_W4(spi_no), data4);//<<(32-bits));

//########## Begin SPI Transaction ##########//
	SET_PERI_REG_MASK(SPI_CMD(spi_no), SPI_USR);
//########## END SECTION ##########//



	//Transaction completed
	return 1; //success
}

uint32 SpiSend(uint8 spi_no, uint8 bytes, uint32 *data){

	uint8 i;

	bytes++;

	//code for custom Chip Select as GPIO PIN here

	while(spi_busy(spi_no)); //wait for SPI to be ready

//########## Enable SPI Functions ##########//
	//disable MOSI, MISO, ADDR, COMMAND, DUMMY in case previously set.
	CLEAR_PERI_REG_MASK(SPI_USER(spi_no), SPI_USR_MOSI|SPI_USR_MISO|SPI_USR_COMMAND|SPI_USR_ADDR|SPI_USR_DUMMY);


	WRITE_PERI_REG(SPI_USER1(HSPI), (((bytes * 32) - 1)&SPI_USR_MOSI_BITLEN)<<SPI_USR_MOSI_BITLEN_S); //((0-1)&SPI_USR_ADDR_BITLEN)<<SPI_USR_ADDR_BITLEN_S | //Number of bits in Address
									  //((bits-1)&SPI_USR_MOSI_BITLEN)<<SPI_USR_MOSI_BITLEN_S | //Number of bits to Send
									  //((0-1)&SPI_USR_MISO_BITLEN)<<SPI_USR_MISO_BITLEN_S |  //Number of bits to receive
									  //((0-1)&SPI_USR_DUMMY_CYCLELEN)<<SPI_USR_DUMMY_CYCLELEN_S); //Number of Dummy bits to insert

	SET_PERI_REG_MASK(SPI_USER(spi_no), SPI_USR_MOSI); //enable MOSI function in SPI module

	for (i = 0; i < bytes; i++)
	{
		WRITE_PERI_REG((REG_SPI_BASE(spi_no) + (0x40 + (i * 4))), data[i]);
	}


//########## Begin SPI Transaction ##########//
	SET_PERI_REG_MASK(SPI_CMD(spi_no), SPI_USR);
//########## END SECTION ##########//

	/*
	uint8 reply[1] = {0x80};

	if(GPIO_INPUT_GET(5) == 1)
	{
		ets_uart_printf("Sending TCP\r\n");
		espconn_send(&pTcpServer, reply, 1);
		ets_uart_printf("Sending TCP done\r\n");
	}
*/
	//Transaction completed
	return 1; //success
}

void ICACHE_FLASH_ATTR UdpReceive_EventHandler(void *arg, char *pdata, unsigned short len)
{
	uint32 data0;
	uint32 data1;
	uint32 data2;
	uint32 data3;
	uint32 data4;

	uint16 i;
	for (i = 0; i < len;)
	{
		data0 = ((uint32)pdata[i++]<<24) | ((uint32)pdata[i++]<<16) | ((uint32)pdata[i++]<<8) | (uint32)pdata[i++];
		data1 = ((uint32)pdata[i++]<<24) | ((uint32)pdata[i++]<<16) | ((uint32)pdata[i++]<<8) | (uint32)pdata[i++];
		data2 = ((uint32)pdata[i++]<<24) | ((uint32)pdata[i++]<<16) | ((uint32)pdata[i++]<<8) | (uint32)pdata[i++];
		data3 = ((uint32)pdata[i++]<<24) | ((uint32)pdata[i++]<<16) | ((uint32)pdata[i++]<<8) | (uint32)pdata[i++];
		data4 = ((uint32)pdata[i++]<<24) | ((uint32)pdata[i++]<<16) | ((uint32)pdata[i++]<<8) | (uint32)pdata[i++];

		//spi_transaction(HSPI, 0, 0, 0, 0, 32, data, 0, 0);

	}

}

void ICACHE_FLASH_ATTR TcpReceive_EventHandler(void *arg, char *pdata, unsigned short len)
{

	uint16 i = 0;
	uint32 masks[4] = {0xFFFFFFFF, 0x000000FF, 0x0000FFFF, 0x00FFFFFF};
	uint32 data[16];
	uint8 *pD = (uint8 *)data;
	uint8 dCnt = 0;
	uint8 words = 0;

/*
	uint8 reply[1] = {0x80};

	if(GPIO_INPUT_GET(5) == 1 && ready == 1)
	{
		ets_uart_printf("Sending TCP\r\n");
		espconn_send(&pTcpServer, reply, 1);
		ets_uart_printf("Sending TCP done\r\n");
		ready = 0;
	}

	if(GPIO_INPUT_GET(5) == 0)
	{
		ready = 1;
	}
*/
	while (i < len)
	{
		*(pD + dCnt) = *(pdata + i);

		i++;
		dCnt++;

		if (dCnt == 64)
		{
			dCnt = 0;
			SpiSend(HSPI, 15, data);
		}


	}

	if (dCnt)
	{
		data[dCnt >> 2] = data[dCnt >> 2] & masks[dCnt & 0x03];
		SpiSend(HSPI, (dCnt - 1) >> 2, data);
	}



}



void ICACHE_FLASH_ATTR InitUdpServer()
{
    pUdpServer.type = ESPCONN_UDP;
    pUdpServer.proto.udp = (esp_udp *)os_zalloc((uint32)sizeof(esp_udp));
    pUdpServer.proto.udp->local_port = PORT;      // server port

    espconn_regist_recvcb(&pUdpServer, UdpReceive_EventHandler);
    espconn_create(&pUdpServer);

}


void ICACHE_FLASH_ATTR InitTcpServer()
{

    pTcpServer.type = ESPCONN_TCP;
    pTcpServer.state = ESPCONN_NONE;
    pTcpServer.proto.tcp = (esp_tcp *)os_zalloc((uint32)sizeof(esp_tcp));
    pTcpServer.proto.tcp->local_port = 6001;      // server port

    espconn_accept(&pTcpServer);
    espconn_regist_time(&pTcpServer, 180, 0);
	espconn_regist_recvcb(&pTcpServer, TcpReceive_EventHandler);
}

void ICACHE_FLASH_ATTR Wifi_EventHandler(System_Event_t * event)
{
	struct ip_info ipInfo;

	switch(event->event) {
		case EVENT_STAMODE_CONNECTED:
			ets_uart_printf("CONNECTING...\r\n"); // station Connecting
			break;
		case EVENT_STAMODE_GOT_IP:
			ets_uart_printf("CONNECTED. IP: "); // station Connected

			ets_uart_printf(IPSTR, IP2STR(&event->event_info.got_ip.ip));
			ets_uart_printf("\r\n");

	        //Set up UDP Server
	        //InitUdpServer();
	        InitTcpServer();

	    	break;

	    default:
	    	//TODO: Set led
	        break;
	    }

}


LOCAL void  gpio_handler(int *dummy)
{
    // clear gpio status. Say ESP8266EX SDK Programming Guide in  5.1.6. GPIO interrupt handler
    uint32 gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
    uint8 reply[1] = {0x80};

    //ets_uart_printf("Int\r\n");
    // if the interrupt was by GPIO0
    if (gpio_status & BIT(5))
    {
        // disable interrupt for GPIO0
        gpio_pin_intr_state_set(GPIO_ID_PIN(5), GPIO_PIN_INTR_DISABLE);


        espconn_send(&pTcpServer, reply, 1);


        // Reactivate interrupts for GPIO0
        gpio_pin_intr_state_set(GPIO_ID_PIN(5), GPIO_PIN_INTR_POSEDGE);
    }

    //clear interrupt status for GPIO0
    GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status);
}


void init_io()
{
    // Initialize the GPIO subsystem.
    gpio_init();


    // =================================================
    // Initialize GPIO2 and GPIO0 as GPIO
    // =================================================
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5);
    //PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);


    PIN_PULLUP_DIS(PERIPHS_IO_MUX_GPIO5_U);
    //PIN_PULLUP_EN(PERIPHS_IO_MUX_GPIO2_U);

    gpio_output_set(0, 0, 0, GPIO_ID_PIN(5)); // set set gpio 0 as input

    // =================================================
    // Activate gpio interrupt for gpio2
    // =================================================

    // Disable interrupts by GPIO
    ETS_GPIO_INTR_DISABLE();

    // Attach interrupt handle to gpio interrupts.
    ETS_GPIO_INTR_ATTACH(gpio_handler, NULL);

    // clear gpio status. Say ESP8266EX SDK Programming Guide in  5.1.6. GPIO interrupt handler
    GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, BIT(5));

    // clear gpio status. Say ESP8266EX SDK Programming Guide in  5.1.6. GPIO interrupt handler
    gpio_pin_intr_state_set(GPIO_ID_PIN(5), GPIO_PIN_INTR_ANYEDGE);


    // Enable interrupts by GPIO
    ETS_GPIO_INTR_ENABLE();
}



void user_rf_pre_init(void)
{


}

void user_init(void)
{
	GPIO_INT_TYPE gpio_type;
	uint8_t gpio_pin;



	system_update_cpu_freq(SYS_CPU_160MHz);
	uart_init(BIT_RATE_115200, BIT_RATE_115200);

	//gpio_output_set(0, 0, 0, BIT5);
	init_io();

	SpiInit();

    //Init Wifi event callback
    wifi_set_event_handler_cb(Wifi_EventHandler);   // monitor wifi state

    InitStation();
    //InitServer();
    InitTcpServer();

}

uint32 ICACHE_FLASH_ATTR

user_rf_cal_sector_set(void)
{
    enum flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 8;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}
