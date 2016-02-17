//*****************************************************************************
//
// enet_uip.c - Sample WebServer Application for Ethernet Demo
//
// Copyright (c) 2007-2013 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 10007 of the EK-LM3S8962 Firmware Package.
//
//*****************************************************************************

#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_ethernet.h"
#include "debug.h"
#include "ethernet.h"
#include "flash.h"
#include "gpio.h"
#include "interrupt.h"
#include "sysctl.h"
#include "systick.h"
#include "rit128x96x4.h"
#include "uip.h"
#include "uip_arp.h"

#include "dhcpc.h"
#include "ustdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include  "semphr.h"
//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Ethernet with uIP (enet_uip)</h1>
//!
//! This example application demonstrates the operation of the Stellaris
//! Ethernet controller using the uIP TCP/IP Stack.  DHCP is used to obtain
//! an Ethernet address.  A basic web site is served over the Ethernet port.
//! The web site displays a few lines of text, and a counter that increments
//! each time the page is sent.
//!
//! For additional details on uIP, refer to the uIP web page at:
//! http://www.sics.se/~adam/
//
//*****************************************************************************


//#define USE_STATIC_IP

#define UIP_ARCH_ADD32

//*****************************************************************************
//
// Macro for accessing the Ethernet header information in the buffer.
//
//*****************************************************************************
#define BUF                     ((struct uip_eth_hdr *)&uip_buf[0])


//*****************************************************************************
//
// UIP Timers (in MS)
//
//*****************************************************************************
#define UIP_PERIODIC_TIMER_MS   500
#define UIP_ARP_TIMER_MS        10000


//*****************************************************************************
//
// Get MAC Address in displayable format.
//
//*****************************************************************************
void GetDisplayableMAC(unsigned char *stringMAC, unsigned char *realMAC)
{
	int k = 0,i= 0;
	int proba;

	realMAC[2]  = ':';
	realMAC[5]  = ':';
	realMAC[8]  = ':';
	realMAC[11] = ':';
	realMAC[14] = ':';
	realMAC[17] = 0;

	for(i = 0; i < 17; i++)
	{
		proba = i % 3;
		if(i%3 != 2 )
		{

			//Swap is needed because the ARM is little endian
			if(((stringMAC[k] & 0xF0) >> 4) <= 9)
			{
				realMAC[i] = ((stringMAC[k] & 0xF0) >> 4) + 48;
			}
			else
			{
				realMAC[i] = ((stringMAC[k] & 0xF0) >> 4) + 55;
			}

			if((stringMAC[k] & 0x0F) <= 9)
			{
				realMAC[i+1] = (stringMAC[k] & 0x0F) + 48;
			}
			else
			{
				realMAC[i+1] = (stringMAC[k] & 0x0F) + 55;
			}
			i++;
			k++;
		}
	}
}

//*****************************************************************************
//
// Display a uIP type IP Address.
//
//*****************************************************************************
void
DisplayIPAddress(void *ipaddr, unsigned long ulCol, unsigned long ulRow)
{
    char pucBuf[16];
    unsigned char *pucTemp = (unsigned char *)ipaddr;

    //
    // Convert the IP Address into a string.
    //
    usprintf(pucBuf, "%d.%d.%d.%d", pucTemp[0], pucTemp[1], pucTemp[2],
             pucTemp[3]);

    //
    // Display the string.
    //
    RIT128x96x4StringDraw(pucBuf, ulCol, ulRow, 15);
}

//*****************************************************************************
//
//! When using the timer module in UIP, this function is required to return
//! the number of ticks.  Note that the file "clock-arch.h" must be provided
//! by the application, and define CLOCK_CONF_SECONDS as the number of ticks
//! per second, and must also define the typedef "clock_time_t".
//
//*****************************************************************************
clock_time_t
clock_time(void)
{
    return((clock_time_t)SysTickValueGet());
}

//*****************************************************************************
//
// The interrupt handler for the Ethernet interrupt.
//
//*****************************************************************************
long lPacketLength;
xSemaphoreHandle RX_SEM = NULL;
void
EthernetIntHandler(void)
{
    unsigned long ulTemp;
    static portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    //
    // Read and Clear the interrupt.
    //
    ulTemp = EthernetIntStatus(ETH_BASE, false);
    EthernetIntClear(ETH_BASE, ulTemp);

    //
    // Check to see if an RX Interrupt has occured.
    //
    if(ulTemp & ETH_INT_RX)
    {
		// Disable Ethernet RX Interrupt.
		EthernetIntDisable(ETH_BASE, ETH_INT_RX);

		lPacketLength = EthernetPacketGetNonBlocking(ETH_BASE, uip_buf,sizeof(uip_buf));

// Ethernet Interupt Handler: free a semaphore for a task which process the packet
		xSemaphoreGiveFromISR(RX_SEM,&xHigherPriorityTaskWoken);
		if (xHigherPriorityTaskWoken == pdTRUE) {
			portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
		}
		EthernetIntEnable(ETH_BASE, ETH_INT_RX);
    }
}


//*****************************************************************************
//
// Callback for when DHCP client has been configured.
//
//*****************************************************************************
void
dhcpc_configured(const struct dhcpc_state *s)
{
    uip_sethostaddr(&s->ipaddr);
    uip_setnetmask(&s->netmask);
    uip_setdraddr(&s->default_router);
    DisplayIPAddress((void *)&s->ipaddr, 2, 26);
}


/*
 * Tasks
 */

void RXPacket_process_task(void* pvParameters);
void ARP_periodic_task(void* pvParameters);
void TCPIP_periodic_task(void* pvParameters);
uip_ipaddr_t ipaddr;
long lPacketLength;
unsigned long ulUser0, ulUser1;
unsigned long ulTemp;
static struct uip_eth_addr sTempAddr;
unsigned char displayableMAC[18];
/*
 * Init Task
 */

void Init_task(void* pvParameters)
{
	while(1)
	{
		    // Initialize the OLED display.
		    //
		    RIT128x96x4Init(1000000);

		    //
		    // Enable and Reset the Ethernet Controller.
		    //
		    SysCtlPeripheralEnable(SYSCTL_PERIPH_ETH);
		    SysCtlPeripheralReset(SYSCTL_PERIPH_ETH);

		    //
		    // Enable Port F for Ethernet LEDs.
		    //  LED0        Bit 3   Output
		    //  LED1        Bit 2   Output
		    //
		    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
		    GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);

		    //
		    // Configure SysTick for a periodic interrupt.
		    //
		    SysTickPeriodSet(SysCtlClockGet() / 1000);
		    SysTickEnable();
		    SysTickIntEnable();

		    //
		    // Intialize the Ethernet Controller and disable all Ethernet Controller
		    // interrupt sources.
		    //
		    EthernetIntDisable(ETH_BASE, (ETH_INT_PHY | ETH_INT_MDIO | ETH_INT_RXER |
		                       ETH_INT_RXOF | ETH_INT_TX | ETH_INT_TXER | ETH_INT_RX));
		    ulTemp = EthernetIntStatus(ETH_BASE, false);
		    EthernetIntClear(ETH_BASE, ulTemp);

		    //
		    // Initialize the Ethernet Controller for operation.
		    //
		    EthernetInitExpClk(ETH_BASE, SysCtlClockGet());

		    //
		    // Configure the Ethernet Controller for normal operation.
		    // - Full Duplex
		    // - TX CRC Auto Generation
		    // - TX Padding Enabled
		    //
		    EthernetConfigSet(ETH_BASE, (ETH_CFG_TX_DPLXEN | ETH_CFG_TX_CRCEN |
		                                 ETH_CFG_TX_PADEN));

		    //
		    // Wait for the link to become active.
		    //
		    RIT128x96x4StringDraw("Waiting for Link", 2, 8, 15);
		    ulTemp = EthernetPHYRead(ETH_BASE, PHY_MR1);
		    while((ulTemp & 0x0004) == 0)
		    {
		        ulTemp = EthernetPHYRead(ETH_BASE, PHY_MR1);
		    }

		    RIT128x96x4Clear();
		    RIT128x96x4StringDraw("Link Established", 2, 8, 15);

		    //
		    // Enable the Ethernet Controller.
		    //
		    EthernetEnable(ETH_BASE);

		    //
		    // Enable the Ethernet interrupt.
		    //
		    IntEnable(INT_ETH);

		    //
		    // Enable the Ethernet RX Packet interrupt source.
		    //
		    EthernetIntEnable(ETH_BASE, ETH_INT_RX);

		    //
		    // Enable all processor interrupts.
		    //
		    IntMasterEnable();

		    //
		    // Initialize the uIP TCP/IP stack.
		    //
		    uip_init();

			#ifdef USE_STATIC_IP
				uip_ipaddr(ipaddr, 192, 168, 2, 102);
				uip_sethostaddr(ipaddr);
				uip_ipaddr(ipaddr, 255, 255, 255, 255);
				DisplayIPAddress(ipaddr, 2, 26);
				uip_setnetmask(ipaddr);
			#else
				uip_ipaddr(ipaddr, 0, 0, 0, 0);
				uip_sethostaddr(ipaddr);
				//DisplayIPAddress(ipaddr, 18, 24);
				uip_ipaddr(ipaddr, 0, 0, 0, 0);
				uip_setnetmask(ipaddr);
			#endif


		    //
		    // Configure the hardware MAC address for Ethernet Controller filtering of
		    // incoming packets.
		    //
		    // For the Ethernet Eval Kits, the MAC address will be stored in the
		    // non-volatile USER0 and USER1 registers.  These registers can be read
		    // using the FlashUserGet function, as illustrated below.
		    //
		    FlashUserGet(&ulUser0, &ulUser1);
		    if((ulUser0 == 0xffffffff) || (ulUser1 == 0xffffffff))
		    {
		        //
		        // We should never get here.  This is an error if the MAC address has
		        // not been programmed into the device.  Exit the program.
		        //
		        RIT128x96x4StringDraw("MAC Address", 0, 16, 15);
		        RIT128x96x4StringDraw("Not Programmed!", 0, 24, 15);
		        while(1)
		        {
		        }
		    }

		    //
		    // Convert the 24/24 split MAC address from NV ram into a 32/16 split MAC
		    // address needed to program the hardware registers, then program the MAC
		    // address into the Ethernet Controller registers.
		    //
		    sTempAddr.addr[0] = ((ulUser0 >>  0) & 0xff);
		    sTempAddr.addr[1] = ((ulUser0 >>  8) & 0xff);
		    sTempAddr.addr[2] = ((ulUser0 >> 16) & 0xff);
		    sTempAddr.addr[3] = ((ulUser1 >>  0) & 0xff);
		    sTempAddr.addr[4] = ((ulUser1 >>  8) & 0xff);
		    sTempAddr.addr[5] = ((ulUser1 >> 16) & 0xff);

		    //
		    // Program the hardware with it's MAC address (for filtering).
		    //

		    EthernetMACAddrSet(ETH_BASE, (unsigned char *)&sTempAddr);
		    uip_setethaddr(sTempAddr);



		    GetDisplayableMAC((unsigned char*)&sTempAddr.addr,displayableMAC);
		    RIT128x96x4StringDraw(displayableMAC, 2, 18, 15);

		    //
		    // Initialize the TCP/IP Application
		    //
		    smart_house_connection_init();

			#ifndef USE_STATIC_IP
		    // Initialize the DHCP Client Application.
		    //
		    dhcpc_init(&sTempAddr.addr[0], 6);
		    dhcpc_request();
			#endif






		    vSemaphoreCreateBinary(RX_SEM);
		    xTaskCreate(RXPacket_process_task,"RX packet task",1000,NULL,2,NULL); //create RX packet handler tasks
		    xTaskCreate(ARP_periodic_task,"ARP task",1000,NULL,1,NULL); //create ARP task
		    xTaskCreate(TCPIP_periodic_task,"TCP/IP packet task",1000,NULL,1,NULL); //create TCP/IP, UDP periodic task
		    vTaskDelete(0);
	}

}
/*
 * RX packet process task
 */
void RXPacket_process_task(void* pvParameters)
{

	while(1)
	{
		//Try to take the semaphore
		xSemaphoreTake(RX_SEM,portMAX_DELAY);

		if(lPacketLength > 0)
		{
			// Set uip_len for uIP stack usage.
			uip_len = (unsigned short)lPacketLength;

			// Process incoming IP packets here.
			if(BUF->type == htons(UIP_ETHTYPE_IP))
			{
				uip_arp_ipin();
				uip_input();

				// If the above function invocation resulted in data that
				// should be sent out on the network, the global variable
				// uip_len is set to a value > 0.
				//
				if(uip_len > 0)
				{
					uip_arp_out();
					EthernetPacketPut(ETH_BASE, uip_buf, uip_len);
					uip_len = 0;
				}
			}
			// Process incoming ARP packets here.
			else if(BUF->type == htons(UIP_ETHTYPE_ARP))
			{
				uip_arp_arpin();

				// If the above function invocation resulted in data that
				// should be sent out on the network, the global variable
				// uip_len is set to a value > 0.
				if(uip_len > 0)
				{
					EthernetPacketPut(ETH_BASE, uip_buf, uip_len);
					uip_len = 0;
				}
			}
		}

		//Enable RX interrupt
		EthernetIntEnable(ETH_BASE, ETH_INT_RX);
	}
}

/*
 * ARP timer task: period = 10000 ms
 */
void ARP_periodic_task(void* pvParameters)
{
	portTickType LastWakeTime;

	LastWakeTime = xTaskGetTickCount();

	while(1)
	{

		uip_arp_timer();

		vTaskDelayUntil(&LastWakeTime, UIP_ARP_TIMER_MS/portTICK_RATE_MS);
	}
}

/*
 * TCP/IP periodic function (+UDP) period  =   500
 */
void TCPIP_periodic_task(void* pvParameters)
{
	portTickType LastWakeTime;
	unsigned long ulTemp;

	LastWakeTime = xTaskGetTickCount();

	while(1)
	{

		for(ulTemp = 0; ulTemp < UIP_CONNS; ulTemp++)
		{
			uip_periodic(ulTemp);

			// If the above function invocation resulted in data that
			// should be sent out on the network, the global variable
			// uip_len is set to a value > 0.
			//
			if(uip_len > 0)
			{
				uip_arp_out();
				EthernetPacketPut(ETH_BASE, uip_buf, uip_len);
				uip_len = 0;
			}
		}

		#if UIP_UDP
            for(ulTemp = 0; ulTemp < UIP_UDP_CONNS; ulTemp++)
            {
                uip_udp_periodic(ulTemp);

                // If the above function invocation resulted in data that
                // should be sent out on the network, the global variable
                // uip_len is set to a value > 0.
                //
                if(uip_len > 0)
                {
                    uip_arp_out();
                    EthernetPacketPut(ETH_BASE, uip_buf, uip_len);
                    uip_len = 0;
                }
            }
		#endif

		vTaskDelayUntil(&LastWakeTime, UIP_PERIODIC_TIMER_MS/portTICK_RATE_MS);
	}
}

/*
 * Main function
 */
int main(void)
{
    // Set the clocking to run directly from the crystal.
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |SYSCTL_XTAL_8MHZ);
	IntPriorityGroupingSet(7); //7 Groups, 0 sub group
	IntRegister(INT_ETH,&EthernetIntHandler);
	IntPrioritySet(INT_ETH,6);

	xTaskCreate(Init_task,"Init Task",1000,NULL,3,NULL);


	vTaskStartScheduler();

    while(1)
    {

    }
}
