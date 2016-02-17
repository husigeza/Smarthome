#include "SH_uIP.h"
#include "uip.h"
#include "rit128x96x4.h"

void smart_house_connection_init(void){
	uip_listen(HTONS(1000));
}

extern void DisplayIPAddress(void *ipaddr, unsigned long ulCol, unsigned long ulRow);
//extern void GetDisplayableMAC(unsigned char *stringMAC, unsigned char *realMAC);
extern unsigned char displayableMAC[18];

int handle_connection(struct smart_home_uip_state *s) {

	uip_ipaddr_t uip_hostaddr_1;

unsigned char *line0[21];
unsigned char *line1[21];
unsigned char *line2[21];
unsigned char *line3[21];

int line_num;
int remain_chars;
int i = 0;

	for(i=0;i<20;i++)
	{
		line0[i] = 0;
		line1[i] = 0;
		line2[i] = 0;
		line3[i] = 0;
	}
	i = 0;

	PSOCK_BEGIN(&s->p);
	PSOCK_READTO(&s->p, '\n');


	while (s->inputbuffer[i] != '\0')
	{
		i++;
	}
	line_num = i/20;
	remain_chars = i%20;

	if(line_num == 0)
	{
		memcpy(line0,s->inputbuffer,20);
		line0[20] = 0;

	}
	else if(line_num == 1)
	{
		memcpy(line0,s->inputbuffer,20);
		line0[20] = 0;
		memcpy(line1,s->inputbuffer+20,20);
		line1[20] = 0;

	}
	else if(line_num == 2)
	{
		memcpy(line0,s->inputbuffer,20);
		line0[20] = 0;
		memcpy(line1,s->inputbuffer+20,20);
		line1[20] = 0;
		memcpy(line2,s->inputbuffer+40,20);
		line2[20] = 0;
	}
	else if(line_num == 3)
	{
		memcpy(line0,s->inputbuffer,20);
		line0[20] = 0;
		memcpy(line1,s->inputbuffer+20,20);
		line1[20] = 0;
		memcpy(line2,s->inputbuffer+40,20);
		line2[20] = 0;
		memcpy(line3,s->inputbuffer+60,20);
		line3[20] = 0;

	}



	for(int k = 0; k < remain_chars ;k++)
	{
		if(line_num == 0) 			line0[20-k] = 0;
		else if (line_num == 1)		line1[20-k] = 0;
		else if (line_num == 2)		line2[20-k] = 0;
		else if (line_num == 3)		line3[20-k] = 0;

	}

	uip_gethostaddr(&uip_hostaddr_1);

	RIT128x96x4Clear();
	RIT128x96x4StringDraw("Link Established", 2, 8, 15);
	RIT128x96x4StringDraw(displayableMAC, 2, 18, 15);
	DisplayIPAddress(&uip_hostaddr_1, 2, 26);
	RIT128x96x4StringDraw(line0, 0, 58, 15);
	RIT128x96x4StringDraw(line1, 0, 68, 15);
	RIT128x96x4StringDraw(line2, 0, 78, 15);
	RIT128x96x4StringDraw(line3, 0, 88, 15);

	PSOCK_SEND_STR(&s->p, "Command ACK!\n ");
	PSOCK_CLOSE(&s->p);
	PSOCK_END(&s->p);

/*
	PSOCK_BEGIN(&s->p);
	PSOCK_READTO(&s->p, '\n');
	RIT128x96x4Clear();
	RIT128x96x4StringDraw(s->inputbuffer,2,48,15);
	PSOCK_SEND_STR(&s->p, "Command ACK!\n ");
	PSOCK_CLOSE(&s->p);
	PSOCK_END(&s->p);
*/
}

void
smart_house_uip_appcall(void) {

  struct smart_home_uip_state *s = &(uip_conn->appstate);

  if(uip_connected()) {
	PSOCK_INIT(&s->p, s->inputbuffer, sizeof(s->inputbuffer));
  }

  handle_connection(s);

}

