
#ifndef __SMART_HOME_UIP_STATE_H__
#define __SMART_HOME_UIP_STATE_H__

#include "uipopt.h"
#include "psock.h"

void smart_house_connection_init(void);
void smart_house_uip_appcall(void);
#ifndef UIP_APPCALL
#define UIP_APPCALL smart_house_uip_appcall
#endif

typedef struct smart_home_uip_state {
	struct psock p;
	unsigned char inputbuffer[1000];
	char name[40];
}uip_tcp_appstate_t;

/*struct httpd_state
{
    u8_t state;
    u16_t count;
    u16_t bufcount;
};*/


#endif

