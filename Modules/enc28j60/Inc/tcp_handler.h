#ifndef __HELLO_WORLD_H__
#define __HELLO_WORLD_H__

/* Since this file will be included by uip.h, we cannot include uip.h
   here. But we might need to include uipopt.h if we need the u8_t and
   u16_t datatypes. */
#include "uipopt.h"

#include "psock.h"

#include "config.h"
#include "tcp_stream_buffer.h"


/* Next, we define the uip_tcp_appstate_t datatype. This is the state
   of our application, and the memory required for this state is
   allocated together with each TCP connection. One application state
   for each TCP connection. */
typedef struct tcp_handler_state {
  struct psock p;
  char inputbuffer[128];
  char name[128];
} uip_tcp_appstate_t;

/* Finally we define the application function to be called by uIP. */
void tcp_handler_appcall(void);
#ifndef UIP_APPCALL
#define UIP_APPCALL tcp_handler_appcall
#endif /* UIP_APPCALL */

void tcp_handler_init(void);

#endif /* __HELLO_WORLD_H__ */
/** @} */
/** @} */
