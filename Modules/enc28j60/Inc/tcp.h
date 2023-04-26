#ifndef ENC28J60_INC_TCP_H_
#define ENC28J60_INC_TCP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"
#include "uip.h"
#include "uip_arp.h"
#include "enc28j60.h"

void tcp_init();
void tcp_periodic_task();
void tcp_task();

#ifdef __cplusplus
}
#endif

#endif /* ENC28J60_INC_TCP_H_ */
