#include "tcp.h"

uint32_t i;
uint8_t delay_arp = 0;
//volatile uint32_t st;

#define BUF ((struct uip_eth_hdr *)&uip_buf[0])

void tcp_init() {
  	struct uip_eth_addr mac = MAC_ADDR;
  	volatile uint8_t ipaddr_[4]=IP_ADDR;
  	volatile uint8_t ipgate[4]=IP_GATE;
  	volatile uint8_t ipmask[4]=IP_MASK;

	enc28j60_init(mac.addr);
	uip_init();
	uip_arp_init();
	tcp_handler_init();

	uip_setethaddr(mac);

	uip_ipaddr_t ipaddr;
	uip_ipaddr(ipaddr, ipaddr_[0], ipaddr_[1], ipaddr_[2], ipaddr_[3]);
	uip_sethostaddr(ipaddr);

	uip_ipaddr(ipaddr, ipgate[0], ipgate[1], ipgate[2], ipgate[3]);
	uip_setdraddr(ipaddr);

	uip_ipaddr(ipaddr, ipmask[0], ipmask[1], ipmask[2], ipmask[3]);
	uip_setnetmask(ipaddr);
}

void tcp_periodic_task() {
	delay_arp++;
	for (i = 0; i < UIP_CONNS; i++) {
		uip_periodic(i);
		if (uip_len > 0) {
			uip_arp_out();
			enc28j60_send_packet((uint8_t *) uip_buf, uip_len);
		}
	}

	#if UIP_UDP
		for(i = 0; i < UIP_UDP_CONNS; i++) {
			uip_udp_periodic(i);
			if(uip_len > 0) {
				uip_arp_out();
				network_send();
			}
		}
	#endif /* UIP_UDP */

	if (delay_arp >= 50) {
		delay_arp = 0;
		uip_arp_timer();
	}
}

void tcp_task() {
	uip_len = enc28j60_recv_packet((uint8_t *) uip_buf, UIP_BUFSIZE);

	if (uip_len > 0) {
		if (BUF->type == htons(UIP_ETHTYPE_IP)) {
			uip_arp_ipin();
			uip_input();
			if (uip_len > 0) {
				uip_arp_out();
				enc28j60_send_packet((uint8_t *) uip_buf, uip_len);
			}
		} else if (BUF->type == htons(UIP_ETHTYPE_ARP)) {
			uip_arp_arpin();
			if (uip_len > 0) {
				enc28j60_send_packet((uint8_t *) uip_buf, uip_len);
			}
		}
	}
}
