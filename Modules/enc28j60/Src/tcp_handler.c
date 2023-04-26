/*
 * We define the application state (struct hello_world_state) in the
 * hello-world.h file, so we need to include it here. We also include
 * uip.h (since this cannot be included in hello-world.h) and
 * <string.h>, since we use the memcpy() function in the code.
 */
#include "uip.h"
#include <string.h>
#include <tcp_handler.h>

extern MessageBufferHandle_t tcp_rx_message_buffer;
extern MessageBufferHandle_t tcp_tx_message_buffer;

/*
 * Declaration of the protosocket function that handles the connection
 * (defined at the end of the code).
 */
static int handle_connection(struct tcp_handler_state *s);
/*---------------------------------------------------------------------------*/
/*
 * The initialization function. We must explicitly call this function
 * from the system initialization code, some time after uip_init() is
 * called.
 */
void
tcp_handler_init(void)
{
  /* We start to listen for connections on TCP port 1000. */
  uip_listen(HTONS(LOCAL_PORT_TCP));
}
/*---------------------------------------------------------------------------*/
/*
 * In hello-world.h we have defined the UIP_APPCALL macro to
 * hello_world_appcall so that this funcion is uIP's application
 * function. This function is called whenever an uIP event occurs
 * (e.g. when a new connection is established, new data arrives, sent
 * data is acknowledged, data needs to be retransmitted, etc.).
 */
void
tcp_handler_appcall(void)
{
  /*
   * The uip_conn structure has a field called "appstate" that holds
   * the application state of the connection. We make a pointer to
   * this to access it easier.
   */
  struct tcp_handler_state *s = &(uip_conn->appstate);

  /*
   * If a new connection was just established, we should initialize
   * the protosocket in our applications' state structure.
   */
  if(uip_connected()) {
    PSOCK_INIT(&s->p, s->inputbuffer, sizeof(s->inputbuffer));
  }

  /*
   * Finally, we run the protosocket function that actually handles
   * the communication. We pass it a pointer to the application state
   * of the current connection.
   */
  handle_connection(s);
}
/*---------------------------------------------------------------------------*/
/*
 * This is the protosocket function that handles the communication. A
 * protosocket function must always return an int, but must never
 * explicitly return - all return statements are hidden in the PSOCK
 * macros.
 */
static int
handle_connection(struct tcp_handler_state *s)
{
  PSOCK_BEGIN(&s->p);

  uint8_t bufferRX[TCP_MESSAGE_BUFF_SIZE];
  uint8_t bufferTX[TCP_MESSAGE_BUFF_SIZE];
  memset(bufferRX, 0, TCP_MESSAGE_BUFF_SIZE);
  memset(bufferTX, 0, TCP_MESSAGE_BUFF_SIZE);

  uint8_t length = 0;



  PSOCK_READBUF_LEN(&s->p, 2);
  uint8_t data_length = PSOCK_DATALEN(&s->p);
//#if defined SEGGER_DEBUG || defined OCD_DEBUG || _UART_DEBUG
//    LOG("TCP recv %d data_length\r\n", data_length);
//#endif
  //memcpy(bufferRX, s->inputbuffer, 2);

#if defined SEGGER_DEBUG || defined OCD_DEBUG || _UART_DEBUG
  //LOG("TCP recv %d data_length - ", data_length);
//  for (int i = 0; i < data_length; ++i) {
//	LOG("%x ", (int)bufferRX[i]);
//  }
//  LOG("\r\n");
#endif

  memcpy(bufferRX, s->inputbuffer, TCP_MESSAGE_BUFF_SIZE);

  uint8_t sof_index = 0;
  uint8_t packet_size = 0;

  for (uint8_t i = 0; i < TCP_MESSAGE_BUFF_SIZE; ++i) {
	sof_index = i;
	if (bufferRX[i] == (char) 0xFF) {
		;
	} else {
		continue;
	}

	if ((sof_index + 1) <= TCP_MESSAGE_BUFF_SIZE) {
		packet_size = bufferRX[sof_index + 1];
	} else {
		break;
	}

	if ((sof_index + packet_size) <= TCP_MESSAGE_BUFF_SIZE) {
		length = bufferRX[sof_index + 1];
#if defined SEGGER_DEBUG || defined OCD_DEBUG || _UART_DEBUG
		//SEGGER_SYSVIEW_PrintfHost("TCP recv %d\n", length);
//		LOG("TCP recv %d sof_index %d packet_size - ", sof_index, packet_size);
//		for (int i = sof_index; i < length; ++i) {
//			LOG("%x ", (int)bufferRX[i]);
//		}
//		LOG("\r\n");
#endif
		i = sof_index + packet_size - 1;
	}
	break;
  }

  if (length > 0) {
	xMessageBufferSend(tcp_rx_message_buffer, bufferRX+sof_index, length, 10);
	//enter to decoder task
  }

  length  = xMessageBufferReceive(tcp_tx_message_buffer, (uint8_t*)bufferTX, sizeof(bufferTX), 10);


	if (( length > 0 ) && (bufferTX[1] < (TCP_MESSAGE_BUFF_SIZE)))
	{
		//memcpy((char*)(tcp_pkt->data), bufferTX, bufferTX[1]);
		if ((uint16_t)bufferTX[1] % 2 == 1) {
			packet_size = (uint16_t)bufferTX[1] + 1;
		} else {
			packet_size = (uint16_t)bufferTX[1];
		}

#if defined SEGGER_DEBUG || defined OCD_DEBUG || _UART_DEBUG
		//SEGGER_SYSVIEW_PrintfHost("TCP send %d\n", length);
//		LOG("TCP send %d packet_size - ", packet_size);
//		for (int i = 0; i < packet_size; ++i) {
//			LOG("%x ", (int)bufferTX[i]);
//		}
//		LOG("\r\n");
#endif
		PSOCK_SEND(&s->p, bufferTX, packet_size);
	}




  //PSOCK_SEND_STR(&s->p, "Hello. What is your name?\n");

  //strncpy(s->name, s->inputbuffer, sizeof(s->name));

//#ifdef SEGGER_DEBUG
//  SEGGER_RTT_printf(0, "RCV %s\n", s->inputbuffer);
//#endif


  //PSOCK_SEND_STR(&s->p, "Hello");
  //PSOCK_SEND_STR(&s->p, s->name);

  
  //PSOCK_CLOSE(&s->p);
  PSOCK_END(&s->p);
}
/*---------------------------------------------------------------------------*/
