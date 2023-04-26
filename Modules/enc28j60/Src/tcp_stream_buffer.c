#include "tcp_stream_buffer.h"

MessageBufferHandle_t tcp_rx_message_buffer = NULL;
MessageBufferHandle_t tcp_tx_message_buffer = NULL;

__inline void tcp_message_create_buffer( void )
{
    if (tcp_rx_message_buffer == NULL) {
    	tcp_rx_message_buffer = xMessageBufferCreate(TCP_MESSAGE_BUFF_SIZE);
    }
    if (tcp_tx_message_buffer == NULL) {
    	tcp_tx_message_buffer = xMessageBufferCreate(TCP_MESSAGE_BUFF_SIZE);
    }
}
