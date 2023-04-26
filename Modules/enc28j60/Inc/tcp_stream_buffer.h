#ifndef ENC28J60_INC_TCP_STREAM_BUFFER_H_
#define ENC28J60_INC_TCP_STREAM_BUFFER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <FreeRTOS.h>
#include <task.h>
#include <message_buffer.h>

#define TCP_MESSAGE_BUFF_SIZE 128

void tcp_message_create_buffer( void );

#ifdef __cplusplus
}
#endif

#endif /* ENC28J60_INC_TCP_STREAM_BUFFER_H_ */


