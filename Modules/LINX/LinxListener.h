/****************************************************************************************
**  Generic LINX listener header.
**
**  For more information see:           www.labviewmakerhub.com/linx
**  For support visit the forums at:    www.labviewmakerhub.com/forums/linx
**  
**  Written By Sam Kristoff
**
** BSD2 License.
****************************************************************************************/	

#ifndef LINX_LISTENER_H
#define LINX_LISTENER_H

/****************************************************************************************
** Defines
****************************************************************************************/

/****************************************************************************************
** Includes
****************************************************************************************/
#include "LinxDevice.h"
#include "LinxCustomCmdHandler.h"
/****************************************************************************************
** Enums
****************************************************************************************/
enum LinxListenerState
{
	START, 
	LISTENING, 
	AVAILABLE, 
	ACCEPT, 
	CONNECTED, 
	CLOSE, 
	EXIT
};

enum LinxListenerInterface
{
	UART, 
	TCP	
};


typedef enum ListenerStatus
{
	LUNKNOWN_STATE=128
}ListenerStatus;

class LinxListener
{
	public:	
		/****************************************************************************************
		**  Variables
		****************************************************************************************/
		LinxDevice* LinxDev;
		LinxListenerState State;
		LinxListenerInterface Interface;
		unsigned char ListenerChan = 0;
		
		unsigned char* recBuffer;
		unsigned char* sendBuffer;
		
		//int (*customCommands[16])(unsigned char, unsigned char*, unsigned char*, unsigned char*);
		typedef LinxStatus (LinxCustomCmdHandler::*customCommand)(unsigned char, unsigned char*, unsigned char*, unsigned char*);
		customCommand customCommands[10];
	    void AttachCustomCommand(unsigned short commandNumber, LinxStatus (LinxCustomCmdHandler::*function)(unsigned char, unsigned char*, unsigned char*, unsigned char*) );

		int (*periodicTasks[1])(unsigned char*, unsigned char*);
		
		
		/****************************************************************************************
		**  Constructors
		****************************************************************************************/
		LinxListener();
		
		/****************************************************************************************
		** Functions
		****************************************************************************************/
		virtual int Start();			//Start Listener
		virtual int Listen();			//Listen For Connection
		virtual int Available();		//New Client Connection Available
		virtual int Accept();		//Accept New Client Connection
		virtual int Connected();	//Connected To Client
		virtual int Close();			//Close Client Connection
		virtual int Exit();			//Stop Listening, Close And Exit
		
		//void AttachCustomCommand(unsigned short commandNumber, int (*function)(unsigned char, unsigned char*, unsigned char*, unsigned char*) );
		void AttachPeriodicTask(int (*function)(unsigned char*, unsigned char*));
		
		virtual int CheckForCommands();		//Execute Listener State Machine		
				
		int ProcessCommand(unsigned char* recBuffer, unsigned char* sendBuffer);
		void PacketizeAndSend(unsigned char* commandPacketBuffer, unsigned char* responsePacketBuffer, unsigned int dataSize, int status);
		void StatusResponse(unsigned char* commandPacketBuffer, unsigned char* responsePacketBuffer, int status);
		void DataBufferResponse(unsigned char* commandPacketBuffer, unsigned char* responsePacketBuffer, const unsigned char* dataBuffer, unsigned char dataSize, int status);
		unsigned char ComputeChecksum(unsigned char* packetBuffer);
		bool ChecksumPassed(unsigned char* packetBuffer);

	private:
		LinxCustomCmdHandler *customCmdHandler = new LinxCustomCmdHandler();
		uint16_t PACKET_NUM = 0xFFFF;
};

#endif //LINX_LISTENER_H
