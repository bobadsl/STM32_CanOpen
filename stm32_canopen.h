/*
  Copyright (c) 2015 Collin Kidder.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _CANOPEN_LIBRARY_
#define _CANOPEN_LIBRARY_

#include <Arduino.h>
#include <can_driver.h>  //relies on stm32_can for hardware access
#include <stm32f1xx_hal.h>

const int MAX_DEVICES = 6;

enum CANOPEN_OPSTATE
{
	BOOTUP,
	PREOP,	
	OPERATIONAL,
	STOPPED,
	RST
};

enum SDO_COMMAND
{
	SDO_WRITE = 0x20,
	SDO_READ = 0x40,
	SDO_WRITEACK = 0x60,
};

typedef union {
	bool boolean[4];
	float Float;
	uint32_t uint32;
	uint16_t uint16[2];
	uint8_t  uint8[4];
	int32_t int32;
	int16_t int16[2];
	int8_t  int8[4];
} Bytes4Union;

struct SDO_FRAME
{
	uint8_t nodeID;
	SDO_COMMAND cmd = SDO_WRITE;
	uint16_t index;
	uint8_t subIndex = 0;
	uint8_t dataLength = 4;
	Bytes4Union data = {0};
};

//CANOpen has slaves and a master. We've got to pick, are we the slave or the master.

class CANOPEN
{
public:
	CANOPEN(int busIndex);
	void setMasterMode();
	void setSlaveMode();
	void setHeartbeatInterval(uint32_t interval);
	bool isInitialized();
	void begin(canBitrate bitrate, int nodeId);
	void sendNodeStart(int nodeId);
	void sendNodePreop(int nodeId);
	void sendNodeReset(int nodeId);
	void sendNodeStop(int nodeId);
	bool sendPDOMessage(int id, int length, unsigned char *data);
	void sendSDORequest(SDO_FRAME *frame);
	void sendSDOResponse(SDO_FRAME *frame);
	void sendHeartbeat();
	void receiveFrame(CAN_FRAME *frame);
	void loop();
	void setStateChangeCallback(void (*cb)(CANOPEN_OPSTATE));
	void setPDOCallback(void (*cb)(CAN_FRAME *));
	void setHeartbeatCallback(void (*cb)(CAN_FRAME *));
	void setSDOReqCallback(void (*cb)(SDO_FRAME *));
	void setSDOReplyCallback(void (*cb)(SDO_FRAME *));
	void addNodeToListenTo(uint8_t nodeId);
	void setOpState(CANOPEN_OPSTATE opState);
	can_driver bus;
	void setDebugging(bool debugging) { debug = debugging; }
	void canFrameToCanMessage(CAN_FRAME* frame, CanMessage* message);
	void canMessagToCanFrame(CanMessage* message, CAN_FRAME* frame);
	uint8_t getNodeId() { return nodeID; }
	CANOPEN_OPSTATE getOpState() { return opState; }
protected:
private:
	bool debug = false;
	bool isMaster = false; // Default to slave
	bool bInitialized; //has a device already got this channel set up?
	CANOPEN_OPSTATE opState;
	int nodeID; //our ID
	int busNum;
	void (*cbStateChange[MAX_DEVICES])(CANOPEN_OPSTATE newState); //callback used when the network state changes
	void (*cbGotPDOMsg[MAX_DEVICES])(CAN_FRAME *); //callback used when we get a PDO request addressed to us
	void (*cbGotHeartbeatMsg[MAX_DEVICES])(CAN_FRAME *); //callback used when we get a Heartbeat frame
	void (*cbGotSDOReq[MAX_DEVICES])(SDO_FRAME *); //callback used when we get a SDO request addressed to us.
	void (*cbGotSDOReply[MAX_DEVICES])(SDO_FRAME *); //callback used when we get a SDO request addressed to us.
	uint8_t nodesToListenTo[12];
	void sendStateChange(CANOPEN_OPSTATE newState);
	void sendGotPDOMsg(CAN_FRAME *frame);
	void sendGotSDOReq(SDO_FRAME *frame);
	void sendGotSDOReply(SDO_FRAME *frame);
	void sendGotHeartbeat(CAN_FRAME *frame);
	uint32_t heartbeatInterval; //in milliseconds
	uint32_t lastHeartbeat;
	void sendNMTMsg(int id, int cmd);
};

extern CANOPEN CanOpen;
#endif // _CANOPEN_LIBRARY_
