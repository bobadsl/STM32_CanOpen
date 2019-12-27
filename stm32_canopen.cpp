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


#include "stm32_canopen.h"

void CanOpenMessagePending(CAN_HandleTypeDef* _hcan)
{
	Serial.println("INT");
	CanMessage msg;
	CAN_FRAME frame;
	CanOpen.bus.can_rx(&msg, 0);
	CanOpen.canMessagToCanFrame(&msg, &frame);
	CanOpen.receiveFrame(&frame);
}
CANOPEN::CANOPEN(int whichbus)
{
	isMaster = false;
	opState = BOOTUP;
	nodeID = 0x5F;
	busNum = whichbus;
	heartbeatInterval = 1000;

	for (int x = 0; x < MAX_DEVICES; x++)
	{
		cbStateChange[x] = NULL;
		cbGotPDOMsg[x] = NULL;
		cbGotSDOReq[x] = NULL;
		cbGotSDOReply[x] = NULL;
		nodesToListenTo[x] = 0;
		nodesToListenTo[x + 6] = 0;
	}

	bInitialized = false;
}

void CANOPEN::setMasterMode()
{
	isMaster = true;
}

void CANOPEN::setSlaveMode()
{
	isMaster = false;
}

void CANOPEN::begin(canBitrate speed = CAN_BITRATE_250K, int id = 0x5F)
{
	pinMode(LED_BUILTIN, OUTPUT);
	nodeID = id;
	bus.can_set_bitrate(speed);
	bus.can_init(id);
	if (isMaster)
	{
		opState = OPERATIONAL; //as the master we automatically become operational
		if (bus.can_add_filter_mask(0, 0) == CAN_FILTER_ERROR)
		{
			Serial.println("CAN FILTER SETUP FOR 0 FAILED");
			Error_Handler();
		}
		Serial.print("Init complete for master with id ");
		Serial.println(id);
	}
	else
	{
		opState = BOOTUP;
		if (bus.can_add_filter_mask(id, 0x7F) == CAN_FILTER_ERROR)
		{
			Serial.println("CAN FILTER SETUP FOR NODE ID FAILED");
			Error_Handler();
		}
		//Also allow through any message to ID 0 which is broadcast (allows all upper bit functions)
		if (bus.can_add_filter_mask(0, 0x7F) == CAN_FILTER_ERROR)
		{
			Serial.println("CAN FILTER SETUP FOR 0 ID FAILED");
			Error_Handler();
		}
		Serial.print("Init complete for slave with id ");
		Serial.println(id);
	}
	bus.can_enable();
	bus.can_wakeup(); // Make sure it is active
	bus.attachInterrupt(HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CanOpenMessagePending);
	if(!isMaster) sendHeartbeat();
	bInitialized = true;
}

bool CANOPEN::isInitialized()
{
	return bInitialized;
}

void CANOPEN::sendNodeStart(int id = 0)
{
	sendNMTMsg(id, 1);
}

void CANOPEN::sendNodePreop(int id = 0)
{
	sendNMTMsg(id, 0x80);
}

void CANOPEN::sendNodeReset(int id = 0)
{
	sendNMTMsg(id, 0x81);
}

void CANOPEN::sendNodeStop(int id = 0)
{
	sendNMTMsg(id, 2);
}

void CANOPEN::canFrameToCanMessage(CAN_FRAME* frame, CanMessage* message)
{
	message->data.uint64 = frame->data.uint64;
	message->length = frame->length;
	message->id = frame->id;
	message->rtr = frame->rtr;
}

void CANOPEN::canMessagToCanFrame(CanMessage* message, CAN_FRAME* frame)
{
	frame->data = message->data;
	frame->length = message->length;
	frame->id = message->id;
	frame->rtr = message->rtr;
}

void CANOPEN::sendNMTMsg(int id, int cmd)
{
	if (!isMaster) return; //only the master sends NMT commands
	id &= 0x7F;
	CAN_FRAME frame;
	frame.id = 0;
	frame.extended = false;
	frame.length = 2;
	frame.data.uint8[0] = cmd;
	frame.data.uint8[1] = id;
	//the rest don't matter
	CanMessage msg;
	canFrameToCanMessage(&frame, &msg);
	CanState result = bus.can_tx(&msg, 0);
	if (result != CanState::BUS_OK) {
		Serial.printlnf("Result is not expected. Expected %d. Got %d", BUS_OK, result);
	}
}

bool CANOPEN::sendPDOMessage(int id, int length, unsigned char* data)
{
	if (id > 0x57F) return false; //invalid ID for a PDO message
	if (id < 0x181) return false; //invalid ID for a PDO message
	if (length > 8 || length < 0) return false; //invalid length
	CAN_FRAME frame;
	frame.id = id;
	frame.extended = false;
	frame.length = length;
	for (int x = 0; x < length; x++) frame.data.uint8[x] = data[x];
	CanMessage msg;
	canFrameToCanMessage(&frame, &msg);
	CanState result = bus.can_tx(&msg, 0);
	if (result != CanState::BUS_OK) {
		Serial.printlnf("Result is not expected. Expected %d. Got %d", BUS_OK, result);
		return false;
	}
	return true;
	

}

void CANOPEN::sendSDOResponse(SDO_FRAME* sframe)
{
	sframe->nodeID &= 0x7f;
	CAN_FRAME frame;
	frame.length = 8;
	frame.extended = false;
	frame.id = 0x580 + sframe->nodeID;
	if (sframe->dataLength <= 4)
	{
		frame.data.uint8[0] = sframe->cmd;
		if (sframe->dataLength > 0) //responding with data
		{
			frame.data.uint8[0] |= 0x0F - ((sframe->dataLength - 1) * 4);
		}
		frame.data.uint8[1] = sframe->index & 0xFF;
		frame.data.uint8[2] = sframe->index >> 8;
		frame.data.uint8[3] = sframe->subIndex;
		for (int x = 0; x < sframe->dataLength; x++) frame.data.uint8[4 + x] = sframe->data.uint8[x];
		CanMessage msg;
		canFrameToCanMessage(&frame, &msg);
		CanState result = bus.can_tx(&msg, 0);
		if (result != CanState::BUS_OK) {
			Serial.printf("Result is not expected. Expected %d. Got %d\r\n", BUS_OK, result);
		}
	}
}

void CANOPEN::sendSDORequest(SDO_FRAME* sframe)
{
	sframe->nodeID &= 0x7F;
	CAN_FRAME frame;
	frame.extended = false;
	frame.length = 8;
	frame.id = 0x600 + sframe->nodeID;
	frame.data = { 0 };
	if (sframe->dataLength <= 4)
	{
		frame.data.uint8[0] = sframe->cmd;
		if (sframe->dataLength > 0) //request to write data
		{
			frame.data.uint8[0] |= 0x0F - ((sframe->dataLength - 1) * 4); //kind of dumb the way this works...
		}
		frame.data.uint8[1] = sframe->index & 0xFF;
		frame.data.uint8[2] = sframe->index >> 8;
		frame.data.uint8[3] = sframe->subIndex;
		// for (int x = 0; x < 4; x++) frame.data.uint8[4 + x] = 0; // Default
		for (int x = 0; x < sframe->dataLength; x++) {
			frame.data.uint8[4 + x] = sframe->data.uint8[x];
		}
		CanMessage msg;
		canFrameToCanMessage(&frame, &msg);
		CanState result = bus.can_tx(&msg, 0);
		if (result != CanState::BUS_OK) {
			Serial.printf("Result is not expected. Expected %d. Got %d\r\n", BUS_OK, result);
		}
	}
}

void CANOPEN::receiveFrame(CAN_FRAME* frame)
{
	SDO_FRAME sframe;
	uint8_t filtersReached = 0;
	if (frame->id == 0) //NMT message
	{
		filtersReached |= 0b0000001;
		if (frame->data.uint8[1] != nodeID && frame->data.uint8[1] != 0) return; //not for us.
		switch (frame->data.uint8[0])
		{
		case 1: //start this node
			opState = OPERATIONAL;
			break;
		case 2: //stop this node
			opState = STOPPED;
			break;
		case 0x80: //enter pre-op state
			opState = PREOP;
			break;
		case 0x81: //reset this node
			opState = RST;
			break;
		}
		sendStateChange(opState);
	}
	if (frame->id > 0x17F && frame->id < 0x580)
	{
		filtersReached |= 0b0000010;
		sendGotPDOMsg(frame);
	}
	if(frame->id > 0x700)
	{
		filtersReached |= 0b0000100;
		if(isMaster)
		{
			sendGotHeartbeat(frame);
		} else
		{
			for (uint8_t nodeToListenTo : nodesToListenTo)
			{
				if (nodeToListenTo == 0) break;
				if (frame->id - 0x700 == nodeToListenTo)
				{
					sendGotHeartbeat(frame);
				}
			}
		}
	}
	if (frame->id == 0x600 + nodeID) //SDO request targetted to our ID
	{
		filtersReached |= 0b0001000;
		sframe.nodeID = nodeID;
		sframe.index = frame->data.uint8[1] + (frame->data.uint8[2] * 256);
		sframe.subIndex = frame->data.uint8[3];
		sframe.cmd = (SDO_COMMAND)(frame->data.uint8[0] & 0xF0);

		if ((frame->data.uint8[0] != 0x40) && (frame->data.uint8[0] != 0x60))
		{
			sframe.dataLength = (3 - ((frame->data.uint8[0] & 0xC) >> 2)) + 1;
		}
		else sframe.dataLength = 0;

		for (int x = 0; x < sframe.dataLength; x++) sframe.data.uint8[x] = frame->data.uint8[4 + x];

		sendGotSDOReq(&sframe);

	}
	if (frame->id == 0x580 + nodeID && !isMaster) //SDO reply to our ID
	{
		filtersReached |= 0b0010000;
		sframe.nodeID = nodeID;
		sframe.index = frame->data.uint8[1] + (frame->data.uint8[2] * 256);
		sframe.subIndex = frame->data.uint8[3];
		sframe.cmd = (SDO_COMMAND)(frame->data.uint8[0] & 0xF0);

		if ((frame->data.uint8[0] != 0x40) && (frame->data.uint8[0] != 0x60))
		{
			sframe.dataLength = (3 - ((frame->data.uint8[0] & 0xC) >> 2)) + 1;
		}
		else sframe.dataLength = 0;

		for (int x = 0; x < sframe.dataLength; x++) sframe.data.uint8[x] = frame->data.uint8[4 + x];

		sendGotSDOReply(&sframe);
	} 
	else if((frame->id >= 0x580 && frame->id < 0x600) && !isMaster)
	{
		for (uint8_t nodeToListenTo : nodesToListenTo)
		{
			if (nodeToListenTo == 0) break;
			if (frame->id - 0x580 == nodeToListenTo)
			{
				filtersReached |= 0b0100000;
				sframe.nodeID = nodeID;
				sframe.index = frame->data.uint8[1] + (frame->data.uint8[2] * 256);
				sframe.subIndex = frame->data.uint8[3];
				sframe.cmd = (SDO_COMMAND)(frame->data.uint8[0] & 0xF0);

				if ((frame->data.uint8[0] != 0x40) && (frame->data.uint8[0] != 0x60))
				{
					sframe.dataLength = (3 - ((frame->data.uint8[0] & 0xC) >> 2)) + 1;
				}
				else sframe.dataLength = 0;

				for (int x = 0; x < sframe.dataLength; x++) sframe.data.uint8[x] = frame->data.uint8[4 + x];

				sendGotSDOReply(&sframe);
			}
		}
	}
	if ((frame->id >= 0x580 && frame->id < 0x600) && isMaster) //SDO reply to our ID
	{
		filtersReached |= 0b1000000;
		sframe.nodeID = nodeID;
		sframe.index = frame->data.uint8[1] + (frame->data.uint8[2] * 256);
		sframe.subIndex = frame->data.uint8[3];
		sframe.cmd = (SDO_COMMAND)(frame->data.uint8[0] & 0xF0);

		if ((frame->data.uint8[0] != 0x40) && (frame->data.uint8[0] != 0x60))
		{
			sframe.dataLength = (3 - ((frame->data.uint8[0] & 0xC) >> 2)) + 1;
		}
		else sframe.dataLength = 0;

		for (int x = 0; x < sframe.dataLength; x++) sframe.data.uint8[x] = frame->data.uint8[4 + x];

		sendGotSDOReply(&sframe);
	}

	if(filtersReached == 0 && debug)
	{
		Serial.print("The following message could not be parsed ");
		Serial.print("\tdata\t");
		for (auto& data : frame->data.uint8) {
			Serial.printf("%#02X ",data);
		}
		Serial.print("\tid\t"); Serial.printf("%#02X ", frame->id);
		Serial.print("\trtr\t"); Serial.print(frame->rtr);
		Serial.print("\tlength\t"); Serial.print(frame->length);
		Serial.println();
	}
	else {
		// Serial.printf("Filters reached %u\r\n", filtersReached);
	}
}

void CANOPEN::setStateChangeCallback(void (*cb)(CANOPEN_OPSTATE))
{
	for (int x = 0; x < MAX_DEVICES; x++)
	{
		if (cbStateChange[x] == NULL)
		{
			cbStateChange[x] = cb;
			return;
		}
	}
}

void CANOPEN::setPDOCallback(void (*cb)(CAN_FRAME*))
{
	for (int x = 0; x < MAX_DEVICES; x++)
	{
		if (cbGotPDOMsg[x] == NULL)
		{
			cbGotPDOMsg[x] = cb;
			return;
		}
	}
}

void CANOPEN::setHeartbeatCallback(void(* cb)(CAN_FRAME*))
{
	for (int x = 0; x < MAX_DEVICES; x++)
	{
		if (cbGotHeartbeatMsg[x] == NULL)
		{
			cbGotHeartbeatMsg[x] = cb;
			return;
		}
	}
}

void CANOPEN::setSDOReqCallback(void (*cb)(SDO_FRAME*))
{
	for (int x = 0; x < MAX_DEVICES; x++)
	{
		if (cbGotSDOReq[x] == NULL)
		{
			cbGotSDOReq[x] = cb;
			return;
		}
	}
}

void CANOPEN::setSDOReplyCallback(void (*cb)(SDO_FRAME*))
{
	for (int x = 0; x < MAX_DEVICES; x++)
	{
		if (cbGotSDOReply[x] == NULL)
		{
			cbGotSDOReply[x] = cb;
			return;
		}
	}
}

void CANOPEN::addNodeToListenTo(uint8_t nodeId)
{
	for(int x = 0; x < MAX_DEVICES * 2; x++)
	{
		if (nodesToListenTo[x] == 0) {
			bus.can_add_filter_mask(nodeId, 0x7F);
			nodesToListenTo[x] = nodeId;
			break;
		}
	}
}

void CANOPEN::setOpState(CANOPEN_OPSTATE opState)
{
	if(debug)
	{
		Serial.print("Setting opState to ");
		Serial.println(opState);
	}
}

void CANOPEN::sendStateChange(CANOPEN_OPSTATE newState)
{
	for (int x = 0; x < MAX_DEVICES; x++)
	{
		if (cbStateChange[x] != NULL)
		{
			cbStateChange[x](newState);
		}
	}
}

void CANOPEN::sendGotPDOMsg(CAN_FRAME* frame)
{
	for (int x = 0; x < MAX_DEVICES; x++)
	{
		if (cbGotPDOMsg[x] != NULL)
		{
			cbGotPDOMsg[x](frame);
		}
	}
}

void CANOPEN::sendGotSDOReq(SDO_FRAME* frame)
{
	for (int x = 0; x < MAX_DEVICES; x++)
	{
		if (cbGotSDOReq[x] != NULL)
		{
			cbGotSDOReq[x](frame);
		}
	}
}

void CANOPEN::sendGotSDOReply(SDO_FRAME* frame)
{
	for (int x = 0; x < MAX_DEVICES; x++)
	{
		if (cbGotSDOReply[x] != NULL)
		{
			cbGotSDOReply[x](frame);
		}
	}
}

void CANOPEN::sendGotHeartbeat(CAN_FRAME* frame)
{
	for (int x = 0; x < MAX_DEVICES; x++)
	{
		if (cbGotHeartbeatMsg[x] != NULL)
		{
			cbGotHeartbeatMsg[x](frame);
		}
	}
}


void CANOPEN::setHeartbeatInterval(uint32_t interval)
{
	heartbeatInterval = interval;
}

void CANOPEN::loop()
{
	if(bus.is_can_msg_pending())
	{
			// Serial.printf("There are %d messages in the buffer\r\n", HAL_CAN_GetRxFifoFillLevel(&bus.hcan, 0));
			CanMessage msg;
			CAN_FRAME frame;
			CanState state = bus.can_rx(&msg, 0);
			if (state == DATA_OK) {
				// Serial.print("Got a CanMessage: ");
				// Serial.print("\tdata\t");
				// for (auto& data : msg.data.uint8) {
				// 	Serial.printf("%#02X ",data);
				// }
				// Serial.print("\tid\t"); Serial.printf("%#02X ", msg.id);
				// Serial.print("\trtr\t"); Serial.print(msg.rtr);
				// Serial.print("\tlength\t"); Serial.print(msg.length);
				// Serial.println();
				canMessagToCanFrame(&msg, &frame);
				receiveFrame(&frame);
			} else
			{
				Serial.printf("ERROR: CanState is not the expected %d. It is %d\r\n", DATA_OK, state);
			}
	}
	if ((millis() - lastHeartbeat) >= heartbeatInterval)
	{
		sendHeartbeat();
	}
	if (opState == BOOTUP)
	{
		opState = PREOP;
	}
}

void CANOPEN::sendHeartbeat()
{
	CAN_FRAME frame;
	frame.id = 0x700 + nodeID;
	frame.length = 1;
	frame.extended = false;
	switch (opState)
	{
	case OPERATIONAL:
		frame.data.uint8[0] = 5;
		break;
	case STOPPED:
		frame.data.uint8[0] = 4;
		break;
	case PREOP:
		frame.data.uint8[0] = 0x7F;
		break;
	case BOOTUP:
		frame.data.uint8[0] = 0;
		break;
	}

	CanMessage msg;
	canFrameToCanMessage(&frame, &msg);
	CanState result = bus.can_tx(&msg, 0);
	if (result != CanState::BUS_OK) {
		digitalWrite(LED_BUILTIN, HIGH); // OFF
		Serial.printf("Result is not expected. Expected %d. Got %d\r\n", BUS_OK, result);
	}
	else
	{
		digitalWrite(LED_BUILTIN, LOW); // ON
	}
	lastHeartbeat = millis();
}

CANOPEN CanOpen(0);