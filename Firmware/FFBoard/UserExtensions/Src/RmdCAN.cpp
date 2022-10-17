/*
 * RmdCAN.cpp
 *
 *  Created on: October 19, 2022
 *      Author: Yannick
 * 				1Plus2Equals3D
 */


#include "target_constants.h"
#ifdef RMD
#include <RmdCAN.h>


// Instance setup

bool RmdCAN1::inUse = false;
ClassIdentifier RmdCAN1::info = {
		 .name = "Rmd (M0)" ,
		 .id=CLSID_MOT_RMD0,	// 5
};
bool RmdCAN2::inUse = false;
ClassIdentifier RmdCAN2::info = {
		 .name = "Rmd (M1)" ,
		 .id=CLSID_MOT_RMD1,	// 6
};

const ClassIdentifier RmdCAN1::getInfo(){
	return info;
}

const ClassIdentifier RmdCAN2::getInfo(){
	return info;
}

bool RmdCAN1::isCreatable(){
	return !RmdCAN1::inUse; // Creatable if not already in use for example by another axis
}

bool RmdCAN2::isCreatable(){
	return !RmdCAN2::inUse; // Creatable if not already in use for example by another axis
}

/*              De/Constructor                 */

RmdCAN::RmdCAN(uint8_t id)  : CommandHandler("rmd", CLSID_MOT_RMD0,id),  Thread("RMD", RMD_THREAD_MEM, RMD_THREAD_PRIO), motorId(id) {

	restoreFlash();

	// Set up a filter to receive rmd commands
	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	this->filterId = this->port->addCanFilter(sFilterConfig);

	if(port->getSpeedPreset() < 5){
		port->setSpeedPreset(5); // Minimum 1000k
	}

	this->port->setSilentMode(false);
	this->registerCommands();
	this->port->takePort();
	this->Start();
}

RmdCAN::~RmdCAN() {
	this->setTorque(0.0);
	this->port->removeCanFilter(filterId);
	this->port->freePort();
}

// Commands and Flash

void RmdCAN::registerCommands(){
	CommandHandler::registerCommands();
	registerCommand("canspd", RmdCAN_commands::canspd, "CAN baudrate",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("error", RmdCAN_commands::error, "Rmd error flags",CMDFLAG_GET);
	registerCommand("state", RmdCAN_commands::state, "Rmd state",CMDFLAG_GET);
	registerCommand("maxtorque", RmdCAN_commands::maxtorque, "Max torque to send for scaling",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("connected", RmdCAN_commands::connected, "Rmd connection state",CMDFLAG_GET);
	registerCommand("voltage", RmdCAN_commands::voltage, "Rmd voltage",CMDFLAG_GET);

}

void RmdCAN::restoreFlash(){
	uint16_t setting1addr = ADR_RMD_SETTING1_M0;
	//uint16_t canidaddr = ADR_RMD_CANID_M0;
	if (this->motorId == 1) {
		setting1addr = ADR_RMD_SETTING1_M1;
		//canidaddr = ADR_RMD_CANID_M1;
	}

	/*
	uint16_t canId = 0x0000;
	if(Flash_Read(canidaddr, &canId)){
		this->motorId = canId;
	}
	*/

	uint16_t settings1 = 0;
	if(Flash_Read(setting1addr, &settings1)){
		maxTorque = (float)clip(settings1 & 0xfff, 0, 0xfff) / 100.0;
	}
}

void RmdCAN::saveFlash(){
	uint16_t setting1addr = ADR_RMD_SETTING1_M0;
	//uint16_t canidaddr = ADR_RMD_CANID_M0;
	if (this->motorId == 1) {
		setting1addr = ADR_RMD_SETTING1_M1;
		//canidaddr = ADR_RMD_CANID_M1;
	}

	/*
	uint16_t canId = 0x3040;	
	Flash_Read(ADR_RMD_CANID, &canIds); // Read again
	if(motorId == 0){
		canIds &= ~0x3F; // reset bits
		canIds |= nodeId & 0x3f;
	}else if(motorId == 1){
		setting1addr = ADR_RMD_SETTING1_M1;
		canIds &= ~0xFC0; // reset bits
		canIds |= (nodeId & 0x3f) << 6;
	}
	canIds &= ~0x7000; // reset bits
	Flash_Write(ADR_RMD_CANID,canIds);
	*/

	uint16_t settings1 = ((int32_t)(maxTorque*100) & 0xfff);
	Flash_Write(setting1addr, settings1);
}

/*                        Run Loop                                */

void RmdCAN::Run(){
	while(true){
		this->Delay(500);

		if (motor_error != RmdError::none)
		{
			this->motorOff();
			motor_error = RmdError::none;
			state = RmdLocalState::IDLE;
			this->Delay(1000);
			continue;
		}
		switch(state){

			case RmdLocalState::IDLE:
				state = RmdLocalState::START_RUNNING;

			case RmdLocalState::START_RUNNING:
				state = RmdLocalState::RUNNING;
				// driver is active,
				// enable torque mode
				this->setPos(0);
				this->startMotor();
				break;

			default:

				break;
		}

		if(HAL_GetTick() - lastVoltageUpdate > 1000){
			sendCmd(RmdCmd::read_status_1); // Update voltage
		}

		if(HAL_GetTick() - lastCanMessage > 2000){ // Timeout
			state = RmdLocalState::IDLE;
			connected = false;
		}else{
			connected = true;
		}

	}
}


/*                      START/STOP/State                           */

void RmdCAN::motorOff(){
	active = false;
	sendCmd(RmdCmd::motor_off);
}


void RmdCAN::stopMotor(){
    // Temporarily calling "motor_off" instead as I am unsure if this is a hard stop or should stop closed loop calculation
    //	active = false; 
    //	this->setTorque(0.0);
    //	sendCmd(RmdCmd::motor_stop);
    this->motorOff();
}

void RmdCAN::startMotor(){
	active = true;
	this->setTorque(0.0);
}

bool RmdCAN::motorReady(){
	if (state == RmdLocalState::RUNNING)
		return true;
	else
		return false;
}


/*                          CAN                            */

void RmdCAN::sendCmd(RmdCmd cmd){
    uint8_t buffer[8] = {0};
    buffer[0] = (uint8_t)cmd;
    this->sendMsg(buffer);
}

void RmdCAN::sendMsg(uint8_t *buffer){
	CAN_tx_msg msg;
    memcpy(&msg.data, buffer, 8);
	msg.header.RTR = CAN_RTR_REMOTE;
	msg.header.DLC = 0;
	msg.header.StdId = outgoing_base_id + motorId + 1;
	port->sendMessage(msg);
}


/*						Direct CANBus incoming messages					*/

void RmdCAN::canRxPendCallback(CAN_HandleTypeDef *hcan,uint8_t* rxBuf,CAN_RxHeaderTypeDef* rxHeader,uint32_t fifo){
	
	uint16_t node = rxHeader->StdId - incoming_base_id - 1;

	if(node != this->motorId){
		return;
	}
	RmdCmd cmd = (RmdCmd)rxBuf[0];

	uint8_t buffer[8];
	memcpy(buffer,rxBuf,8);

	lastCanMessage = HAL_GetTick();

	switch(cmd){
		case RmdCmd::read_status_1:
		{
			RmdError error = (RmdError)buffer_get_uint16(buffer, 6);

			if (error != RmdError::none)
			{
				motor_error = error;
				state =RmdLocalState::IDLE;
			}

			lastVoltageUpdate = HAL_GetTick();
			float voltage =	((float)buffer_get_uint16(buffer, 4))/10;
			memcpy(&lastVoltage,&voltage,sizeof(float));

			break;
		}

		case RmdCmd::read_multiturn_position: // encoder pos float
		{
			float out = (float)buffer_get_int32(buffer, 4);
			memcpy(&lastPos,&out,sizeof(float));
			break;
		}


		default:
			break;
	}	


}

/*              Encoder                                     */

Encoder* RmdCAN::getEncoder(){
	return static_cast<Encoder*>(this);
}

EncoderType RmdCAN::getEncoderType(){
	return EncoderType::absolute;
}


void RmdCAN::setPos(int32_t pos){
	// Only change encoder count internally as offset
	posOffset = lastPos - ((float)pos / (float)getCpr());
}

float RmdCAN::getPos_f(){
	if(this->connected)
		sendCmd(RmdCmd::read_multiturn_position);
	return lastPos-posOffset;
}   

int32_t RmdCAN::getPos(){
	return getCpr() * getPos_f();
}

uint32_t RmdCAN::getCpr(){
	return 0xffff;
}


/*              Movement                        */


/**
 * Turn the motor with positive/negative power.
 * Range should be full signed 16 bit
 * A value of 0 should have no torque. The sign is the direction.
 */
void RmdCAN::turn(int16_t power){
	float torque = ((float)power / (float)0x7FFF) * maxTorque; // 0x7FFF is the int16_t max.    
	this->setTorque(torque); // Send the signed torque(max of 1.0, min of -1.0)
}

// Input must be between -1.0 and 1.0
void RmdCAN::setTorque(float torque){
	if(motorReady())
	{
		uint8_t buffer[8] = {0};
		buffer[0] = (uint8_t)RmdCmd::torque_closed_loop;
		int16_t output_torque  = (int16_t)((base_amps * 100)*torque);
		buffer_append_int16(buffer, output_torque, 4);
		sendMsg(buffer);
	}
}

/*              Interal Command Control             */

CommandStatus RmdCAN::command(const ParsedCommand& cmd,std::vector<CommandReply>& replies){

	switch(static_cast<RmdCAN_commands>(cmd.cmdId)){
	case RmdCAN_commands::voltage:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(lastVoltage);
		}else{
			return CommandStatus::ERR;
		}
		break;

	case RmdCAN_commands::error:
		if(cmd.type == CMDtype::get){
			replies.emplace_back((uint16_t)motor_error);
		}else{
			return CommandStatus::ERR;
		}
		break;

	case RmdCAN_commands::state:
		if(cmd.type == CMDtype::get){
			replies.emplace_back((uint32_t)state);
		}else{
			return CommandStatus::ERR;
		}
		break;
	case RmdCAN_commands::canspd:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(port->getSpeedPreset());
		}else if(cmd.type == CMDtype::set){
			port->setSpeedPreset(std::max<uint8_t>(3,cmd.val));
		}else{
			return CommandStatus::ERR;
		}
		break;
	case RmdCAN_commands::maxtorque:
	{
		if(cmd.type == CMDtype::get){
			int32_t val = maxTorque*100;
			replies.emplace_back(val);
		}else if(cmd.type == CMDtype::set){
			maxTorque = (float)clip(cmd.val, 0, 0xfff) / 100.0;
		}else{
			return CommandStatus::ERR;
		}
		break;
	}
	case RmdCAN_commands::connected:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(connected ? 1 : 0);
		}
		break;

	default:
		return CommandStatus::NOT_FOUND;
	}

	return CommandStatus::OK;

}


/*              ***** Helper Functions *****             */


float normalize(float input, float min, float max)
{
    float average = (min + max)/2;
    float range = (max - min)/2;
    float normalized_x = (input - average) / range;
    return normalized_x;
}

void buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t index) {
    buffer_append_int32(buffer, (int32_t) (number * scale), index);
}

// Index is where I want the item to be placed
void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t index) {
    buffer[(index)++] = number >> 24;
    buffer[(index)++] = number >> 16;
    buffer[(index)++] = number >> 8;
    buffer[(index)++] = number;
}
// Index is where I want the item to be placed
void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t index) {
    buffer[(index)++] = number >> 8;
    buffer[(index)++] = number;
}

void buffer_append_uint32(uint8_t *buffer, uint32_t number, int32_t index) {
    buffer[(index)++] = number >> 24;
    buffer[(index)++] = number >> 16;
    buffer[(index)++] = number >> 8;
    buffer[(index)++] = number;
}

uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t index) {
    uint32_t res = ((uint32_t) buffer[index]) << 24
            | ((uint32_t) buffer[index + 1]) << 16
            | ((uint32_t) buffer[index + 2]) << 8
            | ((uint32_t) buffer[index + 3]);
    return res;
}

uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t index) {
    uint16_t res = ((uint16_t) buffer[index]) << 8
            | ((uint16_t) buffer[index + 1]);
    return res;
}

int32_t buffer_get_int32(const uint8_t *buffer, int32_t index) {
    int32_t res = ((uint32_t) buffer[index]) << 24
            | ((uint32_t) buffer[index + 1]) << 16
            | ((uint32_t) buffer[index + 2]) << 8
            | ((uint32_t) buffer[index + 3]);
    return res;
}

int16_t buffer_get_int16(const uint8_t *buffer, int32_t index) {
    int16_t res = ((uint16_t) buffer[index]) << 8
            | ((uint16_t) buffer[index + 1]);
    return res;
}

float buffer_get_float32(const uint8_t *buffer, float scale, int32_t index) {
    return (float) buffer_get_int32(buffer, index) / scale;
}

float buffer_get_float16(const uint8_t *buffer, float scale, int32_t index) {
    return (float) buffer_get_int16(buffer, index) / scale;
}

#endif