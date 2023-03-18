/*
 * RmdCAN.cpp
 *
 *  Created on: October 19, 2022
 *      Author: Yannick
 * 				1Plus2Equals3D
 * 				PilotWave11
 */

#include "target_constants.h"
#ifdef RMD
#include <RmdCAN.h>

#define PI_F 3.14159265f

// Instance setup

bool RmdCAN1::inUse = false;
ClassIdentifier RmdCAN1::info = {
    .name = "Rmd (M0)",
    .id = CLSID_MOT_RMD0,  // 5
};
bool RmdCAN2::inUse = false;
ClassIdentifier RmdCAN2::info = {
    .name = "Rmd (M1)",
    .id = CLSID_MOT_RMD1,  // 6
};

const ClassIdentifier RmdCAN1::getInfo() { return info; }

const ClassIdentifier RmdCAN2::getInfo() { return info; }

bool RmdCAN1::isCreatable() {
  return !RmdCAN1::inUse;  // Creatable if not already in use for example by another axis
}

bool RmdCAN2::isCreatable() {
  return !RmdCAN2::inUse;  // Creatable if not already in use for example by another axis
}

RmdCAN::RmdCAN(uint8_t id)
    : CommandHandler("rmd", CLSID_MOT_RMD0, id), Thread("RMD", RMD_THREAD_MEM, RMD_THREAD_PRIO), motorId(id) {
  setAddress(id);

  restoreFlash();

  setCanFilter();

  if (port->getSpeedPreset() < 5) {
    port->setSpeedPreset(5);  // Minimum 1000k
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

// Set up a filter to receive odrive commands
void RmdCAN::setCanFilter() {
  CAN_FilterTypeDef sFilterConfig;
  uint16_t can_rx_id = canId + incoming_base_id;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = can_rx_id << 5;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0b11111111111 << 5;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = motorId % 2 == 0 ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  this->filterId = this->port->addCanFilter(sFilterConfig);
}

void RmdCAN::setAddress(uint8_t id) {
  if (id == 0) {
    this->flashAddrs = RmdFlashAddrs({ADR_RMD_CANID_M0, ADR_RMD_MAXTORQUE_M0, ADR_RMD_OFFSET_M0});
  } else if (id == 1) {
    this->flashAddrs = RmdFlashAddrs({ADR_RMD_CANID_M1, ADR_RMD_MAXTORQUE_M1, ADR_RMD_OFFSET_M1});
  }
}

// Commands and Flash
void RmdCAN::registerCommands() {
  CommandHandler::registerCommands();
  registerCommand("canid", RmdCAN_commands::canid, "CAN ID", CMDFLAG_GET | CMDFLAG_SET);
  registerCommand("canspd", RmdCAN_commands::canspd, "CAN baudrate", CMDFLAG_GET | CMDFLAG_SET);
  registerCommand("error", RmdCAN_commands::error, "Rmd error flags", CMDFLAG_GET);
  registerCommand("state", RmdCAN_commands::state, "Rmd state", CMDFLAG_GET | CMDFLAG_SET);
  registerCommand("maxtorque", RmdCAN_commands::maxtorque, "Max torque to send for scaling", CMDFLAG_GET | CMDFLAG_SET);
  registerCommand("connected", RmdCAN_commands::connected, "Rmd connection state", CMDFLAG_GET);
  registerCommand("voltage", RmdCAN_commands::voltage, "Rmd voltage", CMDFLAG_GET);
  registerCommand("epos", RmdCAN_commands::encpos, "Rmd multi-turn relative position (pulses)", CMDFLAG_GET);
  registerCommand("homepos", RmdCAN_commands::homepos, "Rmd multi-turn home position (pulses)", CMDFLAG_GET);
  registerCommand("zoff", RmdCAN_commands::zerooffset, "Rmd multi-turn zero offset (pulses)", CMDFLAG_GET);
  registerCommand("apos", RmdCAN_commands::abspos, "Rmd abs position (deg)", CMDFLAG_GET | CMDFLAG_SET);
  registerCommand("ipos", RmdCAN_commands::incpos, "Rmd inc position (deg)", CMDFLAG_GET | CMDFLAG_SET);
  registerCommand("spd", RmdCAN_commands::spd, "Rmd speed closed loop", CMDFLAG_GET | CMDFLAG_SET);
  registerCommand("trackpos", RmdCAN_commands::trackpos, "Rmd pos tracking", CMDFLAG_GET | CMDFLAG_SET);
  registerCommand("torque", RmdCAN_commands::torque, "Current torque", CMDFLAG_GET);

  registerCommand("debug", RmdCAN_commands::debug, "debugging_data", CMDFLAG_GET);
}

void RmdCAN::restoreFlash() {
  uint16_t dataFlash = 0;

  if (Flash_Read(flashAddrs.canId, &dataFlash)) {
    this->canId = dataFlash;
  }

  if (Flash_Read(flashAddrs.maxTorque, &dataFlash)) {
    this->maxTorque = (float)clip(dataFlash & 0xfff, 0, 0xfff) / 100.0;
  }

  if (Flash_Read(flashAddrs.offset, &dataFlash)) {
    this->posOffset = (float)((int16_t)dataFlash / 10000.0);
  }
}

void RmdCAN::saveFlash() {
  uint16_t cid = canId;
  Flash_Write(flashAddrs.canId, cid);

  uint16_t mt = ((int32_t)(maxTorque * 100) & 0xfff);
  Flash_Write(flashAddrs.maxTorque, mt);

  uint16_t offset = ((int16_t)(this->posOffset * 10000) & 0xFFFF);
  Flash_Write(flashAddrs.offset, offset);
}

void RmdCAN::Run() {
  while (true) {
    this->Delay(500);

    if (motor_error != RmdError::none) {
      this->motorOff();
      motor_error = RmdError::none;
      state = RmdLocalState::IDLE;
      this->Delay(1000);
      continue;
    }
    switch (state) {
      case RmdLocalState::IDLE:
        sendCmd(RmdCmd::read_multiturn_position);
        sendCmd(RmdCmd::read_multiturn_angle);
        this->Delay(500);
        setPos(0);
        state = RmdLocalState::WAIT_READY;
        break;

      case RmdLocalState::WAIT_READY:
	    sendCmd(RmdCmd::read_singleturn_position);
        break;

      case RmdLocalState::WAIT_HOMING:
        break;

      case RmdLocalState::WAIT_HOMING_DONE:
        state = RmdLocalState::START_RUNNING;

      case RmdLocalState::START_RUNNING:
        state = RmdLocalState::RUNNING;
        // driver is active,
        // enable torque mode
        // this->getPosOffset();
        this->startMotor();
        break;

      default:
        break;
    }

    if (HAL_GetTick() - lastVoltageUpdate > 1000) {
      sendCmd(RmdCmd::read_status_1);  // Update voltage
      this->Delay(5);
      sendCmd(RmdCmd::system_operating_mode);  // Get axis state
    }

    if (HAL_GetTick() - lastCanMessage > 2000) {  // Timeout
      state = RmdLocalState::IDLE;
      connected = false;
    } else {
      connected = true;
    }
  }
}

void RmdCAN::motorOff() {
  active = false;
  sendCmd(RmdCmd::motor_shutdown);
}

void RmdCAN::stopMotor() {
  // Temporarily calling "motor_shutdown" instead as I am unsure if this is a hard stop or should
  // stop closed loop calculation
  active = false;
  this->setTorque(0.0);
  this->Delay(5);
  sendCmd(RmdCmd::motor_stop);
  this->Delay(5);
  this->motorOff();
}

void RmdCAN::startMotor() {
  active = true;
  this->setTorque(0.0);
}

bool RmdCAN::motorReady() { return (this->connected && state == RmdLocalState::RUNNING); }

void RmdCAN::resetPID() {
  uint8_t data[8];
  data[0] = 0x32;                                 // Command
  data[1] = 0x00;                                 // Null
  data[2] = (uint8_t)RmdPIDSettings::CurrentKp;   // Current Kp
  data[3] = (uint8_t)RmdPIDSettings::CurrentKi;   // Current Ki
  data[4] = (uint8_t)RmdPIDSettings::SpeedKp;     // Speed Kp
  data[5] = (uint8_t)RmdPIDSettings::SpeedKi;     // Speed Ki
  data[6] = (uint8_t)RmdPIDSettings::PositionKp;  // Position Kp
  data[7] = (uint8_t)RmdPIDSettings::PositionKi;  // Position Ki

  this->sendMsg(data);
}

void RmdCAN::sendCmd(RmdCmd cmd) {
  uint8_t buffer[8] = {0};
  buffer[0] = (uint8_t)cmd;
  this->sendMsg(buffer);
}

void RmdCAN::sendMsg(uint8_t *buffer) {
  CAN_tx_msg msg;

  msg.header.RTR = CAN_RTR_DATA;
  msg.header.DLC = 8;
  msg.header.IDE = 0;
  msg.header.StdId = outgoing_base_id + canId;
  memcpy(&msg.data, buffer, 8 * sizeof(uint8_t));

  this->_debug.lastOutCmd = buffer[0];
  this->_debug.lastOutAddress = msg.header.StdId;

  if (!port->sendMessage(msg)) {
    // Nothing.
  };
}

void RmdCAN::canErrorCallback(CAN_HandleTypeDef *hcan) {
  // pulseErrLed();
}

void RmdCAN::canRxPendCallback(CAN_HandleTypeDef *hcan, uint8_t *rxBuf, CAN_RxHeaderTypeDef *rxHeader, uint32_t fifo) {
  this->_debug.lastInAddress = rxHeader->StdId;
  this->_debug.lastInCmd = rxBuf[0];

  uint16_t canNodeId = rxHeader->StdId - incoming_base_id;

  if (canNodeId != this->canId) {
    return;
  }
  RmdCmd cmd = static_cast<RmdCmd>(rxBuf[0]);

  lastCanMessage = HAL_GetTick();

  switch (cmd) {
    case RmdCmd::read_singleturn_position:  // 0x90
    {
      float epos = static_cast<float>(buffer_get_int16(rxBuf, 2));
      float cpr = getCpr();
      this->lastPos = static_cast<float>(epos / cpr);
      this->lastPosTime = HAL_GetTick();
      this->posWaiting = false;
      break;
    }

    case RmdCmd::read_multiturn_position:  // 0x60
    {
      float epos = static_cast<float>(buffer_get_int32(rxBuf, 4));
      float cpr = getCpr();
      this->lastPos = static_cast<float>(epos / cpr);
      this->lastPosTime = HAL_GetTick();
      this->posWaiting = false;
      break;
    }

    case RmdCmd::read_home_position:  // 0x61
    {
      this->homePos = buffer_get_int32(rxBuf, 4);
      break;
    }

    case RmdCmd::read_zero_offset:  // 0x62
    {
      // this->posOffset = buffer_get_int32(rxBuf, 4);
      break;
    }

    case RmdCmd::system_operating_mode:  // 0x70
    {
      this->axisState = static_cast<RmdControlMode>(buffer_get_uint8(rxBuf, 7));
      break;
    }

    case RmdCmd::torque_closed_loop:  // 0xA1
    {
      this->currentTorque = buffer_get_int16(rxBuf, 2);
      break;
    }

    case RmdCmd::speed_closed_loop:  // 0xA2
    {
      /* 1 dps/LSB precision */
      this->lastSpeed = buffer_get_int16(rxBuf, 4);
      break;
    }

    case RmdCmd::pos_tracking_control:  // 0xA3
    {
      /* 1 dps/LSB precision */
      this->lastSpeed = buffer_get_int16(rxBuf, 4);
      break;
    }

    case RmdCmd::abs_pos_closed_loop:  // 0xA4
    {
      /* 1 dps/LSB precision */
      this->lastSpeed = buffer_get_int16(rxBuf, 4);
      break;
    }

    case RmdCmd::inc_pos_closed_loop:  // 0xA8
    {
      /* 1 dps/LSB precision */
      this->lastSpeed = buffer_get_int16(rxBuf, 4);
      break;
    }

    case RmdCmd::read_multiturn_angle:  // 0x92
    {
      /* 0.01 deg/LSB precision, leave as-is for now */
      this->lastAng = buffer_get_int32(rxBuf, 4);
      break;
    }

    case RmdCmd::read_status_1:  // 0x9A
    {
      this->motor_error = static_cast<RmdError>(buffer_get_uint16(rxBuf, 6));
      if (motor_error != RmdError::none) {
        state = RmdLocalState::IDLE;
      }
      lastVoltageUpdate = HAL_GetTick();
      this->lastVoltage = (float)buffer_get_uint16(rxBuf, 4);
      break;
    }

    default:
      break;
  }
}

Encoder *RmdCAN::getEncoder() { return static_cast<Encoder *>(this); }

EncoderType RmdCAN::getEncoderType() { return EncoderType::absolute; }

void RmdCAN::getPosOffset() {
  // Only change encoder count internally as offset
  // if(this->connected)
  // sendCmd(RmdCmd::read_zero_offset);
  // posOffset = lastPos - ((float)pos / (float)getCpr());
}

void RmdCAN::setPos(int32_t pos) {
  // Only change encoder count internally as offset
  posOffset = lastPos;
  angOffset = lastAng;
}

float RmdCAN::getPos_f() {
  if (motorReady() && (!posWaiting || HAL_GetTick() - lastPosTime > 5)) {
    posWaiting = true;
    sendCmd(RmdCmd::read_multiturn_position);
    sendCmd(RmdCmd::read_multiturn_angle);
  }
  return lastPos - posOffset;
}

int32_t RmdCAN::getPos() { return getPos_f() * getCpr(); }
uint32_t RmdCAN::getCpr() { return 16384; }

/**
 * Turn the motor with positive/negative power.
 * Range should be full signed 16 bit
 * A value of 0 should have no torque. The sign is the direction.
 */
void RmdCAN::turn(int16_t power) {
  float torque = ((float)power / (float)0x7FFF) * maxTorque;  // 0x7FFF is the int16_t max.
  this->setTorque(torque);                                    // Send the signed torque(max of 1.0, min of -1.0)
}

// Input must be between -1.0 and 1.0
void RmdCAN::setTorque(float torque) {
  if (motorReady()) {
    uint8_t buffer[8] = {0};
    buffer[0] = (uint8_t)RmdCmd::torque_closed_loop;
    int16_t output_torque;
    if (torque == 0.0)
      output_torque = 0;
    else
      /* 0.01 A/LSB precision */
      output_torque = (int16_t)(1000 * torque);
    buffer_append_int16(buffer, output_torque, 4);
    sendMsg(buffer);
  }
}

//
void RmdCAN::moveToPosition(float posDegrees) {
  //   if (motorReady()) {
  uint8_t buffer[8] = {0};
  buffer[0] = (uint8_t)RmdCmd::abs_pos_closed_loop;  // 0xA4
  uint16_t maxspd = 100;
  buffer_append_uint16(buffer, maxspd, 2);
  /* posDegrees is assumed to be in deg,
  and precision is 0.01 deg/LSB */
  int32_t absAng = (posDegrees * 100.0 + angOffset);
  buffer_append_int32(buffer, absAng, 4);
  sendMsg(buffer);
  //   }
}

void RmdCAN::executeMotionPlan(float pos, float vel, float kp, float kd, float t_ff) {
  uint8_t buffer[8] = {0};

  float pos_rad = pos * PI_F / 180.0f;
  const float pos_offset = -12.5f;   // rad
  const float pos_maxRange = 25.0f;  // rad

  uint16_t pos_i = (pos_rad - pos_offset) / pos_maxRange * 0xFFFF;

  const float vel_offset = -45.0f;   // rad/s
  const float vel_maxRange = 90.0f;  // rad/s

  const float kp_max = 500.0f;
  const float kd_max = 5.0f;

  const float t_offset = -24.0f;   // N.m
  const float t_maxRange = 48.0f;  // N.m
}

CommandStatus RmdCAN::command(const ParsedCommand &cmd, std::vector<CommandReply> &replies) {
  switch (static_cast<RmdCAN_commands>(cmd.cmdId)) {
    case RmdCAN_commands::canid:
      if (cmd.type == CMDtype::get) {
        replies.emplace_back(canId);
      } else if (cmd.type == CMDtype::set) {
        canId = cmd.val;
        this->port->removeCanFilter(filterId);
        setCanFilter();
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::canspd:
      if (cmd.type == CMDtype::get) {
        replies.emplace_back(port->getSpeedPreset());
      } else if (cmd.type == CMDtype::set) {
        port->setSpeedPreset(std::max<uint8_t>(3, cmd.val));
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::error:
      if (cmd.type == CMDtype::get) {
        replies.emplace_back((uint16_t)motor_error);
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::state:
      if (cmd.type == CMDtype::get) {
        replies.emplace_back((uint32_t)state);
      } else if (cmd.type == CMDtype::set) {
        state = static_cast<RmdLocalState>(cmd.val);
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::maxtorque:
      if (cmd.type == CMDtype::get) {
        int32_t val = maxTorque * 100;
        replies.emplace_back(val);
      } else if (cmd.type == CMDtype::set) {
        maxTorque = (float)clip(cmd.val, 0, 0xfff) / 100.0;
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::connected:
      if (cmd.type == CMDtype::get) {
        replies.emplace_back(connected ? 1 : 0);
      }
      break;

    case RmdCAN_commands::voltage:
      if (cmd.type == CMDtype::get) {
        replies.emplace_back(lastVoltage);
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::encpos:
      if (cmd.type == CMDtype::get) {
        float pos = lastPos;
        replies.emplace_back(pos);
      }
      break;

    case RmdCAN_commands::torque:
      if (cmd.type == CMDtype::get) {
        replies.emplace_back(this->currentTorque);
      }
      break;

    case RmdCAN_commands::homepos:
      if (cmd.type == CMDtype::get) {
        sendCmd(RmdCmd::read_home_position);
        replies.emplace_back((int32_t)this->homePos);
      }
      break;

    case RmdCAN_commands::zerooffset:
      if (cmd.type == CMDtype::get) {
        sendCmd(RmdCmd::read_zero_offset);
        replies.emplace_back(this->posOffset);
      }
      break;

    case RmdCAN_commands::abspos:
      if (cmd.type == CMDtype::get) {
        replies.emplace_back(lastAng - angOffset);
      } else if (cmd.type == CMDtype::set) {
        this->moveToPosition(cmd.val);
        this->Delay(750);
        sendCmd(RmdCmd::motor_shutdown);
      }
      break;

    case RmdCAN_commands::incpos:
      if (cmd.type == CMDtype::get) {
        break;
      } else if (cmd.type == CMDtype::set) {
        if (motorReady()) {
          uint8_t buffer[8] = {0};
          buffer[0] = (uint8_t)RmdCmd::inc_pos_closed_loop;
          uint16_t maxspd = 250;
          buffer_append_uint16(buffer, maxspd, 2);
          /* cmd.val is assumed to be in deg,
          and precision is 0.01 deg/LSB */
          int32_t incAng = cmd.val * 100;
          buffer_append_int32(buffer, incAng, 4);
          sendMsg(buffer);
        }
      }
      break;

    case RmdCAN_commands::spd:
      if (cmd.type == CMDtype::get) {
        replies.emplace_back(this->lastSpeed);
      } else if (cmd.type == CMDtype::set) {
        if (motorReady()) {
          uint8_t buffer[8] = {0};
          buffer[0] = (uint8_t)RmdCmd::speed_closed_loop;
          /* cmd.val is assumed to be in dps,
          and precision is 0.01 dps/LSB */
          int32_t spd = cmd.val * 100;
          buffer_append_int32(buffer, spd, 4);
          sendMsg(buffer);
        }
      }
      break;

    case RmdCAN_commands::trackpos:
      if (cmd.type == CMDtype::get) {
        break;
      } else if (cmd.type == CMDtype::set) {
        if (motorReady()) {
          uint8_t buffer[8] = {0};
          buffer[0] = (uint8_t)RmdCmd::pos_tracking_control;
          /* cmd.val is assumed to be in deg,
          and precision is 0.01 deg/LSB */
          int32_t pos = cmd.val * 100;
          buffer_append_int32(buffer, pos, 4);
          sendMsg(buffer);
        }
      }
      break;

    case RmdCAN_commands::debug:
      if (cmd.type == CMDtype::get) {
        replies.emplace_back((uint32_t)this->_debug.lastInAddress);
        replies.emplace_back((uint32_t)this->_debug.lastOutAddress);
        replies.emplace_back((uint32_t)this->_debug.lastInCmd);
        replies.emplace_back((uint32_t)this->_debug.lastOutCmd);
      }
      break;

    default:
      return CommandStatus::NOT_FOUND;
  }

  return CommandStatus::OK;
}

float normalize(float input, float min, float max) {
  float average = (min + max) / 2;
  float range = (max - min) / 2;
  float normalized_x = (input - average) / range;
  return normalized_x;
}

// Index is where I want the item to be placed
void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t index) {
  buffer[(index)++] = number;
  buffer[(index)++] = number >> 8;
}

void buffer_append_uint16(uint8_t *buffer, uint16_t number, int32_t index) {
  buffer[(index)++] = number;
  buffer[(index)++] = number >> 8;
}

void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t index) {
  buffer[(index)++] = number;
  buffer[(index)++] = number >> 8;
  buffer[(index)++] = number >> 16;
  buffer[(index)++] = number >> 24;
}

void buffer_append_uint32(uint8_t *buffer, uint32_t number, int32_t index) {
  buffer[(index)++] = number;
  buffer[(index)++] = number >> 8;
  buffer[(index)++] = number >> 16;
  buffer[(index)++] = number >> 24;
}

uint8_t buffer_get_uint8(const uint8_t *buffer, int32_t index) {
  uint8_t res = ((uint8_t)buffer[index]);
  return res;
}

int16_t buffer_get_int16(const uint8_t *buffer, int32_t index) {
  int16_t res = ((uint16_t)buffer[index + 1]) << 8 | ((uint16_t)buffer[index]);
  return res;
}

uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t index) {
  uint16_t res = ((uint16_t)buffer[index + 1]) << 8 | ((uint16_t)buffer[index]);
  return res;
}

int32_t buffer_get_int32(const uint8_t *buffer, int32_t index) {
  int32_t res = ((uint32_t)buffer[index + 3]) << 24 | ((uint32_t)buffer[index + 2]) << 16 |
                ((uint32_t)buffer[index + 1]) << 8 | ((uint32_t)buffer[index]);
  return res;
}

uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t index) {
  uint32_t res = ((uint32_t)buffer[index + 3]) << 24 | ((uint32_t)buffer[index + 2]) << 16 |
                 ((uint32_t)buffer[index + 1]) << 8 | ((uint32_t)buffer[index]);
  return res;
}
#endif
