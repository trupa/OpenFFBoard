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
#include <math.h>

#define PI_F 3.14159265f

// Instance setup

bool RmdCAN1::inUse           = false;
ClassIdentifier RmdCAN1::info = {
    .name = "Rmd (M0)",
    .id   = CLSID_MOT_RMD0,  // 5
};
bool RmdCAN2::inUse           = false;
ClassIdentifier RmdCAN2::info = {
    .name = "Rmd (M1)",
    .id   = CLSID_MOT_RMD1,  // 6
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
  uint16_t can_rx_id                 = canId + incoming_base_id;
  sFilterConfig.FilterBank           = 0;
  sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh         = can_rx_id << 5;
  sFilterConfig.FilterIdLow          = 0x0000;
  sFilterConfig.FilterMaskIdHigh     = 0b11111111111 << 5;
  sFilterConfig.FilterMaskIdLow      = 0x0000;
  sFilterConfig.FilterFIFOAssignment = motorId % 2 == 0 ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
  sFilterConfig.FilterActivation     = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  this->filterId                     = this->port->addCanFilter(sFilterConfig);
}

void RmdCAN::setAddress(uint8_t id) {
  if (id == 0) {
    this->flashAddrs = RmdFlashAddrs({ADR_RMD_CANID_M0, ADR_RMD_MAXTORQUE_M0, ADR_RMD_OFFSET_M0, ADR_RMD_PID_0_M0,
                                      ADR_RMD_PID_1_M0, ADR_RMD_PID_2_M0});
  } else if (id == 1) {
    this->flashAddrs = RmdFlashAddrs({ADR_RMD_CANID_M1, ADR_RMD_MAXTORQUE_M1, ADR_RMD_OFFSET_M1, ADR_RMD_PID_0_M1,
                                      ADR_RMD_PID_1_M1, ADR_RMD_PID_2_M1});
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
  registerCommand("single_pos", RmdCAN_commands::single_pos, "Rmd read single-turn encoder command", CMDFLAG_GET);
  registerCommand("single_offset", RmdCAN_commands::single_offset, "Rmd read single-turn zero offset", CMDFLAG_GET);
  registerCommand("multi_pos", RmdCAN_commands::multi_pos, "Rmd read multi-turn encoder command", CMDFLAG_GET);
  registerCommand("single_ang", RmdCAN_commands::single_ang, "Rmd read single-turn angle command", CMDFLAG_GET);
  registerCommand("multi_ang", RmdCAN_commands::multi_ang, "Rmd read multi-turn angle command", CMDFLAG_GET);
  registerCommand("pos_turns", RmdCAN_commands::pos_turns, "Rmd multi-turn relative position (turns)", CMDFLAG_GET);
  registerCommand("pos_turns_offset", RmdCAN_commands::pos_turns_offset, "Get the internally stored offset (turns)",
                  CMDFLAG_GET | CMDFLAG_SET);
  registerCommand("multi_pos_raw", RmdCAN_commands::multi_pos_raw, "Rmd multi-turn raw position (pulses)", CMDFLAG_GET);
  registerCommand("multi_offset", RmdCAN_commands::multi_offset, "Rmd multi-turn zero offset (pulses)",
                  CMDFLAG_GET | CMDFLAG_SET);
  registerCommand("start", RmdCAN_commands::start, "Start the Rmd motor", CMDFLAG_SET);
  registerCommand("stop", RmdCAN_commands::stop, "Stop the Rmd motor", CMDFLAG_SET);
  registerCommand("brake", RmdCAN_commands::brake, "Engage/disengage the Rmd brake", CMDFLAG_SET);
  registerCommand("baudrate", RmdCAN_commands::baudrate, "Rmd set baudrates for RS485/CAN", CMDFLAG_SET);
  registerCommand("function", RmdCAN_commands::function, "Rmd function command. Takes a control value.", CMDFLAG_SET);
  registerCommand("pid", RmdCAN_commands::pid, "Rmd get/set PID values", CMDFLAG_GET | CMDFLAG_SET);
  registerCommand("motion", RmdCAN_commands::motion, "Rmd issue motion command", CMDFLAG_GET | CMDFLAG_SETADR);
  registerCommand("apos", RmdCAN_commands::abspos, "Rmd abs position (deg) control command", CMDFLAG_GET | CMDFLAG_SET);
  registerCommand("ipos", RmdCAN_commands::incpos, "Rmd inc position (deg) control command", CMDFLAG_GET | CMDFLAG_SET);
  registerCommand("spd", RmdCAN_commands::spd, "Rmd speed closed loop command", CMDFLAG_GET | CMDFLAG_SET);
  registerCommand("torque", RmdCAN_commands::torque, "Rmd current torque (A)", CMDFLAG_GET);
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

  if (Flash_Read(flashAddrs.pid0, &dataFlash)) {
    this->pidSettings.CurrentKp = dataFlash & 0xFF;
    this->pidSettings.CurrentKi = (dataFlash >> 8) & 0xFF;
  }
  if (Flash_Read(flashAddrs.pid1, &dataFlash)) {
    this->pidSettings.SpeedKp = dataFlash & 0xFF;
    this->pidSettings.SpeedKi = (dataFlash >> 8) & 0xFF;
  }
  if (Flash_Read(flashAddrs.pid2, &dataFlash)) {
    this->pidSettings.PositionKp = dataFlash & 0xFF;
    this->pidSettings.PositionKi = (dataFlash >> 8) & 0xFF;
  }
}

void RmdCAN::saveFlash() {
  uint16_t cid = canId;
  Flash_Write(flashAddrs.canId, cid);

  uint16_t mt = ((int32_t)(maxTorque * 100) & 0xfff);
  Flash_Write(flashAddrs.maxTorque, mt);

  uint16_t offset = ((int16_t)(this->posOffset * 10000) & 0xFFFF);
  Flash_Write(flashAddrs.offset, offset);

  uint16_t params = this->pidSettings.CurrentKp | ((uint16_t)this->pidSettings.CurrentKi << 8);
  Flash_Write(flashAddrs.pid0, params);

  params = this->pidSettings.SpeedKp | ((uint16_t)this->pidSettings.SpeedKi << 8);
  Flash_Write(flashAddrs.pid1, params);

  params = this->pidSettings.PositionKp | ((uint16_t)this->pidSettings.PositionKi << 8);
  Flash_Write(flashAddrs.pid2, params);
}

void RmdCAN::Run() {
  while (true) {
    this->Delay(250);

    if (motor_error != RmdError::none) {
      this->motorOff();
      motor_error = RmdError::none;
      // state       = RmdLocalState::IDLE;
      this->Delay(1000);
      continue;
    }
    switch (state) {
      case RmdLocalState::IDLE:
        state = RmdLocalState::WAIT_READY;
        break;

      case RmdLocalState::WAIT_READY:
        break;

      case RmdLocalState::START_RUNNING:
        startMotor();
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
      state     = RmdLocalState::IDLE;
      connected = false;
    } else {
      connected = true;
    }
  }
}

void RmdCAN::motorOff() {
  active = false;
  sendCmd(RmdCmd::motor_shutdown);
  state = RmdLocalState::IDLE;
}

void RmdCAN::stopMotor() {
  active = false;
  this->setTorque(0.0);
  this->Delay(10);
  this->motorOff();
}

void RmdCAN::startMotor() {
  if (!suspend) {
    active = true;
    state  = RmdLocalState::RUNNING;
    this->Delay(1);
    this->setTorque(0.0);
  }
}

void RmdCAN::suspendAxis(bool flag) { suspend = flag; }

bool RmdCAN::isSuspended() { return suspend; }

bool RmdCAN::motorReady() { return (this->connected && state == RmdLocalState::RUNNING); }

void RmdCAN::writePID() {
  uint8_t data[8] = {0};

  data[0] = (uint8_t)RmdCmd::write_pid_ram;  // Command

  data[2] = pidSettings.CurrentKp;   // Current Kp
  data[3] = pidSettings.CurrentKi;   // Current Ki
  data[4] = pidSettings.SpeedKp;     // Speed Kp
  data[5] = pidSettings.SpeedKi;     // Speed Ki
  data[6] = pidSettings.PositionKp;  // Position Kp
  data[7] = pidSettings.PositionKi;  // Position Ki

  this->sendMsg(data);
}

void RmdCAN::sendCmd(RmdCmd cmd) {
  uint8_t buffer[8] = {0};
  buffer[0]         = (uint8_t)cmd;
  this->sendMsg(buffer);
}

void RmdCAN::sendMsg(uint8_t *buffer) {
  CAN_tx_msg msg;

  msg.header.RTR   = CAN_RTR_DATA;
  msg.header.DLC   = 8;
  msg.header.IDE   = 0;
  msg.header.StdId = outgoing_base_id + canId;
  memcpy(&msg.data, buffer, 8 * sizeof(uint8_t));

  this->_debug.lastOutCmd     = buffer[0];
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
  this->_debug.lastInCmd     = rxBuf[0];

  uint16_t canNodeId = rxHeader->StdId - incoming_base_id;

  if (canNodeId != this->canId) {
    return;
  }
  RmdCmd cmd = static_cast<RmdCmd>(rxBuf[0]);

  lastCanMessage = HAL_GetTick();

  switch (cmd) {
    case RmdCmd::read_multiturn_position:  // 0x60
    {
      this->multiturnEncPos = buffer_get_int32(rxBuf, 4);
      float epos            = static_cast<float>(multiturnEncPos);
      float cpr             = static_cast<float>(getCpr());
      float curPos          = fractional(epos / cpr);
      // Need to check for noisy reporting. If the delta is greater than half a turn during the period,
      // consider it an error and return the last position instead.
      this->lastPos     = (abs(curPos - lastPos) > 0.5f) ? lastPos : curPos;
      this->lastPosTime = HAL_GetTick();
      this->posWaiting  = false;
      break;
    }

    case RmdCmd::read_multiturn_position_raw:  // 0x61
    {
      this->multiturnEncPosRaw = buffer_get_int32(rxBuf, 4);
      break;
    }

    case RmdCmd::read_multiturn_zero_offset:  // 0x62
    {
      this->multiturnEncZeroOffset = buffer_get_int32(rxBuf, 4);
      break;
    }

    case RmdCmd::system_operating_mode:  // 0x70
    {
      this->axisState = static_cast<RmdAxisState>(buffer_get_uint8(rxBuf, 7));
      break;
    }

    case RmdCmd::torque_control:  // 0xA1
    {
      this->currentTorque = buffer_get_int16(rxBuf, 2);
      this->currentSpeed  = buffer_get_int16(rxBuf, 4);
      break;
    }

    case RmdCmd::speed_control:  // 0xA2
    {
      /* 1 dps/LSB precision */
      this->lastSpeed = buffer_get_int16(rxBuf, 4);
      break;
    }

    case RmdCmd::abs_pos_control:  // 0xA4
    {
      /* 1 dps/LSB precision */
      this->lastSpeed = buffer_get_int16(rxBuf, 4);
      break;
    }

    case RmdCmd::inc_pos_control:  // 0xA8
    {
      /* 1 dps/LSB precision */
      this->lastSpeed = buffer_get_int16(rxBuf, 4);
      break;
    }

    case RmdCmd::read_singleturn_position:  // 0x90
    {
      this->singleturnEncPos        = buffer_get_int16(rxBuf, 2);
      this->singleturnEncPosRaw     = buffer_get_int16(rxBuf, 4);
      this->singleturnEncZeroOffset = buffer_get_int16(rxBuf, 6);
      break;
    }

    case RmdCmd::read_multiturn_angle:  // 0x92
    {
      /* 0.01 deg/LSB precision, leave as-is for now */
      this->lastAng = buffer_get_int32(rxBuf, 4);
      break;
    }

    case RmdCmd::read_singleturn_angle:  // 0x94
    {
      /* 0.01 deg/LSB precision, leave as-is for now */
      this->lastAng = buffer_get_int16(rxBuf, 6);
      break;
    }

    case RmdCmd::read_status_1:  // 0x9A
    {
      this->motor_error = static_cast<RmdError>(buffer_get_uint16(rxBuf, 6));
      if (motor_error != RmdError::none) {
        // state = RmdLocalState::IDLE;
      }
      lastVoltageUpdate = HAL_GetTick();
      this->lastVoltage = (float)buffer_get_uint16(rxBuf, 4);
      break;
    }

    case RmdCmd::read_pid:  // 0x30
    {
      pidSettings.CurrentKp  = rxBuf[2];
      pidSettings.CurrentKi  = rxBuf[3];
      pidSettings.SpeedKp    = rxBuf[4];
      pidSettings.SpeedKi    = rxBuf[5];
      pidSettings.PositionKp = rxBuf[6];
      pidSettings.PositionKi = rxBuf[7];
    }

    default:
      break;
  }
}

Encoder *RmdCAN::getEncoder() { return static_cast<Encoder *>(this); }

EncoderType RmdCAN::getEncoderType() { return EncoderType::absolute; }

void RmdCAN::setPos(int32_t pos) {
  // Only change encoder count internally as offset
  posOffset = lastPos;
  angOffset = lastAng;
}

float RmdCAN::getPos_f() {
  if (motorReady() && (!posWaiting || HAL_GetTick() - lastPosTime > 5)) {
    posWaiting = true;
    this->Delay(1);
    sendCmd(RmdCmd::read_multiturn_position);
  }
  return lastPos - posOffset;
}

int32_t RmdCAN::getPos() { return getPos_f() * getCpr(); }
inline uint32_t RmdCAN::getCpr() { return 16384; }

/**
 * Turn the motor with positive/negative power.
 * Range should be full signed 16 bit
 * A value of 0 should have no torque. The sign is the direction.
 */
void RmdCAN::turn(int16_t power) {
  float torque = ((float)power / (float)0x7FFF) * maxTorque;  // 0x7FFF is the int16_t max.
  this->Delay(1);
  this->setTorque(torque);  // Send the signed torque(max of 1.0, min of -1.0)
}

// Input must be between -1.0 and 1.0
void RmdCAN::setTorque(float torque) {
  if (motorReady()) {
    uint8_t buffer[8] = {0};
    buffer[0]         = (uint8_t)RmdCmd::torque_control;
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

void RmdCAN::moveToPosition(float posDegrees) {
  uint8_t buffer[8] = {0};
  buffer[0]         = (uint8_t)RmdCmd::abs_pos_control;  // 0xA4
  uint16_t maxspd   = 750;
  buffer_append_uint16(buffer, maxspd, 2);
  /* posDegrees is assumed to be in deg,
  and precision is 0.01 deg/LSB */
  int32_t absAng = (posDegrees * 100.0 + angOffset);
  buffer_append_int32(buffer, absAng, 4);
  sendMsg(buffer);
}

void RmdCAN::executeMotionPlan(uint64_t params) {
  // uint64_t test = 0x7fff7ff0000007ff;
  // Need to use the custom can id 0x400
  CAN_tx_msg msg;

  msg.header.RTR   = CAN_RTR_DATA;
  msg.header.DLC   = 8;
  msg.header.IDE   = 0;
  msg.header.StdId = motion_command_base_id + canId;

  buffer_append_uint64(msg.data, params);

  if (!port->sendMessage(msg)) {
    // Nothing.
  };
}

void RmdCAN::setBaudrates(uint8_t value) {
  uint8_t buffer[8] = {0};
  buffer[0]         = (uint8_t)RmdCmd::set_communication_baudrate;  // 0xB4
  buffer[7]         = value;
  sendMsg(buffer);
}

void RmdCAN::sendFunctionCmd(RmdFunctionControl fnc) {
  uint8_t buffer[8] = {0};
  buffer[0]         = (uint8_t)RmdCmd::function_command;  // 0x20
  buffer[1]         = (uint8_t)fnc;
  sendMsg(buffer);
}

void RmdCAN::writeAccelerationPlanParameters(float maxPosAccel, float maxPosDecel, float maxVelAccel,
                                             float maxVelDecel) {
  uint8_t buffer[8] = {0};
  buffer[0]         = (uint8_t)RmdCmd::write_acceleration;

  buffer[1] = 0x00;
  buffer_append_uint32(buffer, maxPosAccel, 4);
  sendMsg(buffer);

  this->Delay(1);
  buffer[1] = 0x01;
  buffer_append_uint32(buffer, maxPosDecel, 4);
  sendMsg(buffer);

  this->Delay(1);
  buffer[1] = 0x02;
  buffer_append_uint32(buffer, maxVelAccel, 4);
  sendMsg(buffer);

  this->Delay(1);
  buffer[1] = 0x03;
  buffer_append_uint32(buffer, maxVelDecel, 4);
  sendMsg(buffer);
}

CommandStatus RmdCAN::command(const ParsedCommand &cmd, std::vector<CommandReply> &replies) {
  switch (static_cast<RmdCAN_commands>(cmd.cmdId)) {
    // Different cases
    case RmdCAN_commands::single_pos:  // 0x90
      this->Delay(1);
      if (cmd.type == CMDtype::get) {
        sendCmd(RmdCmd::read_singleturn_position);
        replies.emplace_back(this->singleturnEncPos);
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::single_offset:  // helper command
      this->Delay(1);
      if (cmd.type == CMDtype::get) {
        replies.emplace_back(this->singleturnEncZeroOffset);
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::multi_ang:  // 0x92
      this->Delay(1);
      if (cmd.type == CMDtype::get) {
        sendCmd(RmdCmd::read_multiturn_angle);
        replies.emplace_back(this->lastAng);
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::single_ang:  // 0x94
      this->Delay(1);
      if (cmd.type == CMDtype::get) {
        sendCmd(RmdCmd::read_singleturn_angle);
        replies.emplace_back(this->lastAng / 9);
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::multi_pos:  // 0x60
      this->Delay(1);
      if (cmd.type == CMDtype::get) {
        sendCmd(RmdCmd::read_multiturn_position);
        replies.emplace_back(this->multiturnEncPos);
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::multi_pos_raw:  // 0x61
      this->Delay(1);
      if (cmd.type == CMDtype::get) {
        sendCmd(RmdCmd::read_multiturn_position_raw);
        replies.emplace_back(this->multiturnEncPosRaw);
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::multi_offset:  // 0x62
      this->Delay(1);
      if (cmd.type == CMDtype::get) {
        sendCmd(RmdCmd::read_multiturn_zero_offset);
        replies.emplace_back(this->multiturnEncZeroOffset);
      } else if (cmd.type == CMDtype::set) {
        sendCmd(RmdCmd::write_multiturn_position);
        this->Delay(1);
        sendCmd(RmdCmd::system_reset);
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::baudrate:
      this->Delay(1);
      if (cmd.type == CMDtype::set) {
        setBaudrates(static_cast<uint8_t>(cmd.val));
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::function:
      this->Delay(1);
      if (cmd.type == CMDtype::set) {
        sendFunctionCmd(static_cast<RmdFunctionControl>(cmd.val));
        this->Delay(1);
        sendCmd(RmdCmd::system_reset);
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::start:
      this->Delay(1);
      if (cmd.type == CMDtype::set) {
        suspendAxis(false);
        state = RmdLocalState::START_RUNNING;
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::stop:
      this->Delay(1);
      if (cmd.type == CMDtype::set) {
        suspendAxis(true);
        stopMotor();
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::pid:  // 0x30 / 0x31
      this->Delay(1);
      if (cmd.type == CMDtype::get) {
        sendCmd(RmdCmd::read_pid);
        replies.emplace_back((uint64_t)(pidSettings));
      } else if (cmd.type == CMDtype::set) {
        pidSettings = (uint64_t)cmd.val;
        writePID();
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::brake:
      this->Delay(1);
      if (cmd.type == CMDtype::set) {
        if (cmd.val == 1) {
          sendCmd(RmdCmd::system_brake_lock);
        } else {
          sendCmd(RmdCmd::system_brake_release);
        }
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::motion:
      this->Delay(1);
      if (cmd.type == CMDtype::setat) {
        // Combine the two int64_t values into one uint64_t value
        this->motionBuffer = static_cast<uint64_t>(((uint64_t)cmd.val) | ((uint64_t)(cmd.adr << 32)));
        executeMotionPlan(motionBuffer);
      } else if (cmd.type == CMDtype::get) {
        replies.emplace_back((uint64_t)this->motionBuffer);
      } else {
        return CommandStatus::ERR;
      }
      break;

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
      this->Delay(1);
      if (cmd.type == CMDtype::get) {
        replies.emplace_back((uint32_t)state);
      } else if (cmd.type == CMDtype::set) {
        state = static_cast<RmdLocalState>(cmd.val);
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::maxtorque:
      this->Delay(1);
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
      this->Delay(1);
      if (cmd.type == CMDtype::get) {
        replies.emplace_back(connected ? 1 : 0);
      }
      break;

    case RmdCAN_commands::voltage:
      this->Delay(1);
      if (cmd.type == CMDtype::get) {
        replies.emplace_back(lastVoltage);
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::pos_turns:
      this->Delay(1);
      if (cmd.type == CMDtype::get) {
        replies.emplace_back(lastPos * 10000);
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::pos_turns_offset:
      this->Delay(1);
      if (cmd.type == CMDtype::get) {
        replies.emplace_back(posOffset * 10000);
      } else if (cmd.type == CMDtype::set) {
        setPos(0);
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::torque:
      this->Delay(1);
      if (cmd.type == CMDtype::get) {
        replies.emplace_back(this->currentTorque);
      } else {
        return CommandStatus::ERR;
      }
      break;

    case RmdCAN_commands::abspos:
      this->Delay(1);
      if (cmd.type == CMDtype::get) {
        replies.emplace_back(lastAng - angOffset);
      } else if (cmd.type == CMDtype::set) {
        moveToPosition(cmd.val);
      }
      break;

    case RmdCAN_commands::incpos:
      if (cmd.type == CMDtype::get) {
        break;
      } else if (cmd.type == CMDtype::set) {
        if (motorReady()) {
          uint8_t buffer[8] = {0};
          buffer[0]         = (uint8_t)RmdCmd::inc_pos_control;
          uint16_t maxspd   = 250;
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
      this->Delay(1);
      if (cmd.type == CMDtype::get) {
        replies.emplace_back(this->currentSpeed);
      } else if (cmd.type == CMDtype::set) {
        if (motorReady()) {
          uint8_t buffer[8] = {0};
          buffer[0]         = (uint8_t)RmdCmd::speed_control;
          /* cmd.val is assumed to be in dps,
          and precision is 0.01 dps/LSB */
          int32_t spd = cmd.val * 100;
          buffer_append_int32(buffer, spd, 4);
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

inline float clamp(float x, float upper, float lower) { return std::min(upper, std::max(x, lower)); }

inline float fractional(float x) { return std::modf(x, nullptr); }

float normalize(float input, float min, float max) {
  float average      = (min + max) / 2;
  float range        = (max - min) / 2;
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

void buffer_append_uint64(uint8_t *buffer, uint64_t number) {
  buffer[0] = number >> 56;
  buffer[1] = number >> 48;
  buffer[2] = number >> 40;
  buffer[3] = number >> 32;
  buffer[4] = number >> 24;
  buffer[5] = number >> 16;
  buffer[6] = number >> 8;
  buffer[7] = number;
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

/* From StackOverflow
https://stackoverflow.com/questions/43113387/conversion-of-long-long-to-byte-array-and-back-in-c
*/
// Convert byte array to long long
uint64_t ByteArrayToInt(const uint8_t *buffer, int length) {
  uint64_t recoveredValue = 0;
  for (int i = 0; i < length; i++) {
    auto byteVal = ((uint64_t)(buffer[i]) << (8 * i));
    recoveredValue |= byteVal;
  }
  return recoveredValue;
}

// Convert long long to byte array
void IntToByteArray(const uint64_t value, uint8_t *buffer, int length) {
  for (int i = 0; i < length; i++) {
    buffer[i] = ((value >> (8 * i)) & 0XFF);
  }
}

#endif
