/*
 * RmdCAN.h
 *
 *  Created on: October 19, 2022
 *      Author: Yannick
 * 				1Plus2Equals3D
 *              PilotWave11
 */

#ifndef USEREXTENSIONS_SRC_RMDCAN_H_
#define USEREXTENSIONS_SRC_RMDCAN_H_

#include "CAN.h"
#include "CanHandler.h"
#include "CommandHandler.h"
#include "Encoder.h"
#include "MotorDriver.h"
#include "PersistentStorage.h"
#include "cpp_target_config.h"
#include "thread.hpp"

#ifdef RMD
#define RMD_THREAD_MEM 512
#define RMD_THREAD_PRIO 25  // Must be higher than main thread

uint64_t ByteArrayToInt(const uint8_t *buffer, int length);
void IntToByteArray(const uint64_t value, uint8_t *buffer, int length);
struct RmdPIDSettings {

  // Conversion to uint64_t
  explicit operator uint64_t() {
    uint8_t buffer[6] = {0};
    buffer[0]         = CurrentKp;
    buffer[1]         = CurrentKi;
    buffer[2]         = SpeedKp;
    buffer[3]         = SpeedKi;
    buffer[4]         = PositionKp;
    buffer[5]         = PositionKi;
    uint64_t result   = ByteArrayToInt(buffer, 6);
    return result;
  }

  RmdPIDSettings() = default;
   // Converting ctor from uint64_t
  RmdPIDSettings(uint64_t val) {
    uint8_t buffer[6] = {0};
    IntToByteArray(val, buffer, 6);
    CurrentKp  = buffer[0];
    CurrentKi  = buffer[1];
    SpeedKp    = buffer[2];
    SpeedKi    = buffer[3];
    PositionKp = buffer[4];
    PositionKi = buffer[5];
  }

  uint8_t CurrentKp;   // Current Kp
  uint8_t CurrentKi;   // Current Ki
  uint8_t SpeedKp;     // Speed Kp
  uint8_t SpeedKi;     // Speed Ki
  uint8_t PositionKp;  // Position Kp
  uint8_t PositionKi;  // Position Ki
};

enum class RmdAxisState : uint8_t {
  IDLE                 = 0x00,
  CURRENT_CLOSED_LOOP  = 0x01,
  SPEED_CLOSED_LOOP    = 0x02,
  POSITION_CLOSED_LOOP = 0x03
};

enum class RmdFunctionControl : uint8_t {
  clearMultiturnValue = 0x01,
  canFilterIdEnable   = 0x02
};

enum class RmdLocalState : uint32_t {
  IDLE          = 0x00,
  WAIT_READY    = 0x01,
  START_RUNNING = 0x02,
  RUNNING       = 0x03
};

enum class RmdError : uint16_t {
  none                      = 0x0000,
  motor_stall               = 0x0002,
  low_pressure              = 0x0004,
  overvoltage               = 0x0008,
  overcurrent               = 0x0010,
  power_overrun             = 0x0040,
  speeding                  = 0x0100,
  motor_over_temperature    = 0x1000,
  encoder_calibration_error = 0x2000
};

enum class RmdCmd : uint8_t {
  none                        = 0x00,
  read_pid                    = 0x30,
  write_pid_ram               = 0x31,
  write_pid_rom               = 0x32,
  read_acceleration           = 0x42,
  write_acceleration          = 0x43,
  read_multiturn_position     = 0x60,
  read_multiturn_position_raw = 0x61,  // w/o zero offset
  read_multiturn_zero_offset  = 0x62,
  write_multiturn_value       = 0x63,
  write_multiturn_position    = 0x64,
  read_singleturn_position    = 0x90,
  read_multiturn_angle        = 0x92,
  read_singleturn_angle       = 0x94,
  read_status_1               = 0x9A,
  read_status_2               = 0x9C,
  read_status_3               = 0x9D,
  motor_shutdown              = 0x80,
  motor_stop                  = 0x81,
  torque_control              = 0xA1,
  speed_control               = 0xA2,
  abs_pos_control             = 0xA4,
  singleturn_pos_control      = 0xA6,
  inc_pos_control             = 0xA8,
  system_operating_mode       = 0x70,
  motor_power                 = 0x71,
  system_reset                = 0x76,
  system_brake_release        = 0x77,
  system_brake_lock           = 0x78,
  runtime_read                = 0xB1,
  read_software_version       = 0xB2,
  set_communication_interrupt = 0xB3,
  set_communication_baudrate  = 0xB4,
  read_motor_model            = 0xB5,
  function_command            = 0x20,
  set_canid                   = 0x79
};

// Internal commands
enum class RmdCAN_commands : uint32_t {
  canid,
  canspd,
  error,
  state,
  maxtorque,
  connected,
  voltage,
  baudrate,
  pos_turns,
  pos_turns_offset,
  single_pos,
  single_offset,
  single_ang,
  multi_ang,
  multi_pos,
  multi_pos_raw,
  multi_offset,
  function,
  start,
  stop,
  pid,
  motion,
  brake,
  abspos,
  incpos,
  spd,
  trackpos,
  torque,
  debug
};

struct RmdFlashAddrs {
  uint16_t canId     = ADR_RMD_CANID_M0;
  uint16_t maxTorque = ADR_RMD_MAXTORQUE_M0;
  uint16_t offset    = ADR_RMD_OFFSET_M0;
};

// DEBUG START
struct RmdDebug {
  uint32_t lastOutAddress;
  uint32_t lastInAddress;
  uint8_t lastInCmd;
  uint8_t lastOutCmd;
};
// DEBUG STOP

class RmdCAN : public MotorDriver,
               public PersistentStorage,
               public Encoder,
               public CanHandler,
               public CommandHandler,
               cpp_freertos::Thread {
 public:
  RmdCAN(uint8_t id);
  virtual ~RmdCAN();
  void setAddress(uint8_t id);

  const ClassIdentifier getInfo() = 0;
  void Run();

  // Overrides
  void turn(int16_t power) override;
  void stopMotor() override;
  void startMotor() override;
  Encoder *getEncoder() override;
  bool hasIntegratedEncoder() { return true; }
  bool motorReady() override;
  float getPos_f() override;
  uint32_t getCpr() override;
  int32_t getPos() override;
  void setPos(int32_t pos) override;
  EncoderType getEncoderType() override;
  void canErrorCallback(CAN_HandleTypeDef *hcan);
  void canRxPendCallback(CAN_HandleTypeDef *hcan, uint8_t *rxBuf, CAN_RxHeaderTypeDef *rxHeader,
                         uint32_t fifo) override;
  void saveFlash() override;     // Write to flash here
  void restoreFlash() override;  // Load from flash
  CommandStatus command(const ParsedCommand &cmd, std::vector<CommandReply> &replies) override;

  void suspendAxis(bool flag);
  bool isSuspended();
  void getPosOffset();

  void sendCmd(RmdCmd cmd);
  void sendFunctionCmd(RmdFunctionControl fnc);
  void sendMsg(uint8_t *buffer);
  void setBaudrates(uint8_t value);

  void motorOff();
  void setTorque(float torque);
  void moveToPosition(float posDegrees);
  void executeMotionPlan(uint64_t params);
  void writeAccelerationPlanParameters(float maxPosAccel, float maxPosDecel, float maxVelAccel, float maxVelDecel);
  void readyCb();
  void writePID();

  void setMode(RmdAxisState controlMode);

  void registerCommands();

  void setCanFilter();

  std::string getHelpstring() { return "Rmd motor driver with CAN"; };

 private:
  RmdFlashAddrs flashAddrs;

  int32_t multiturnEncPos        = 0; /* counts */
  int32_t multiturnEncPosRaw     = 0; /* counts */
  int32_t multiturnEncZeroOffset = 0; /* counts */

  int16_t singleturnEncPos        = 0; /* counts */
  int16_t singleturnEncPosRaw     = 0; /* counts */
  int16_t singleturnEncZeroOffset = 0; /* counts */

  uint64_t motionBuffer = 0; /* Used to capture motion data before it is sent */

  RmdPIDSettings pidSettings;

  int32_t lastAng   = 0; /* deg */
  int32_t angOffset = 0; /* deg */

  int16_t currentTorque = 0;
  int16_t currentSpeed = 0;

  CANPort *port   = &canport;
  float lastPos   = 0; /* turns */
  float posOffset = 0; /* turns */

  float lastSpeed            = 0; /* dps */
  float lastVoltage          = 0;
  uint32_t lastVoltageUpdate = 0;
  uint32_t lastCanMessage    = 0;

  uint32_t lastPosTime = 0;
  bool posWaiting      = false;

  int8_t motorId   = 0;
  uint16_t canId   = 0;
  int32_t filterId = 0;

  uint16_t outgoing_base_id       = 0x140;
  uint16_t incoming_base_id       = 0x240;
  uint16_t motion_command_base_id = 0x400;  // Used for special motion command
  uint16_t multimotor_command_id  = 0x280;  // Used for simultaneous multi-motor control.

  uint8_t baudrate = CANSPEEDPRESET_500;  // 250000, 500000, 1M

  bool connected = false;
  bool active    = false;

  float maxTorque = 0.0;  // range how to scale the torque output

  volatile bool suspend = true;

  volatile RmdLocalState state    = RmdLocalState::IDLE;
  volatile RmdAxisState axisState = RmdAxisState::IDLE;

  volatile RmdError motor_error = RmdError::none;
  // Not yet used by rmd (0.5.4):
  volatile uint32_t rmdMotorFlags      = 0;
  volatile uint32_t rmdEncoderFlags    = 0;
  volatile uint32_t rmdControllerFlags = 0;

  RmdDebug _debug;
};

/*                  Instance Creation                         */
/**
 * Instance 1 of Rmd
 * Use for M0 output
 */
class RmdCAN1 : public RmdCAN {
 public:
  RmdCAN1() : RmdCAN{0} { inUse = true; }
  const ClassIdentifier getInfo();
  ~RmdCAN1() { inUse = false; }
  static bool isCreatable();
  static ClassIdentifier info;
  static bool inUse;
};

/**
 * Instance 2 of Rmd
 * Use for M1 output
 */
class RmdCAN2 : public RmdCAN {
 public:
  RmdCAN2() : RmdCAN{1} { inUse = true; }
  const ClassIdentifier getInfo();
  ~RmdCAN2() { inUse = false; }
  static bool isCreatable();
  static ClassIdentifier info;
  static bool inUse;
};

/*                ***** Helper Functions *****                */
float normalize(float input, float min, float max);
void buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t index);

void buffer_append_uint64(uint8_t *buffer, uint64_t number);
void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t index);
void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t index);
void buffer_append_uint32(uint8_t *buffer, uint32_t number, int32_t index);
void buffer_append_uint16(uint8_t *buffer, uint16_t number, int32_t index);

uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t index);
uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t index);
uint8_t buffer_get_uint8(const uint8_t *buffer, int32_t index);
int32_t buffer_get_int32(const uint8_t *buffer, int32_t index);
int16_t buffer_get_int16(const uint8_t *buffer, int32_t index);

float buffer_get_float32(const uint8_t *buffer, float scale, int32_t index);
float buffer_get_float16(const uint8_t *buffer, float scale, int32_t index);

#endif
#endif
