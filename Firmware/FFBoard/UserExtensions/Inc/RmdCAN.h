/*
 * RmdCAN.h
 *
 *  Created on: October 19, 2022
 *      Author: Yannick
 * 				1Plus2Equals3D
 */


#ifndef USEREXTENSIONS_SRC_RMDCAN_H_
#define USEREXTENSIONS_SRC_RMDCAN_H_

#include "MotorDriver.h"
#include "cpp_target_config.h"
#include "CAN.h"
#include "Encoder.h"
#include "thread.hpp"
#include "CanHandler.h"
#include "CommandHandler.h"
#include "PersistentStorage.h"

#ifdef RMD
#define RMD_THREAD_MEM 512
#define RMD_THREAD_PRIO 25 // Must be higher than main thread

/*              ***** Data *****             */

enum class RmdControlMode : uint8_t {
    idle = 0x00,
    current_loop = 0x01,
    speed_loop = 0x02,
    position_loopmode = 0x03
};

enum class RmdLocalState : uint32_t {
    IDLE,
    RUNNING,
    START_RUNNING
};

enum class RmdError : uint16_t {
    none = 0x0000,
    motor_stall = 0x0002,
    low_pressure = 0x0004,
    overvoltage = 0x0008,
    overcurrent = 0x0010,
    power_overrun = 0x0040,
    speeding = 0x0100,
    motor_over_temperature = 0x1000,
    encoder_calibration_error = 0x2000
    /*
    AXIS_ERROR_NONE = 0x00000000,
    AXIS_ERROR_INVALID_STATE  = 0x00000001,
    AXIS_ERROR_WATCHDOG_TIMER_EXPIRED = 0x00000800,
    AXIS_ERROR_MIN_ENDSTOP_PRESSED = 0x00001000,
    AXIS_ERROR_MAX_ENDSTOP_PRESSED = 0x00002000,
    AXIS_ERROR_ESTOP_REQUESTED = 0x00004000,
    AXIS_ERROR_HOMING_WITHOUT_ENDSTOP = 0x00020000,
    AXIS_ERROR_OVER_TEMP = 0x00040000,
    AXIS_ERROR_UNKNOWN_POSITION = 0x00080000
    */
};



enum class RmdCmd : uint8_t
{
    /* */
    // Simple commands to place in first position in the uint8_t[8] buffer of a can messsage. These trigger
    // A simple reaction or reply
    motor_stop = 0x81,
    motor_off = 0x80,
    read_status_1 = 0x9A,
    read_status_2 = 0x9C,
    read_status_3 = 0x9D,
    read_home_position = 0x61, //w/o zero offset
    read_multiturn_position = 0x60,
    read_multiturn_angle = 0x92,
    read_acceleration = 0x42,
    read_pid = 0x30,
    get_software_version = 0xB2,
    System_operating_mode_acquisition = 0x70,
    // Complex commands that require changing more than just the first item in the buffer array, see protocol
    torque_closed_loop = 0xA1, //data in [4] and [5]
};

// Internal commands
enum class RmdCAN_commands : uint32_t{
    canid,canspd,error,state,maxtorque,connected,voltage
};


/*                    Main Class                              */


class RmdCAN : public MotorDriver,public PersistentStorage, public Encoder, public CanHandler, public CommandHandler, cpp_freertos::Thread{
public:
    RmdCAN(uint8_t id);
    virtual ~RmdCAN();
    const ClassIdentifier getInfo() = 0;
    void Run();


    // Overrides
    void turn(int16_t power) override;
    void stopMotor() override;
    void startMotor() override;
    Encoder* getEncoder() override;
    bool hasIntegratedEncoder() {return true;}
    bool motorReady() override;
       float getPos_f() override;
    uint32_t getCpr() override;
    int32_t getPos() override;
    void setPos(int32_t pos) override;
    EncoderType getEncoderType() override;
       void canRxPendCallback(CAN_HandleTypeDef *hcan,uint8_t* rxBuf,CAN_RxHeaderTypeDef* rxHeader,uint32_t fifo) override;
    void saveFlash() override; 		// Write to flash here
    void restoreFlash() override;	// Load from flash
    CommandStatus command(const ParsedCommand& cmd,std::vector<CommandReply>& replies) override;

    void sendCmd(RmdCmd cmd);
    void sendMsg(uint8_t *buffer);

    void motorOff();
    void setTorque(float torque);
    void readyCb();

    void setMode(RmdControlMode controlMode);

    void registerCommands();

    std::string getHelpstring(){return "Rmd motor driver with CAN";};


private:
    CANPort* port = &canport;
    float lastPos = 0;
    float lastSpeed = 0;
    float posOffset = 0;
    float lastVoltage = 0;
    uint32_t lastVoltageUpdate = 0;
    uint32_t lastCanMessage = 0;

    int8_t motorId = 0;

    uint16_t outgoing_base_id = 0x140; // Add 1 for motor 1, 2 for motor2
    uint16_t incoming_base_id = 0x240; // Add 1 for motor 1, 2 for motor2

    uint8_t baudrate = CANSPEEDPRESET_500; // 250000, 500000, 1M
    int32_t filterId = 0;
    bool connected = false;

    float base_amps = 10; // in amps, 1 Amp base unit, must be sent as 0.01a out to the motors, so must be *100
    float maxTorque = 1.0; // range how to scale the torque output
    bool active = false;

    volatile RmdLocalState state = RmdLocalState::IDLE;

      volatile RmdError motor_error = RmdError::none; 
    // Not yet used by rmd (0.5.4):
    volatile uint32_t rmdMotorFlags = 0;
    volatile uint32_t rmdEncoderFlags = 0;
    volatile uint32_t rmdControllerFlags = 0;

};


/*                  Instance Creation                         */
/**
 * Instance 1 of Rmd
 * Use for M0 output
 */
class RmdCAN1 : public RmdCAN{
public:
    RmdCAN1() : RmdCAN{0} {inUse = true;}
    const ClassIdentifier getInfo();
    ~RmdCAN1(){inUse = false;}
    static bool isCreatable();
    static ClassIdentifier info;
    static bool inUse;
};

/**
 * Instance 2 of Rmd
 * Use for M1 output
 */
class RmdCAN2 : public RmdCAN{
public:
    RmdCAN2() : RmdCAN{1} {inUse = true;}
    const ClassIdentifier getInfo();
    ~RmdCAN2(){inUse = false;}
    static bool isCreatable();
    static ClassIdentifier info;
    static bool inUse;
};



/*                ***** Helper Functions *****                */

float normalize(float input, float min, float max);
void buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t index);

void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t index); 
void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t index);
void buffer_append_uint32(uint8_t *buffer, uint32_t number, int32_t index);

uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t index);
uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t index);
int32_t buffer_get_int32(const uint8_t *buffer, int32_t index);
int16_t buffer_get_int16(const uint8_t *buffer, int32_t index);

float buffer_get_float32(const uint8_t *buffer, float scale, int32_t index);
float buffer_get_float16(const uint8_t *buffer, float scale, int32_t index);

#endif
#endif