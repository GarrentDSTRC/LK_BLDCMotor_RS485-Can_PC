#include<iostream>
#include<string.h>
#include <stdio.h>

using namespace std;

 // 最小帧长
#define LEAST_FRAME_SIZE					  5		// 最小帧长度 header + cmd + id + dataLength + headCheckSum										
// HEAD                                 	
#define CMD_HEAD							      0x3E	// 头


#define CMD_OPEN_CONTROL						0xA0	// 开环控制MS
#define CMD_TORQUE_CONTROL					0xA1	// 力矩控制MF MG
#define CMD_SPEED_CONTROL						0xA2	// 速度控制
#define CMD_ANGLE_CONTROL1					0xA3	// 位置控制


extern uint8_t ctlCmd;
extern uint8_t motorId;
extern int64_t ctlValue;
extern uint32_t commandSendCount;
extern uint32_t commandUnreceivedCount;

extern int8_t motorTemperature;
extern int16_t motorPowerOrTorque;
extern int16_t motorSpeed;
extern uint16_t motorEncoder;


void control_Run(void);
void command_Run(void);
void status_Run(void);
void control_Send(uint8_t cmd, uint8_t id, int64_t value);
void control_Receive(void);
uint8_t control_CheckReceivedData(void);
void control_PackCmd(uint8_t *buffer, uint8_t cmd, uint8_t id, uint8_t dataSize, uint8_t *data);



