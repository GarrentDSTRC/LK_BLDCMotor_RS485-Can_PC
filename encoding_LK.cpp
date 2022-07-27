#include<iostream>
#include<string.h>
#include <stdio.h>
#include "encoding_LK.h"
using namespace std;

 extern "C"
{
struct re_val packmsg(char cmd, uint8_t id, uint8_t d,uint16_t value);
} 

uint8_t uart1TxDataSize=0;

struct re_val {
    uint64_t L1=0;
    uint32_t L2=0;
    uint16_t L3=0;
};

int ustrcat(unsigned char *msg,unsigned char *buf,int point,int len){
    for (int i=0;i<(len);i++)
        {*(msg+point)=*(buf+i);
        point++;} //指向没写的空位
    return point;
}

void control_PackCmd(uint8_t *buffer, uint8_t cmd, uint8_t id, uint8_t size, uint8_t *data)
{
	uint8_t i = 0;

	buffer[0] = CMD_HEAD;
	buffer[1] = cmd;
	buffer[2] = id;
	buffer[3] = size;
	buffer[4] = 0;	// 需要先清0
	for (i=0; i<4; i++)
		buffer[4] += buffer[i];
	
	if (size != 0)
	{
		buffer[LEAST_FRAME_SIZE+size] = 0;	// 需要先清0
		for (i=0; i<size; i++)	// 复制数据并计算校验值
		{
			buffer[LEAST_FRAME_SIZE+i] = data[i];
			buffer[LEAST_FRAME_SIZE+size] += buffer[LEAST_FRAME_SIZE+i];
		}
		uart1TxDataSize = i + LEAST_FRAME_SIZE + 1;	// 需要发送的数据总长度
	}
	else
		uart1TxDataSize = LEAST_FRAME_SIZE ;	// 需要发送的数据总长度
}

struct re_val packmsg(char cmd, uint8_t id, uint8_t d,uint16_t value){
    struct re_val r;
    uint8_t dataSize = 0;	// 命令数据长度
	int16_t openCtlData = 0;	// 开环控制数据
	int16_t torqueCtlData = 0;	// 力矩环控制数据
	int16_t speedCtlData = 0;	// 速度环控制数据
	int32_t angleCtlData = 0;	// 位置环控制数据
	uint8_t Buffer[20];


	if (cmd == 'p')
	{
		//数据预处理
        uint8_t angleCtlData[4];
        angleCtlData[0]=d;
        angleCtlData[1]=value&0xFF;
        angleCtlData[2]=value>>8&0xFF;
		angleCtlData[3]=0x00;
		cmd=0xA5;
        
		dataSize = 4;
		
		control_PackCmd( Buffer, cmd, id, dataSize, angleCtlData);

        for(int i=0;i<8;i++){
            r.L1=(r.L1<<8)+Buffer[i];
        }
        for(int i=8;i<10;i++){
            r.L3=(r.L3<<8)+Buffer[i];
        }
        //cout<<hex<<"L1"<<r.L1<<endl<<"L3"<<r.L3<<endl;
	}

//早知道不能传指针出去 我就应该直接定义基于longlong的struct堆栈
    return r;
}
int main(){
   unsigned char b[4]={0x00,0x00,0x00,0xbe};
    struct re_val r=packmsg('p',1,0,30000);
    cout<<hex<<r.L1<<endl<<r.L3;

 return 0;
}
//g++ -o encoding_LK.dll -shared encoding_LK.cpp 145531359052480133 939548549