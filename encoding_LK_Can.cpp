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
    uint64_t Lh=0;
	uint64_t Ll=0;
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

struct re_val packmsg(char cmd, uint8_t id, uint16_t v,uint16_t Pvalue){
    struct re_val r;
    uint8_t dataSize = 0;	// 命令数据长度
	int16_t openCtlData = 0;	// 开环控制数据
	int16_t torqueCtlData = 0;	// 力矩环控制数据
	int16_t speedCtlData = 0;	// 速度环控制数据
	int32_t angleCtlData = 0;	// 位置环控制数据
	uint8_t Buffer[20];


	if (cmd == '0')
	{
		if (Pvalue<0)
			{r.Lh=(0xA601<<8)&v;
			Pvalue=-Pvalue;}
		else
			r.Lh=(0xA600<<8)&v;
		r.Ll=Pvalue<<8;
	}
    return r;
}

int main(){
   unsigned char b[4]={0x00,0x00,0x00,0xbe};
    struct re_val r=packmsg('p',1,0,30000);
    cout<<hex<<r.L1<<endl<<r.L3;

 return 0;
}
//g++ -o encoding_LK.dll -shared encoding_LK.cpp 145531359052480133 939548549