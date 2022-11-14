#include<iostream>
#include<string.h>
#include <stdio.h>
#include "encoding_LK.h"
using namespace std;
//尽量用uint_16
 extern "C"
{
struct re_val packmsg(uint16_t cmd,bool dir, uint8_t id,uint16_t v,uint16_t Pvalue);
struct decoder motorRead(uint64_t Read);
} 
struct decoder{
	uint16_t P=0;
	uint16_t V=0;
	uint16_t I=0;
	uint8_t T=0;
};

struct re_val {
    uint64_t Lh=0;
	uint64_t Ll=0;
};

struct decoder motorRead(uint64_t Read){
	struct decoder d;
	d.P=((0xFF&Read)<<8)+(Read>>8)&0xFF;
	Read=Read>>16;
	d.V=((0xFF&Read)<<8)+(Read>>8)&0xFF;
	Read=Read>>16;
	d.I=((0xFF&Read)<<8)+(Read>>8)&0xFF;
	Read=Read>>16;
	d.T=0xFF&Read;
	return d;
}

struct re_val packmsg(uint16_t cmd,bool dir, uint8_t id, uint16_t v,uint16_t Pvalue){
    struct re_val r;

	cout<<"Pvalue:"<<Pvalue<<"V:"<<v<<"dir"<<dir<<"id"<<id<<"cmd"<<cmd<<endl;

	if (cmd == 0)
	{
		if (dir==1)
			r.Lh=0xA601;
		else
			r.Lh=0xA600;
		r.Lh=(r.Lh<<16)+(v&0xFF)+((v>>8)&0xFF);
		r.Lh=(r.Lh<<32)+(( ((Pvalue&0xFF)<<8) +(Pvalue>>8) )<<16);
	}
    return r;
}

int main(){
   unsigned char b[4]={0x00,0x00,0x00,0xbe};
    struct re_val r=packmsg(0,0,0,256,300);
	uint16_t A;
	A=(0xFFFF& r.Lh);
    cout<<hex<<r.Lh<<endl<<(0xFFFF&r.Lh);


 return 0;
}
//g++ -o encoding_LK_Can.dll -shared encoding_LK_Can.cpp 145531359052480133 939548549